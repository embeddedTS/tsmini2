/* This program takes the 40 MBytes/sec of samples from the TS-MINI ADC and
 * sends it to stdout.  It uses realtime priority and locks 512 MBytes of RAM
 * for a software FIFO.  If stdout can not keep up with 40MBytes/sec, the 512
 * Mbyte software FIFO will eventually overflow.  When it does, it will stop
 * acquiring samples from the TS-MINI, flush the remaining 512MBytes of sample
 * backlog, and then terminate gracefully with exit state 1.  If stdout can 
 * keep up with 40MBytes/sec, the sample stream outputs forever or until EOF 
 * or signal termination.
 *
 * Example usage:
 *   nc -e ./tsmini2 192.168.1.30 1234
 *       - Use the NetCat utility to initiate a TCP connection to IP 
 *         192.168.1.30, port 1234 and send samples.
 *
 *   ./tsmini2 > samples.out
 *       - Capture samples to file storage for later processing.  
 *
 *   ./tsmini2 | some_filter_process > samples.out
 *       - Same as above, but send samples through a custom util 
 *         named "some_filter_process".  Processes in pipelines allow multiple
 *         CPUs to be utilized in parallel.
 *
 *   nc -l 1234 -e ./tsmini2
 *       - Use the NetCat utility to listen for an incoming TCP connection on
 *         port 1234 and send samples to the remote system.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <assert.h>
#include <sched.h>
#include <pthread.h>
#include <getopt.h>

#define BUFSIZE (512 * 0x100000)
#define MAX_WRITE 0x200000
#define MAX_LATENCY_US 100000

static void *fpga, *dmabuf;
static uint32_t reg10h;
static uint32_t dmabuf_phys;
static uint32_t last, get;
static volatile uint32_t put, nf;
static uint8_t *buf;
static pthread_mutex_t buflock;
static pthread_cond_t bufcond = PTHREAD_COND_INITIALIZER;

static uint8_t read_spi_byte(void) {
  uint32_t n = 0;
  uint8_t ret;
  for (ret = n = 0; n < 8; n++) {
    *(volatile uint32_t *)(fpga + 0x10) = reg10h & ~(1<<17); /* clk lo */
    *(volatile uint32_t *)(fpga + 0x10) = reg10h | (1<<17); /* clk hi */
    ret = (ret << 1);
    if (*(volatile uint32_t *)(fpga + 0x10) & (1<<19)) ret |= 1;
  }
  return ret;
}

static void write_spi_byte(uint8_t x) {
  uint32_t n = 0;
  uint32_t v;
  for (n = 0; n < 8; n++) {
    if (x & 0x80) v = reg10h | (1<<16); else v = reg10h & ~(1<<16);
    *(volatile uint32_t *)(fpga + 0x10) = v & ~(1<<17); /* clk lo */
    *(volatile uint32_t *)(fpga + 0x10) = v | (1<<17); /* clk hi */
    x = x << 1;
  }
  return;
}

static void enable_cs(void) {
  reg10h &= ~(1<<18);
  *(volatile uint32_t *)(fpga + 0x10) = reg10h;
}

static void disable_cs(void) {
  reg10h |= (1<<18);
  *(volatile uint32_t *)(fpga + 0x10) = reg10h;
}

static void spi_one_byte_cmd(uint8_t cmd) {
  enable_cs();
  write_spi_byte(cmd);
  disable_cs();
}

static void spif_read(uint32_t adr, uint8_t *buf, uint32_t cnt) {
  enable_cs();
  write_spi_byte(3);
  write_spi_byte(adr >> 16);
  write_spi_byte(adr >> 8);
  write_spi_byte(adr);
  while(cnt--) *(buf++) = read_spi_byte();
  disable_cs();
}


static void spif_write(uint32_t adr, uint8_t *buf, uint32_t cnt) {
  uint32_t n = 0, i;
  uint8_t s;

  while (cnt > 256) {
    spif_write(adr, buf, 256);
    adr += 256;
    buf += 256;
    cnt -= 256;
  }

  for (i = 0; i < cnt; i++) if (buf[i] != 0xff) break;
  if (i == cnt) return; /* All 1's ! */

  spi_one_byte_cmd(6); /* Write enable */

  enable_cs();
  write_spi_byte(2); /* Page program */
  write_spi_byte(adr >> 16);
  write_spi_byte(adr >> 8);
  write_spi_byte(adr);
  while(cnt--) write_spi_byte(*(buf++));
  disable_cs();

  enable_cs();
  write_spi_byte(5); /* Read status register */
  do { s = read_spi_byte(); n++;} while ((s & 1) && n < 8192);
  disable_cs();
  if (n == 8192) {
    fprintf(stderr, "SPI flash chip write timeout!\n");
    exit(-1);
  }
  spi_one_byte_cmd(4); /* Write disable */
}

static void spif_erase(void) {
  uint8_t s;
  uint32_t n = 0;

  spi_one_byte_cmd(6); /* Write enable */
  spi_one_byte_cmd(0xc7); /* Chip erase */
  enable_cs();
  write_spi_byte(5); /* Read status register */
  do {
    s = read_spi_byte();
    n++;
  } while ((s & 1) && n < 3000000);
  disable_cs();
  if (n == 3000000) {
    fprintf(stderr, "SPI flash chip erase timeout!\n");
    exit(-1);
  }
  spi_one_byte_cmd(4); /* Write disable */
}

static uint32_t spif_rdid(void) {
  uint32_t r;

  disable_cs();
  spi_one_byte_cmd(0x66);
  spi_one_byte_cmd(0x99);
  enable_cs();
  write_spi_byte(0x9f);
  r = read_spi_byte();
  r |= read_spi_byte() << 8;
  r |= read_spi_byte() << 16;
  disable_cs();

  return r;
}

static int opt_save(char *arg) {
	int i, c;
	uint8_t *fpga_spif;
	uint32_t id, fpgarev;
	FILE *out = stdout;

	fpgarev = *(volatile uint32_t *)(fpga) & 0xff;
	assert(fpgarev >= 3);
	reg10h = *(volatile uint32_t *)(fpga + 0x10);
	fpga_spif = (uint8_t *)malloc(0x200000);
	assert(fpga_spif != NULL);

	fprintf(stderr, "Reading FPGA SPI flash chip ID...");
	id = spif_rdid();
	fprintf(stderr, "ID:0x%x, ", id);
	if (id == 0xffffff || id == 0) {
		fprintf(stderr, "Bad SPI flash chip ID!\n");
		return 2;
	} else fprintf(stderr, "ok\n");

	if (arg && strcmp(arg, "-") != 0) {
		fprintf(stderr, "Opening \"%s\" file for writing...", arg);
		out = fopen(arg, "w");
		if (out == NULL) {
			perror(arg);
			return 1;
		} else fprintf(stderr, "ok\n");
	} 

	for(i = 0; i < (0x200000 / 256); i++) {
		spif_read(i * 256, &fpga_spif[i * 256], 256);
		if ((i*256)>>18 != ((i-1)*256)>>18) 
		  fprintf(stderr, "Reading flash, %3d%% done...\n", 
		    (i*100/(0x200000 / 256)));
	}
	fprintf(stderr, "Reading flash, 100%% done...\n");
	
	for(i = 0x200000 - 1; i > 0; i--) if (fpga_spif[i] != 0xff) break;
	fprintf(stderr, "Writing %d bytes to output...", i + 1);
	fwrite(fpga_spif, i + 1, 1, out);
	fprintf(stderr, "ok\n");
	if (out != stdout) fclose(out);
}

static int opt_program(char *arg) {
	int i, c;
	uint8_t *fpga_spif;
	uint32_t id, fpgarev;
	FILE *in = stdin;

	fpgarev = *(volatile uint32_t *)(fpga) & 0xff;
	assert(fpgarev >= 3);
	reg10h = *(volatile uint32_t *)(fpga + 0x10);
	fpga_spif = (uint8_t *)malloc(0x200000);
	assert(fpga_spif != NULL);
	memset(fpga_spif, 0xff, 0x200000);

	if (arg && strcmp(arg, "-") != 0) {
		fprintf(stderr, "Opening \"%s\" file for reading...", arg);
		in = fopen(arg, "r");
		if (in == NULL) {
			perror(arg);
			return 1;
		} else fprintf(stderr, "ok, ");
	} else fprintf(stderr, "Reading program from stdin...");

	for (i = 0; i < 0x200000; i++) {
		c = fgetc(in);
		if (c == EOF) break;
		else fpga_spif[i] = c;
	}
	if (in != stdin) fclose(in);

	fprintf(stderr, "%d bytes\nReading FPGA SPI flash chip ID...", i);
	id = spif_rdid();
	fprintf(stderr, "ID:0x%x, ", id);
	if (id == 0xffffff || id == 0) {
		fprintf(stderr, "Bad SPI flash chip ID!\n");
		return 2;
	} else fprintf(stderr, "ok\n");
		
	fprintf(stderr, "Erasing...");
	spif_erase();
	fprintf(stderr, "ok\n");

	for(i = 0; i < (0x200000 / 256); i++) {
		spif_write(i * 256, &fpga_spif[i * 256], 256);
		if ((i*256)>>18 != ((i-1)*256)>>18) 
		  fprintf(stderr, "Writing flash, %3d%% done...\n", 
		    (i*100/(0x200000 / 256)));
	}
	printf("Writing flash, 100%% done...\n");

	for(i = 0; i < (0x200000 / 256); i++) {
		uint8_t buf[256];
		spif_read(i * 256, buf, 256);
		if ((i*256)>>18 != ((i-1)*256)>>18) 
		  fprintf(stderr, "Verifying flash, %3d%% done...\n", 
		    (i*100/(0x200000 / 256)));
		if (memcmp(&fpga_spif[i * 256], buf, 256) != 0) {
			fprintf(stderr, "Verify failed at %d%%!", 
			  i * 256 * 100 / 0x200000);
			return 3;
		}
	}
	fprintf(stderr, "Verifying flash, 100%% done...\n");
	free(fpga_spif);
	return 0;
}

void usage(char **argv) {
	fprintf(stderr, "Usage: %s [OPTION] ...\n"
	  "Technologic Systems TS-MINI PCIe card manipulation.\n"
	  "\n"
	  "  -i, --initdma=PHYS       Initialize 2MB DMA buffer at physical address PHYS\n"
	  "  -o, --initcn1=OUTPUTS    Initialize CN1 digital outputs to OUTPUTS\n"
	  "  -c, --config=VAL         Initialize config reg to VAL\n"
	  "  -p, --program=RPDFILE    Program new FPGA configuration flash from RPDFILE\n"
	  "  -s, --save=RPDFILE       Save existing FPGA flash config to RPDFILE\n"
	  "\n"
	  "By default, this program connects to the TS-MINI and sends 4x 16-bit channels\n" 
          "of raw binary analog data at 5 megasample/sec.\n", argv[0]);
}



static void buf_put(uint8_t *b, uint32_t len) {
	if (put + len <= BUFSIZE) memcpy(&buf[put], b, len);
	else {
		uint32_t n = BUFSIZE - put;
		memcpy(&buf[put], b, n);
		memcpy(buf, b + n, len - n);
	}
	put += len;
	if (put >= BUFSIZE) put -= BUFSIZE;
}

static void *fpga_loop(void *x) {
	uint32_t cur, n;
	struct sched_param sched;
	uint32_t sleep = 1000;

	/* Linux trick for improved realtime determinism: */
	sched.sched_priority = 99;
	pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched);

superloop:
	pthread_mutex_lock(&buflock);
	cur = *(volatile uint32_t *)(fpga + 8) - dmabuf_phys;
	if (cur & 1) {
		// Hard FIFO overflow; close stdout, we failed
		nf = UINT32_MAX;
		fprintf(stderr, "Linux realtime kernel bug detected!\n");
	} else {
		cur &= ~3;
	
		n = (cur - last) & 0x1fffff;
		if (BUFSIZE - nf <= n) n = (BUFSIZE - nf - 1) & ~0x7f;
	
		if (last + n > 0x200000) {
			uint32_t i = 0x200000 - last;
			buf_put(dmabuf + last, i);
			buf_put(dmabuf, n - i);
		} else buf_put(dmabuf + last, n);
	
		last = (last + n) & 0x1fffff;

		/* Wake up writer when FIFO moves from empty to not-empty */
		if (nf == 0 && n > 0) pthread_cond_signal(&bufcond);

		nf += n;

		/* Adaptive sleep attempts to wakeup when hard FIFO 3/4 full */
		if (n < 0x180000) sleep += 1000; else sleep -= 1000;
		if (sleep < 10000) sleep = 10000;
		else if (sleep > MAX_LATENCY_US) sleep = MAX_LATENCY_US;
	}

	// Soft FIFO overflow; close stdout, we failed
	if (nf >= BUFSIZE - 1 - 128) {
		nf = UINT32_MAX;
		pthread_mutex_unlock(&buflock);
		return (void *)1;
	} 

	pthread_mutex_unlock(&buflock);
	usleep(sleep);
	goto superloop;

	return NULL;
}

int main(int argc, char **argv) {
	ssize_t r;
	int fpgafd, memfd, c, regset = 0;
	pthread_mutexattr_t mattr;
	pthread_attr_t attr;
	pthread_t tid;
	fd_set wfds;
	char *opt_save_arg = NULL;
	char *opt_program_arg = NULL;
	static struct option long_options[] = {
	  { "program", 1, 0, 'p' },
	  { "save", 1, 0, 's' },
	  { "initdma", 1, 0, 'i' },
	  { "initcn1", 1, 0, 'o' },
	  { "config", 1, 0, 'c' },
	  { "help", 0, 0, 'h' },
	  { 0, 0, 0, 0}
	};

	fpgafd = open("/tsmini2/resource0", O_RDWR|O_SYNC);
	if (fpgafd == -1) {
		perror("/tsmini2/resource0");
		return 3;
	}

	fpga = mmap(0, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fpgafd, 0);
	assert (fpga != (void *)-1);

	while ((c = getopt_long(argc, argv, "c:o:i:s:p:h", long_options, NULL)) != -1) {
		switch(c) {
		case 'c':
			regset = 1;
			*(volatile uint32_t *)(fpga) = strtoul(optarg, NULL, 0);
			break;
		case 'o':
			regset = 1;
			*(volatile uint32_t *)(fpga + 0x10) = strtoul(optarg, NULL, 0);
			break;
		case 'i':
			regset = 1;
			*(volatile uint32_t *)(fpga + 4) = strtoul(optarg, NULL, 0);
			break;
		case 's':
			opt_save_arg = strdup(optarg);
			break;
		case 'p':
			opt_program_arg = strdup(optarg);
			break;
		case 'h':
		default:
			usage(argv);
			return 0;
		}
	}

	if (opt_save_arg) return opt_save(opt_save_arg);
	else if (opt_program_arg) return opt_program(opt_program_arg);
	else if (regset) return 0;

	memfd = open("/dev/udmabuf0", O_RDWR);
	if (memfd == -1) {
		perror("/dev/udmabuf0");
		return 3;
	}

	dmabuf = mmap(0, 0x200000, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, 0);
	assert (dmabuf != (void *)-1);

	buf = (uint8_t *)malloc(BUFSIZE);
	if (buf == NULL) {
		fprintf(stderr, "%s: Memory allocation failed\n", argv[0]);
		return 3;
	}
	nf = put = get = 0;

	/* Linux trick for improved realtime determinism: */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	memset(buf, 0, BUFSIZE);

	dmabuf_phys = *(uint32_t *)(fpga + 4);
	last = *(uint32_t *)(fpga + 8) - dmabuf_phys;
	last &= ~3;

	FD_ZERO(&wfds);
	pthread_mutexattr_init(&mattr);
	pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT);
	pthread_mutex_init(&buflock, &mattr);
	pthread_mutexattr_destroy(&mattr);
	pthread_mutex_lock(&buflock);
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 1024 * 32);
	pthread_create(&tid, &attr, fpga_loop, NULL);
	pthread_attr_destroy(&attr);
superloop:
	while (put != get) {
		if (put > get) r = put - get;
		else r = BUFSIZE - get;

		if (r > MAX_WRITE) r = MAX_WRITE;

		pthread_mutex_unlock(&buflock);
		r = write(1, &buf[get], r);

		if (r == 0 || (r==-1 && (errno==EAGAIN||errno==EWOULDBLOCK))) {
			/* This shouldn't happen unless stdout is O_NONBLOCK */
			FD_SET(1, &wfds);
			select(2, NULL, &wfds, &wfds, NULL);
			pthread_mutex_lock(&buflock);
			continue;
		} else if (r == -1 && errno == EINTR) {
			pthread_mutex_lock(&buflock);
			continue;
		} else if (r == -1) {
			perror("stdout");
			return 2;
		} else pthread_mutex_lock(&buflock);
		get += r;
		if (get >= BUFSIZE) get -= BUFSIZE;
		if (nf != UINT32_MAX) nf -= r;
	} 

	if (nf == UINT32_MAX) return 1;
	else pthread_cond_wait(&bufcond, &buflock);

	goto superloop;

	return 0;
}

