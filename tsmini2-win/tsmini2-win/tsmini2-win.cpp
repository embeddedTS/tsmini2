/*
	This program may be used to read and/or write a register on the TS-MINI-ADC.
	It may also be used to sample any or all of the ADC channels, printing the
	output to the terminal.


	Be sure to link in SetupAPI.lib (see Project->Properties->Linker->Input)
*/

#include "stdafx.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <filesystem>
#include <iostream>

#include <setupapi.h>

using namespace std;
using namespace std::experimental::filesystem;

#define NCHANNELS	4

#define FIFOSIZE (512 * 0x100000)    // 512MB or 128MB per channel
#define FIFOSIZE_IN_WORDS (FIFOSIZE / sizeof(WORD))

#define DMA_SIZE	0x200000		// 2MB
#define DMA_SIZE_IN_WORDS	(DMA_SIZE / sizeof(WORD))

static TCHAR fname[256];

GUID GUID_CLASS_TSMiniDriver = // {280D1736-F08A-494D-AFA2-5A731DBBC7BF}
{ 0x280d1736, 0xf08a, 0x494d,{ 0xaf, 0xa2, 0x5a, 0x73, 0x1d, 0xbb, 0xc7, 0xbf } };

GUID GUID_DEVINTERFACE_TSMiniDriver = // {cadae116-f4d7-42c9-9bb1-bac6d8e2d6d0}
{ 0xcadae116, 0xf4d7, 0x42c9,{ 0x9b, 0xb1, 0xba, 0xc6, 0xd8, 0xe2, 0xd6, 0xd0 } };

#define TSMINIADC_IOCTL_READ_REG CTL_CODE(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_READ_DATA)
#define TSMINIADC_IOCTL_WRITE_REG CTL_CODE(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_WRITE_DATA)
#define TSMINIADC_IOCTL_READ_PCI_CONFIG CTL_CODE(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_WRITE_DATA)


SRWLOCK fifoLock_0, fifoLock_1, fifoLock_2, fifoLock_3;
CONDITION_VARIABLE fifoCV_0, fifoCV_1, fifoCV_2, fifoCV_3;
HANDLE hFpgaLoop;     // for fpga_loop thread
HANDLE hMatlabLoop;     // for matlab

DWORD loop_fpga(LPVOID pParam);
DWORD loop_output(LPVOID pParam);

DWORD tid_fpga, tid_matlab;

uint64_t consumed, produced;
BOOL fifoEmpty_0, fifoEmpty_1, fifoEmpty_2, fifoEmpty_3;

PUINT16 dataFIFO;
PUINT16 dataFIFO_0, dataFIFO_1, dataFIFO_2, dataFIFO_3;
static volatile uint32_t put_0, get_0, put_1, get_1, put_2, get_2, put_3, get_3;
static HANDLE TsMiniDeviceHandle;
static int64_t num_samples;

static bool ch1_active, ch2_active, ch3_active, ch4_active;
static bool binaryMode;
static uint32_t reg10h;
static int opt_save(char *arg);
static int opt_program(char *arg);
static bool sampling_completed;


static void usage(char **argv)
{
	path prog(argv[0]);

	cout << "Usage: " << prog.filename() << " [OPTION] ..." << endl <<
		"  -a ADR        Specify register address" << endl <<
		"  -w VAL        Write VAL to register" << endl <<
		"  -r            Read register" << endl <<
		"  -o OUTPUTS    Initialize CN1 digital outputs to OUTPUTS" << endl <<
		"  -c VAL        Initialize config reg to VAL" << endl <<
		"  -S channels   Select active channels, specified for example as 1.2.4 (default is to sample all channels)" << endl << 
		"  -n NUM        Stop after NUM samples per active channel (default is to never stop)" << endl <<
		"  -p RPDFILE    Program new FPGA configuration flash from RPDFILE" << endl <<
		"  -s RPDFILE    Save existing FPGA flash config to RPDFILE" << endl <<
		"  -b            Output samples in binary" << endl;
}

int main(int argc, char *argv[])
{
	int reg;
	unsigned long val;
	bool do_read = false;
	bool do_write = false;
	bool do_show_config = false;
	bool have_register = false;
	bool have_val = false;
	char *opt_save_arg = NULL;
	char *opt_prog_arg = NULL;

	ch1_active = ch2_active = ch3_active = ch4_active = true;
	binaryMode = sampling_completed = false;
	
	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			char *cp = argv[i];

			if (*cp == '-' || *cp == '/') {
				cp++;
				if (!*cp) {
					cerr << "Missing switch at -" << endl;
					return 1;
				}
				if (*cp == 'a') {
					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (cp) {
						reg = strtoul(cp, NULL, 0);
						if (reg < 0 || reg > 5) {
							cerr << "Register address out of range (0..5)" << endl;
							return 1;
						}
						have_register = true;
					}
					else {
						cerr << "Missing argument to -a" << endl;
						return 1;
					}
				}
				else if (*cp == 'w') {
					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (cp) {
						val = strtoul(cp, NULL, 0);
						have_val = true;
					}
					else {
						cerr << "Missing argument to -w" << endl;
						return 1;
					}

					do_write = true;
				}
				else if (*cp == 'r') {
					do_read = true;
				}
				else if (*cp == 's') {

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -s" << endl;
						return 1;
					}

					opt_save_arg = _strdup(cp);

				} else if (*cp == 'n') {

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -n" << endl;
						return 1;
					}

					num_samples = strtoull(cp, NULL, 0);
					if (num_samples < 0) {
						cerr << "Argument to -n cannot be negative" << endl;
						return 1;
					}
					
				} else if (*cp == 'c') {

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -c" << endl;
						return 1;
					}

					val = strtoul(cp, NULL, 0);
					reg = 0;
					have_val = have_register = do_write = true;
					
				} else if (*cp == 'o') {

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -o" << endl;
						return 1;
					}

					val = strtoul(cp, NULL, 0) & 0xC0000FFF;
					reg = 4;
					have_val = have_register = do_write = true;

				} else if (*cp == 'p') {

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -p" << endl;
						return 1;
					}

					opt_prog_arg = _strdup(cp);

				}
				else if (*cp == 'S') {
					char *tok, *next;
					int c,n;

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						cerr << "Missing argument to -S" << endl;
						return 1;
					}

					ch1_active = ch2_active = ch3_active = ch4_active = false;

					next = NULL;
					n = 0;
					tok = strtok_s(cp, ":.-\0", &next);
					while (tok) {

						c = atoi(tok);
						if (c < 1 || c > 4) {
							cerr << "Channel number '" << tok << "' out of bounds in -s" << endl;
							return 1;
						}
						n++;
						if (n > 4) {
							cerr << "Too many channels in -S" << endl;
							return 1;
						}

						switch (c) {
							case 1:		ch1_active = true; break;
							case 2:		ch2_active = true; break;
							case 3:		ch3_active = true; break;
							case 4:		ch4_active = true; break;
						}
						tok = strtok_s(NULL, ":.-\0", &next);
					}

					if (ch1_active == false && ch2_active == false && ch3_active == false && ch4_active == false) {
						cerr << "No channels active" << endl;
						return 1;
					}

				}
				else if (*cp == 'h') {
					usage(argv);
					return 0;

				} else if (*cp == 'b') {
					binaryMode = true;
				}
				else if (*cp == 'l') {
					do_show_config = true;						
				} else {
					cerr << "Unrecognized argument -" << *cp << endl;
					return 1;
				}
			}
		}
	}


	if (do_write) {
		if (!have_register) {
			cerr << "Missing register" << endl;
			return 1;
		}
		else if (!have_val) {
			cerr << "Missing value to write" << endl;
			return 1;
		}
	}

	if (do_read && !have_register) {
		cerr << "Missing register" << endl;
		return 1;
	}

	wcsncpy_s(fname, _T("TS-MINI-ADC"), sizeof(fname) / 2);

	HDEVINFO info = SetupDiGetClassDevs(&GUID_DEVINTERFACE_TSMiniDriver, NULL, NULL,  DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

	if (info == INVALID_HANDLE_VALUE) {
		cerr << "No such class with GUID 280D1736-F08A-494D-AFA2-5A731DBBC7BF" << endl;
		return 1;
	}
	
	SP_DEVICE_INTERFACE_DATA ifdata;
	ifdata.cbSize = sizeof(ifdata);
	DWORD devindex;

	TsMiniDeviceHandle = INVALID_HANDLE_VALUE;
	for (devindex = 0; SetupDiEnumDeviceInterfaces(info, NULL, &GUID_DEVINTERFACE_TSMiniDriver, devindex, &ifdata); ++devindex)
	{
		// for each device
		// Determine the symbolic link name for this device instance. Since
		// this is variable in length, make an initial call to determine
		// the required length.

		DWORD needed;
		SetupDiGetDeviceInterfaceDetail(info, &ifdata, NULL, 0, &needed, NULL);

		PSP_INTERFACE_DEVICE_DETAIL_DATA detail = (PSP_INTERFACE_DEVICE_DETAIL_DATA)malloc(needed);
		SP_DEVINFO_DATA did = { sizeof(SP_DEVINFO_DATA) };
		detail->cbSize = sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);
		if (!SetupDiGetDeviceInterfaceDetail(info, &ifdata, detail, needed, NULL, &did))
		{						// can't get detail info
			free((PVOID)detail);
			continue;
		}


		if (SetupDiGetDeviceRegistryProperty(info, &did, SPDRP_FRIENDLYNAME, NULL, (PBYTE)fname, sizeof(fname) / 2, NULL)
			&& SetupDiGetDeviceRegistryProperty(info, &did, SPDRP_DEVICEDESC, NULL, (PBYTE)fname, sizeof(fname) / 2, NULL))

		{
		
			TsMiniDeviceHandle = CreateFile(detail->DevicePath, GENERIC_ALL, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_SYSTEM, 0);
		
			if (TsMiniDeviceHandle == INVALID_HANDLE_VALUE) {
				cerr << "Cannot open " << detail->DevicePath << endl;
				return 1;
			}
			else {
				break;
			}
		}
	}

	if (info) {
		SetupDiDestroyDeviceInfoList(info);
	}

	if (TsMiniDeviceHandle == INVALID_HANDLE_VALUE) {
		cerr << "Cannot open TS-MINI-ADC device (is it present on your system?) " << endl;
		return 1;
	}

	if (do_show_config) {
		ULONG result, returnedLen;
		reg = 0;
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);
		
		printf("rev=%d\n", result & 0xff);
		printf("sel_an1_gnd=%d\n", !!(result & (1 << 8)));
		printf("sel_an2_gnd=%d\n", !!(result & (1 << 9)));
		printf("sel_an3_gnd=%d\n", !!(result & (1 << 10)));
		printf("sel_an4_gnd=%d\n", !!(result & (1 << 11)));
		printf("sel_an4_gnd=%d\n", !!(result & (1 << 11)));
		printf("sel_an3_dc=%d\n", !!(result & (1 << 12)));
		printf("sel_an4_dc=%d\n", !!(result & (1 << 13)));
		return 0;
	}

	if (opt_save_arg)
		return opt_save(opt_save_arg);

	if (opt_prog_arg)
		return opt_program(opt_prog_arg);

	if (do_read || do_write) {
		ULONG result, returnedLen;

		if (do_write) {
			ULONG64 regVal = ((ULONG64)val << 32) | reg;
			if (reg == 1) {
				DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);

				if (result) {
					cerr << "DMA address register already written; disallowing another write" << endl;
					return 1;
				}
			}
			DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);
		}

		if (do_read) {
			DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);

			printf("0x%08X\n", result);
		}

		return 0;
	}



	InitializeSRWLock(&fifoLock_0);
	InitializeSRWLock(&fifoLock_1);
	InitializeSRWLock(&fifoLock_2);
	InitializeSRWLock(&fifoLock_3);

	InitializeConditionVariable(&fifoCV_0);
	InitializeConditionVariable(&fifoCV_1);
	InitializeConditionVariable(&fifoCV_2);
	InitializeConditionVariable(&fifoCV_3);

	dataFIFO = (PUINT16)malloc(FIFOSIZE);

	if (dataFIFO == NULL) {
		cerr << "Failed to allocate memory for FIFO" << endl;
		return 1;
	}

	dataFIFO_0 = dataFIFO;
	dataFIFO_1 = &dataFIFO[FIFOSIZE_IN_WORDS / 4];
	dataFIFO_2 = &dataFIFO[(FIFOSIZE_IN_WORDS * 2) / 4];
	dataFIFO_3 = &dataFIFO[(FIFOSIZE_IN_WORDS * 3) / 4];

	fifoEmpty_0 = fifoEmpty_1 = fifoEmpty_2 = fifoEmpty_3 = true;
	put_0 = get_0 = put_1 = get_1 = put_2 = get_2 = put_3 = get_3 = 0;
	consumed = produced = 0;


	hFpgaLoop = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)loop_fpga, NULL, 0, &tid_fpga);
	if (hFpgaLoop == INVALID_HANDLE_VALUE)
		cerr << "Failed to create fpga_loop" << endl;
	else {

		SetThreadPriority(hFpgaLoop, THREAD_PRIORITY_HIGHEST);

		hMatlabLoop = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)loop_output, NULL, 0, &tid_matlab);
		if (hMatlabLoop == INVALID_HANDLE_VALUE)
			cerr << "Failed to create matlab_loop" << endl;
		else {
			time_t startTime, timeElapsed;

			SetThreadPriority(hMatlabLoop, THREAD_PRIORITY_NORMAL);

			startTime = time(NULL);
			while (!sampling_completed) {

				Sleep(1000);
				timeElapsed = time(NULL) - startTime;
#if DEBUG_ON
				printf("%lld %lld %lld, %ld bytes/sec\n", produced, consumed, consumed - produced, (long)(produced / timeElapsed));
#endif
			}
		}
	}

	return 0;
}




static uint16_t b[DMA_SIZE_IN_WORDS];

static inline void fifo_put2(HANDLE h, DWORD nBytesWanted, PDWORD nPut)
{
	DWORD nBytesRead,
		space_0, space_1, space_2, space_3, space_total,
		nWords,
		nWordsRead;
	int i, j;
	bool a0, a1, a2, a3;

	AcquireSRWLockExclusive(&fifoLock_0);
	AcquireSRWLockExclusive(&fifoLock_1);
	AcquireSRWLockExclusive(&fifoLock_2);
	AcquireSRWLockExclusive(&fifoLock_3);

	space_0 = (FIFOSIZE_IN_WORDS / NCHANNELS) - put_0;
	space_1 = (FIFOSIZE_IN_WORDS / NCHANNELS) - put_1;
	space_2 = (FIFOSIZE_IN_WORDS / NCHANNELS) - put_2;
	space_3 = (FIFOSIZE_IN_WORDS / NCHANNELS) - put_3;

	space_total = space_0 + space_1 + space_2 + space_3;
	a0 = a1 = a2 = a3 = false;
	nWords = min(space_total, (nBytesWanted / 2));

	nBytesRead = 0;
	//printf("space = %d words, reading %d bytes, ", space_total, nWords * 2);
	if (!ReadFile(h, b, nWords * 2, &nBytesRead, 0))
		printf("Error in ReadFile()\n");

	else if (nBytesRead) {

		produced += nBytesRead;
		nWordsRead = nBytesRead / 2;
		*nPut += nBytesRead;

		//printf("read %d bytes, produced %lld\n", nBytesRead, produced);

		j = 0;
		while (j < (int)nWordsRead)
			for (i = 0; i < NCHANNELS; i++) {
				if (i == 0) {
					dataFIFO_0[put_0] = b[j];
					a0 = true;
					put_0++;
					if (put_0 >= (FIFOSIZE_IN_WORDS / NCHANNELS))
						put_0 = 0;
				}
				else if (i == 1) {
					dataFIFO_1[put_1] = b[j];
					a1 = true;
					put_1++;
					if (put_1 >= (FIFOSIZE_IN_WORDS / NCHANNELS))
						put_1 = 0;
				}
				else if (i == 2) {
					dataFIFO_2[put_2] = b[j];
					a2 = true;
					put_2++;
					if (put_2 >= (FIFOSIZE_IN_WORDS / NCHANNELS))
						put_2 = 0;
				}
				else if (i == 3) {
					dataFIFO_3[put_3] = b[j];
					a3 = true;
					put_3++;
					if (put_3 >= (FIFOSIZE_IN_WORDS / NCHANNELS))
						put_3 = 0;
				}

				j++;
				if (j > (int)nWordsRead)
					break;
			}
	}

	if (a0) {
		fifoEmpty_0 = false;
		WakeAllConditionVariable(&fifoCV_0);
	}
	if (a1) {
		fifoEmpty_1 = false;
		WakeAllConditionVariable(&fifoCV_1);
	}
	if (a2) {
		fifoEmpty_2 = false;
		WakeAllConditionVariable(&fifoCV_2);
	}
	if (a3) {
		fifoEmpty_3 = false;
		WakeAllConditionVariable(&fifoCV_3);
	}

	ReleaseSRWLockExclusive(&fifoLock_3);
	ReleaseSRWLockExclusive(&fifoLock_2);
	ReleaseSRWLockExclusive(&fifoLock_1);
	ReleaseSRWLockExclusive(&fifoLock_0);
}


DWORD loop_fpga(LPVOID pParam)
{
	DWORD reg, dma_buffer_base, sleepTime;
	ULONG returnedLen;
	static LARGE_INTEGER pos = { 0, 0 };

	reg = 1;	// the dma address register

	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &dma_buffer_base, sizeof(dma_buffer_base), &returnedLen, 0);

	if (dma_buffer_base == NULL) {	// If DMA hasn't been started already, read the buffer address from the driver...
		reg = 5;	// a pseudo-register (ie, in the driver, not in the FPGA)
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &dma_buffer_base, sizeof(dma_buffer_base), &returnedLen, 0);
		reg = 1;
		if (dma_buffer_base) {	// and write it to the FPGA dma base register
			ULONG result;
			ULONG64 regVal = ((ULONG64)dma_buffer_base << 32) | reg;

			DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);
		}
	}

	reg = 2;	// the last-write register
	sleepTime = 35;
	for (;;) {

		DWORD nRead, nWanted;
		volatile DWORD last_write;

		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), (PDWORD)&last_write, sizeof(last_write), &returnedLen, 0);
		if (last_write & 1) {
			if (sleepTime > 2)
				sleepTime--;
			cerr << "  *** DMA Overflow *** try " << sleepTime << endl;
		}

		nRead = 0;
		last_write &= ~3;
		last_write -= dma_buffer_base;

		if (last_write == pos.LowPart) {
			sleepTime++;

		}
		else if (last_write > pos.LowPart) {
			nWanted = last_write - pos.LowPart;
			nRead = 0;
			SetFilePointerEx(TsMiniDeviceHandle, pos, NULL, FILE_BEGIN);
			fifo_put2(TsMiniDeviceHandle, nWanted, &nRead);
			pos.LowPart = last_write;
		}
		else {
			nRead = 0;
			nWanted = DMA_SIZE - pos.LowPart;
			SetFilePointerEx(TsMiniDeviceHandle, pos, NULL, FILE_BEGIN);
			fifo_put2(TsMiniDeviceHandle, nWanted, &nRead);

			pos.LowPart = 0;
			nWanted = last_write;
			SetFilePointerEx(TsMiniDeviceHandle, pos, NULL, FILE_BEGIN);
			fifo_put2(TsMiniDeviceHandle, nWanted, &nRead);
			pos.LowPart = last_write;
		}

		Sleep(sleepTime);
	}
}


#define FIFO_GET_(X) \
int16_t fifo_get_ ## X(void)\
{\
	int16_t v;\
\
	AcquireSRWLockExclusive(&fifoLock_ ## X);\
\
	if (fifoEmpty_ ## X  && SleepConditionVariableSRW(&fifoCV_ ## X, &fifoLock_ ## X, 120, 0) == 0) {\
		printf("w");\
		if (GetLastError() == ERROR_TIMEOUT)\
			printf("to\n");\
		ReleaseSRWLockExclusive(&fifoLock_ ## X);\
		return -1;\
	}\
\
	v = dataFIFO_ ## X[get_ ## X];\
	get_ ## X ++;\
	if (get_ ## X >= (FIFOSIZE_IN_WORDS / NCHANNELS))\
		get_ ## X = 0;\
\
	if (get_ ## X == put_ ## X)\
		fifoEmpty_ ## X = true;\
\
	consumed += sizeof(v);\
\
	ReleaseSRWLockExclusive(&fifoLock_ ## X);\
\
	return v;\
}

FIFO_GET_(0)
FIFO_GET_(1)
FIFO_GET_(2)
FIFO_GET_(3)


DWORD loop_output(LPVOID pParam)
{
	volatile uint16_t sample;

	for (;;) {
		if (!binaryMode) {
			sample = fifo_get_0();
			if (ch1_active)
				printf("%04X ", sample);
			sample = fifo_get_1();
			if (ch2_active)
				printf("%04X ", sample);
			sample = fifo_get_2();
			if (ch3_active)
				printf("%04X ", sample);
			sample = fifo_get_3();
			if (ch4_active)
				printf("%04X ", sample);
			printf("\n");
		}
		else {
			sample = fifo_get_0();
			if (ch1_active)
				fwrite((void*)&sample, sizeof(sample), 1, stdout);
			sample = fifo_get_1();
			if (ch2_active)
				fwrite((void*)&sample, sizeof(sample), 1, stdout);
			sample = fifo_get_2();
			if (ch3_active)
				fwrite((void*)&sample, sizeof(sample), 1, stdout);
			sample = fifo_get_3();
			if (ch4_active)
				fwrite((void*)&sample, sizeof(sample), 1, stdout);
		}
		if (num_samples > 0) {
			if (--num_samples == 0)
				break;
		}		
	}

	sampling_completed = true;
	return 0;
}

///////////  SPI Flash code follows ///////////////////


static uint8_t read_spi_byte(void) {
	uint32_t n = 0;
	uint8_t ret;
	int reg = 4;
	ULONG result, returnedLen, val;
	ULONG64 regVal;

	for (ret = n = 0; n < 8; n++) {	
		val = reg10h & ~(1 << 17); /* clk lo */
		regVal = ((ULONG64)val << 32) | reg;
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);
		
		val = reg10h | (1 << 17); /* clk hi */
		regVal = ((ULONG64)val << 32) | reg;
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);

		ret = (ret << 1);

		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);

		if (result & (1 << 19)) ret |= 1;
	}
	return ret;
}

static void write_spi_byte(uint8_t x) {
	uint32_t n = 0;
	uint32_t v;

	ULONG result, returnedLen, val;
	ULONG64 regVal;

	for (n = 0; n < 8; n++) {
		if (x & 0x80)
			v = reg10h | (1 << 16);
		else
			v = reg10h & ~(1 << 16);

		val = v & ~(1 << 17); /* clk lo */
		regVal = ((ULONG64)val << 32) | 4;
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);

		val = v | (1 << 17); /* clk hi */
		regVal = ((ULONG64)val << 32) | 4;
		DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);

		x = x << 1;
	}
	return;
}

static void enable_cs(void) {
	ULONG result, returnedLen, val;
	ULONG64 regVal;

	reg10h &= ~(1 << 18);
	val = reg10h;
	regVal = ((ULONG64)val << 32) | 4;
	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);
}

static void disable_cs(void) {
	ULONG result, returnedLen, val;
	ULONG64 regVal;

	reg10h |= (1 << 18);
	val = reg10h;
	regVal = ((ULONG64)val << 32) | 4;
	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_WRITE_REG, &regVal, sizeof(regVal), &result, sizeof(result), &returnedLen, 0);
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
	while (cnt--) *(buf++) = read_spi_byte();
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
	while (cnt--) write_spi_byte(*(buf++));
	disable_cs();

	enable_cs();
	write_spi_byte(5); /* Read status register */
	do { s = read_spi_byte(); n++; } while ((s & 1) && n < 8192);
	disable_cs();
	if (n == 8192) {
		cerr << "SPI flash chip write timeout!" << endl;
		exit(1);
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
		cerr << "SPI flash chip erase timeout!" << endl;
		exit(1);
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

static int opt_save(char *arg)
{
	int i;
	uint8_t *fpga_spif;
	uint32_t id, fpgarev;
	FILE *out = stdout;

	ULONG result, returnedLen;
	int reg = 0;

	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);
	fpgarev = result & 0xff;

	cerr << "fpgarev = 0x" << hex << fpgarev << endl;
		
	if (fpgarev < 3) {
		cerr << "FPGA must be rev 3 or later" << endl;
		return 1;
	}

	reg = 4;	// reg10h
	result = 0;
	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);
	reg10h = result;

	fpga_spif = (uint8_t *)malloc(0x200000);	
	if (fpga_spif == NULL) {
		cerr << "Memory allocation error in " << __func__ << endl;
		return 1;
	}

	cerr << "Reading FPGA SPI flash chip ID...0x";
	id = spif_rdid();
	cerr << hex << id;
	
	if (id == 0xffffff || id == 0) {
		cerr << "  Bad SPI flash chip ID!" << endl;
		return 2;
	}
	else 
		cerr << "  ok" << endl;

	if (arg && strcmp(arg, "-") != 0) {
		cerr << "Opening \"" << arg << "\" file for writing...";
		if (fopen_s(&out, arg, "wb")) {
			perror(arg);
			return 1;
		}
		else cerr << "ok" << endl;
	}

	for (i = 0; i < (0x200000 / 256); i++) {
		spif_read(i * 256, &fpga_spif[i * 256], 256);
		if ((i * 256) >> 18 != ((i - 1) * 256) >> 18)
			cerr << "Reading flash, " << std::dec << (i * 100 / (0x200000 / 256)) << "% done..." << endl;			
	}
	cerr << "Reading flash, 100% done..." << endl;

	for (i = 0x200000 - 1; i > 0; i--) if (fpga_spif[i] != 0xff) break;
	i++;
	cerr << "Writing " << std::dec << i << " bytes to output...";
	fwrite(fpga_spif, 1, i, out);
	cerr << "ok" << endl;
		
	if (out != stdout) 
		fclose(out);

	return 0;
}

static int opt_program(char *arg) 
{
	int i, c;
	uint8_t *fpga_spif;
	uint32_t id, fpgarev;
	FILE *in = stdin;

	ULONG result, returnedLen;
	int reg = 0;
	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);
	
	fpgarev = result & 0xff;
	cerr << "fpgarev = 0x" << hex << fpgarev << endl;

	if (fpgarev < 3) {
		cerr << "FPGA must be rev 3 or later" << endl;
		return 1;
	}

	reg = 4;	// reg10h
	result = 0;
	DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);	
	reg10h = result;

	fpga_spif = (uint8_t *)malloc(0x200000);
	if (fpga_spif == NULL) {
		cerr << "Memory allocation error in " << __func__ << endl;
		return 1;
	}

	memset(fpga_spif, 0xff, 0x200000);

	if (arg && strcmp(arg, "-") != 0) {
		cerr << "Opening \"" << arg << "\" file for reading...";

		if (fopen_s(&in, arg, "rb")) {
			perror(arg);
			return 1;
		}
		else cerr << "ok" << endl;
	}
	else cerr << "Reading program from stdin..." << endl;

	for (i = 0; i < 0x200000; i++) {
		c = fgetc(in);
		if (c == EOF) break;
		else fpga_spif[i] = c;
	}
	if (in != stdin) fclose(in);

	cerr << "Reading FPGA SPI flash chip ID...0x";
	id = spif_rdid();
	cerr << hex << id;

	if (id == 0xffffff || id == 0) {
		cerr << "  Bad SPI flash chip ID!" << endl;
		return 2;
	}
	else
		cerr << "  ok" << endl;
	
	cerr << "Erasing...";
	spif_erase();
	cerr << "ok" << endl;


	for (i = 0; i < (0x200000 / 256); i++) {
		spif_write(i * 256, &fpga_spif[i * 256], 256);
		if ((i * 256) >> 18 != ((i - 1) * 256) >> 18)
			cerr << "Writing flash, " << std::dec << (i * 100 / (0x200000 / 256)) << "% done..." << endl;		
	}
	cerr << "Writing flash, 100% done..." << endl;
	
	for (i = 0; i < (0x200000 / 256); i++) {
		uint8_t buf[256];
		spif_read(i * 256, buf, 256);
		if ((i * 256) >> 18 != ((i - 1) * 256) >> 18)
			cerr << "Verifying flash, " << std::dec << (i * 100 / (0x200000 / 256)) << "% done..." << endl;
		if (memcmp(&fpga_spif[i * 256], buf, 256) != 0) {
			cerr << "Verify failed at " << (i * 256 * 100 / 0x200000) << "%!!" << endl;
			free(fpga_spif);
			return 3;
		}
	}
	cerr << "Verifying flash, 100% done..." << endl;
	free(fpga_spif);
	return 0;
}


