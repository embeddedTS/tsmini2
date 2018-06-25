#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

int main(int argc, char **argv)
{
	printf("chan0, chan1, chan2, chan3\n");
	while(1){
		int16_t buf[4];
		int j;
		ssize_t rd;

		rd = read(0, buf, 8);
		if(rd == -1)
			return 1;

		/* Make sure we read back a multiple of 8 bytes */
		assert(rd == 8);

		/* Print as csv */
		printf("%hd, %hd, %hd, %hd\n",
			buf[(j*4)],
			buf[(j*4) + 1],
			buf[(j*4) + 2],
			buf[(j*4) + 3]);
	}

	return 0;
}
