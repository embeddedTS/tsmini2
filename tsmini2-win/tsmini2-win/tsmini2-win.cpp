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

#include <iostream>
#include <setupapi.h>


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

static bool ch1_active, ch2_active, ch3_active, ch4_active;
static bool binaryMode;

int main(int argc, char *argv[])
{
	int reg;
	unsigned long val;
	bool do_read = false;
	bool do_write = false;
	bool have_register = false;
	bool have_val = false;

	ch1_active = ch2_active = ch3_active = ch4_active = true;
	binaryMode = false;

	if (argc > 1) {
		for (int i = 1; i < argc; i++) {
			char *cp = argv[i];

			if (*cp == '-' || *cp == '/') {
				cp++;
				if (!*cp) {
					std::cerr << "Missing switch at -" << std::endl;
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
						if (reg < 0 || reg > 4) {
							std::cerr << "Register address out of range (0..4)" << std::endl;
							return 1;
						}
						have_register = true;
					}
					else {
						std::cerr << "Missing argument to -a" << std::endl;
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
						std::cerr << "Missing argument to -w" << std::endl;
						return 1;
					}

					do_write = true;
				}
				else if (*cp == 'r') {
					do_read = true;
				}
				else if (*cp == 's') {
					char *tok, *next;
					int c,n;

					cp++;
					if (!*cp) {
						i++;
						cp = argv[i];
					}

					if (!cp) {
						std::cerr << "Missing argument to -s" << std::endl;
						return 1;
					}

					ch1_active = ch2_active = ch3_active = ch4_active = false;

					next = NULL;
					n = 0;
					tok = strtok_s(cp, ":.-\0", &next);
					while (tok) {

						c = atoi(tok);
						if (c < 1 || c > 4) {
							std::cerr << "Channel number '" << tok << "' out of bounds in -s" << std::endl;
							return 1;
						}
						n++;
						if (n > 4) {
							std::cerr << "Too many channels in -s" << std::endl;
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
						std::cerr << "No channels active" << std::endl;
						return 1;
					}

				}
				else if (*cp == 'h') {
					std::cout << "Usage:" << std::endl <<
						"    tsmini2-win -a N -r    (to read register N)" << std::endl <<
						"    tsmini2-win -a N -w V  (to write V register N)" << std::endl <<
						"    tsmini2-win -s1:3:4    (to sample channels 1,3 and 4; default is to sample all channels)" << std::endl << std::endl <<
						"    Use -b for binary output" << std::endl;
					return 0;

				} else if (*cp == 'b') {
					binaryMode = true;
				}
				else {
					std::cerr << "Unrecognized argument -" << *cp << std::endl;
					return 1;
				}
			}
		}
	}

	if (do_write) {
		if (!have_register) {
			std::cerr << "Missing register" << std::endl;
			return 1;
		}
		else if (!have_val) {
			std::cerr << "Missing value to write" << std::endl;
			return 1;
		}
	}

	if (do_read && !have_register) {
		std::cerr << "Missing register" << std::endl;
		return 1;
	}

	wcsncpy_s(fname, _T("TS-MINI-ADC"), sizeof(fname) / 2);

	HDEVINFO info = SetupDiGetClassDevs(&GUID_DEVINTERFACE_TSMiniDriver, NULL, NULL,  DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

	if (info == INVALID_HANDLE_VALUE) {
		std::cerr << "No such class with GUID 280D1736-F08A-494D-AFA2-5A731DBBC7BF" << std::endl;
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
				std::cerr << "Cannot open " << detail->DevicePath << std::endl;
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
		std::cerr << "Cannot open TS-MINI-ADC device (is it present on your system?) " << std::endl;
		return 1;
	}

	if (do_read || do_write) {
		ULONG result, returnedLen;

		if (do_write) {
			ULONG64 regVal = ((ULONG64)val << 32) | reg;
			if (reg == 4) {
				DeviceIoControl(TsMiniDeviceHandle, TSMINIADC_IOCTL_READ_REG, &reg, sizeof(reg), &result, sizeof(result), &returnedLen, 0);

				if (result) {
					std::cerr << "DMA address register already written; disallowing another write" << std::endl;
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
		std::cerr << "Failed to allocate memory for FIFO" << std::endl;
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
		std::cerr << "Failed to create fpga_loop" << std::endl;
	else {

		SetThreadPriority(hFpgaLoop, THREAD_PRIORITY_HIGHEST);

		hMatlabLoop = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)loop_output, NULL, 0, &tid_matlab);
		if (hMatlabLoop == INVALID_HANDLE_VALUE)
			std::cerr << "Failed to create matlab_loop" << std::endl;
		else {
			time_t startTime, timeElapsed;

			SetThreadPriority(hMatlabLoop, THREAD_PRIORITY_NORMAL);

			startTime = time(NULL);
			for (;;) {

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
		reg = 4;	// a pseudo-register (ie, in the driver, not in the FPGA)
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
			std::cerr << "  *** DMA Overflow *** try " << sleepTime << std::endl;
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
	}

	return 0;
}

