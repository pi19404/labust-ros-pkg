/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_smbus.c
*
* Programmers:
*	Neo Lo
*
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "REL_Linux_SUSI.H"

// Return 0 if platform infomation is correctly obtained.
// Otherwise, return 1.
int show_platform_info(void)
{
	int result;
	DWORD major, minor, year, month, date;
	const int BUF_LENGTH = 128;
	TCHAR buf[BUF_LENGTH];  // Buffer length includes the null character.

	SusiDllGetVersion(&major, &minor);
    year    = minor/10000;
    month   = minor%10000/100;
    date    = minor%100;
	printf("Version: %li (20%02li/%02li/%02li)\n", major, year, month, date);

	// Get platform name.
	result = SusiGetPlatformName(buf, BUF_LENGTH);
	if (result < 0) {
		printf("SusiGetPlatformName() failed\n");
		return 1;
	}
	else if (result > 0) {
		printf("SusiGetPlatformName(): buffer is too short\n");
		return 1;
	}
	else
		printf("Platform name: %s\n", buf);

	// Get BIOS version.
	result = SusiGetBIOSVersion(buf, BUF_LENGTH);
	if (result < 0) {
		return 1;
	}
	else if (result > 0) {
		printf("SusiGetBIOSVersion(): buffer is too short\n");
		return 1;
	}
	else
		printf("BIOS version: %s\n", buf);

	return 0;
}

void show_menu(void)
{
	printf("\n");
	printf("0) Terminate this program\n");
	printf("1) Read a single byte\n");
	printf("2) Read a single word\n");
	printf("3) Read multiple bytes\n");
	printf("4) Write a single byte\n");
	printf("5) Write a single word\n");
	printf("6) Write multiple bytes\n");
	printf("7) Scan device\n");
	printf("Enter your choice: ");
}

int read_byte(void)
{
	int result, data;
	BYTE address, offset, storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	result = SusiSMBusReadByte(address, offset, &storage);
	if (result == FALSE) {
		printf("SusiSMBusReadByte() failed\n");
		return 1;
	}
	else
		printf("Data read: 0x%02x\n", (int) storage);
	return 0;
}

int read_word(void)
{
	int result, data;
	BYTE address, offset;
	WORD storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	result = SusiSMBusReadWord(address, offset, &storage);
	if (result == FALSE) {
		printf("SusiSMBusReadWord() failed\n");
		return 1;
	}
	else
		printf("Data read: 0x%04x\n", (int) storage);
	return 0;
}

int read_bytes(void)
{
	int result, data, count, index;
	BYTE address, offset, *storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	printf("Count: ");
	if (scanf("%i", &count) <= 0)
		return 1;
	storage = (BYTE*) malloc(count);
	if (! storage) {
		printf("Memory allocation failed\n");
		return 1;
	}

	result = SusiSMBusReadByteMulti(address, offset, storage, count);
	if (result == FALSE) {
		printf("SusiSMBusReadByteMulti() failed\n");
		return 1;
	}
	else {
		printf("Data read:");
		for (index = 0; index < count; index++) {
			printf(" %02x", storage[index]);
		}
		printf("\n");
	}
	free(storage);
	return 0;
}

int write_byte(void)
{
	int result, data;
	BYTE address, offset, storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	printf("Data to write: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	storage = (BYTE) data;

	result = SusiSMBusWriteByte(address, offset, storage);
	if (result == FALSE) {
		printf("SusiSMBusWriteByte() failed\n");
		return 1;
	}
	return 0;
}

int write_word(void)
{
	int result, data;
	BYTE address, offset;
	WORD storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	printf("Data to write: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	storage = (WORD) data;

	result = SusiSMBusWriteWord(address, offset, storage);
	if (result == FALSE) {
		printf("SusiSMBusWriteWord() failed\n");
		return 1;
	}
	return 0;
}

int write_bytes(void)
{
	int result, data, count, index;
	BYTE address, offset, *storage, bt_count;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Offset: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	offset = (BYTE) data;

	printf("Count: ");
	if (scanf("%i", &count) <= 0)
		return 1;
	bt_count = (BYTE) count;
	storage = (BYTE*) malloc(bt_count);
	if (! storage) {
		printf("Memory allocation failed\n");
		return 1;
	}

	for (index = 0; index < bt_count; index++) {
		printf("Data %i: 0x", index);
		if (scanf("%x", &data) <= 0)
			return 1;
		storage[index] = (BYTE) data;
	}

	result = SusiSMBusWriteByteMulti(address, offset, storage, bt_count);
	if (result == FALSE) {
		printf("SusiSMBusWriteByteMulti() failed\n");
		return 1;
	}
	free(storage);
	return 0;
}

int scan_device(void)
{
	int i, j, result;
	const int MAX_PORT = 0x80;

	printf("Existing smbus device address :\n");
	for (i=0; i < MAX_PORT; i+=16)
	{
		for (j = 0; j  < 16; j++)
		{
			result = SusiSMBusScanDevice(i+j);

			if (result < 0)
			{
				printf("SusiSMBusScanDevice() result < 0");
				return 1;
			}

			if (result == 1)
				printf("0x%02x ", (i+j) << 1);
		}
	}
	printf("\n");
		
	return 0;
}

int main(void)
{
	int result, done, op;

	result = SusiDllInit();
	if (result == FALSE) {
		printf("SusiDllInit() failed\n");
		return 1;
	}

	result = SusiSMBusAvailable();
	if (result == 0) {
		printf("SusiSMBusAvailable() failed\n");
		SusiDllUnInit();
		return 1;
	}

	result = show_platform_info();
	
	done = 0;
	while (! done) {
		show_menu();
		if (scanf("%i", &op) <= 0)
			op = -1;

		switch (op) {
		case 0:
			done = 1;
			continue;
		case 1:
			result = read_byte();
			break;
		case 2:
			result = read_word();
			break;
		case 3:
			result = read_bytes();
			break;
		case 4:
			result = write_byte();
			break;
		case 5:
			result = write_word();
			break;
		case 6:
			result = write_bytes();
			break;
		case 7:
			result = scan_device();
			break;
		default:
			printf("\nUnknown choice!\n\n");
			continue;
		}
		if (result != 0) {
			printf("Library returns with error.\n");
			SusiDllUnInit();
			return 1;
		}
	}

	result = SusiDllUnInit();
	if (result == FALSE) {
		printf("SusiDllUnInit() failed\n");
		return 1;
	}

	return 0;
}

