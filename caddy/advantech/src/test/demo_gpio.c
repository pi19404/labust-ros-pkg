/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_gpio.c
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
	printf("1) Get GPIO direction mask\n");
	printf("2) Read a single GPIO pin\n");
	printf("3) Read multiple GPIO pins\n");
	printf("4) Write a single GPIO pin\n");
	printf("5) Write multiple GPIO pins\n");
	printf("6) Set a GPIO direction\n");
	printf("7) Set multi GPIO directions\n");
	printf("Enter your choice: ");
}

// Return 0 on success and 1 on failure.
int get_dir_mask(void)
{
	DWORD TargetPinMask, PinDirMask, InPinDirMask, OutPinDirMask;

	if (SusiIOQueryMask(ESIO_SMASK_PIN_FULL, &TargetPinMask) == FALSE) {
		printf("SusiIOQueryMask() failed\n");
		return 1;
	}
    
	if (SusiIOQueryMask(ESIO_DMASK_DIRECTION, &PinDirMask) == FALSE) {
		printf("SusiIOQueryMask() failed\n");
		return 1;
	}
    
	if (SusiIOQueryMask(ESIO_DMASK_IN, &InPinDirMask) == FALSE) {
		printf("SusiIOQueryMask() failed\n");
		return 1;
	}
    
	if (SusiIOQueryMask(ESIO_DMASK_OUT, &OutPinDirMask) == FALSE) {
		printf("SusiIOQueryMask() failed\n");
		return 1;
	}
    
	else
	{
        printf("Total pins mask             = 0x%08lX\n", TargetPinMask);
        printf("direction mask(0:out, 1:in) = 0x%08lX\n", PinDirMask);
		printf("input pins mask             = 0x%08lX\n", InPinDirMask);
		printf("output pins mask            = 0x%08lX\n", OutPinDirMask);
	}
	return 0;
}

// Return 0 on success and 1 on failure.
int read_pin(void)
{
	int result, data;
	BYTE pin;
	BOOL status;

	printf("Which pin: ");
	if (scanf("%i", &data) <= 0)
		return 1;
	pin = (BYTE) data;
	result = SusiIOReadEx(pin, &status);
	if (result == FALSE) {
		printf("SusiIOReadEx() failed\n");
		return 1;
	}
	else
		printf("Status: %s\n", status ? "ONE" : "ZERO");
	return 0;
}

// Return 0 on success and 1 on failure.
int read_pins(void)
{
	int result, data;
	DWORD pins, status;

	printf("Bit mask of the pins: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	pins = (DWORD) data;
	result = SusiIOReadMultiEx(pins, &status);
	if (result == FALSE) {
		printf("SusiIOReadMultiEx() failed\n");
		return 1;
	}
	else
		printf("Status: 0x%lx\n", status);
	return 0;
}

// Return 0 on success and 1 on failure.
int write_pin(void)
{
	int result, data;
	BYTE pin;
	BOOL boolean;

	printf("Which pin: ");
	if (scanf("%i", &data) <= 0)
		return 1;
	pin = (BYTE) data;
	printf("Value (0 or 1): ");
	if (scanf("%i", &data) <= 0)
		return 1;
	boolean = (data) ? 1 : 0;
	result = SusiIOWriteEx(pin, boolean);
	if (result == FALSE) {
		printf("SusiIOWriteEx() failed\n");
		return 1;
	}
	return 0;
}

// Return 0 on success and 1 on failure.
int write_pins(void)
{
	int result, data;
	DWORD pins, status;

	printf("Bit mask of the pins: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	pins = (DWORD) data;
	printf("Bit pattern: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	status = (DWORD) data;
	result = SusiIOWriteMultiEx(pins, status);
	if (result == FALSE) {
		printf("SusiIOWriteMulti() failed\n");
		return 1;
	}
	return 0;
}
// Return	0	on success and 1 on	failure.
int set_pin_direction(void)
{
	BYTE PinNum, IO;
	int result, data;
	 
	printf("Which pin: ");
	if (scanf("%x", &data) <= 0)
		return 1;
	PinNum = (BYTE)data;
	printf("direction (input pin :1 output pin:0 ): \n");
	if (scanf("%x", &data) <= 0)
		return 1;
	IO = (BYTE)	data ;
	 
	result=SusiIOSetDirection(PinNum,IO, NULL);
	if (result == FALSE) 
	{
		printf("set_pin_direction() failed\n");
		return 1;
	}
	return 0;
}

// Return	0	on success and 1 on	failure.
int set_pins_direction(void)
{
	int result, data;
	DWORD PinNum, PinDirMask;

	printf("mask of	the pins: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	PinNum = (DWORD) data;
	printf("PinDirMask(0: output, 1: input): 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	PinDirMask = (DWORD) data;
	result = SusiIOSetDirectionMulti(PinNum, &PinDirMask);
	if (result == FALSE) 
	{
		printf("SusiIOSetDirectionMulti() failed\n");
		return 1;
	}
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

	result = SusiIOAvailable();
	if (result == 0) {
		printf("SusiIOAvailable() failed\n");
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
			result = get_dir_mask();
			break;
		case 2:
			result = read_pin();
			break;
		case 3:
			result = read_pins();
			break;
		case 4:
			result = write_pin();
			break;
		case 5:
			result = write_pins();
			break;
		case 6:
			result = set_pin_direction();//PCA9554 gpio
			break;
		case 7:
			result = set_pins_direction();	
			break;		
		default:
			printf("\nUnknown choice!\n\n");
			continue;
		}
		if (done != 1 && result < 0) {
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

