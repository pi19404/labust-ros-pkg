/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_brightness.c
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
	printf("1) Get brightness range\n");
	printf("2) Get brightness value\n");
	printf("3) Set brightness value\n");
	printf("Enter your choice: ");
}

// Return 0 on success and 1 on failure.
int get_bright_rang(void)
{
	int result;
	BYTE minimum, maximum, stepping;

    minimum = maximum = stepping = 0;
	result = SusiVCGetBrightRange(&minimum, &maximum, &stepping);
	if (result == FALSE) {
		printf("SusiVCGetBrightRange() failed\n");
		return 1;
	}
	else
	{
		printf("(minimum, maximum, stepping) = (%i, %i, %i)\n", minimum, maximum, stepping);
	}
    
	return 0;
}

// Return 0 on success and 1 on failure.
int get_bright(void)
{
	int result;
    BYTE brightness = 0;

	result = SusiVCGetBright(&brightness);
	if (result == FALSE) {
		printf("SusiVCGetBright() failed\n");
		return 1;
	}
	else
		printf("Value: %i\n", brightness);
	return 0;
}

// Return 0 on success and 1 on failure.
int set_bright(void)
{
	int result;
	int brightness = 0;

	printf("Brightness Value: ");
	if (scanf("%i", &brightness) <= 0)
		return 1;
	result = SusiVCSetBright(brightness);
	if (result == FALSE) {
		printf("SusiVCSetBright() failed\n");
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

	result = SusiVCAvailable();
	if ((result & VC_BRIGHT_CTL_SUPPORT) == 0) {
		printf("SusiVCAvailable() = %d failed\n", result);
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
			result = get_bright_rang();
			break;
		case 2:
			result = get_bright();
			break;
		case 3:
			result = set_bright();
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

