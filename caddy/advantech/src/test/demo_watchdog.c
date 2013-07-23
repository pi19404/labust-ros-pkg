/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_watchdog.c
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
	printf("1) Set Watchdog timeout value\n");
	printf("2) Ping/Trigger the Watchdog\n");
	printf("3) Stop/Disable the Watchdog\n");
	printf("Enter your choice: ");
}

void get_delay_timeout(DWORD min, DWORD max, DWORD step, DWORD *delay, DWORD *timeout)
{
	int done = 0;

	printf("Delay (in m-sec): ");
	if (scanf("%li", delay) <= 0)
		return;
	if (step) {
		while (! done) {
			printf("Timeout (in m-sec): ");
			if ((scanf("%li", timeout) <= 0) || *timeout < min || *timeout > max) {
				printf("\"Timeout\" should be between ");
				printf("%li and %li\n", min, max);
				continue;
			}
			done = 1;
		}
	}
	else {
		printf("This Watchdog does not support stepping.\n");
		printf("Set \"timeout\" to the default value.\n");
		*timeout = min;
	}
}

int main(void)
{
	int result, done, op;
	DWORD delay, timeout;
	DWORD min, max, step;

	result = SusiDllInit();
	if (result == FALSE) {
		printf("SusiDllInit() failed\n");
		return 1;
	}

	result = SusiWDAvailable();
	if (result == 0) {
		printf("SusiWDTAvailable() failed\n");
		SusiDllUnInit();
		return 1;
	}

	result = show_platform_info();

	result = SusiWDGetRange(&min, &max, &step);
	if (result == FALSE) {
		printf("SusiWDTGetRange() failed\n");
		SusiDllUnInit();
		return 1;
	}
	else
		printf("Timeout value: (min, max, step) = (%ld, %ld, %ld)\n", min, max, step);

	done = 0;
	while (! done) {
		show_menu();
		if (scanf("%i", &op) <= 0)
			op = -1;

		switch(op) {
		case 0:
			done = 1;
			continue;
		case 1:
			get_delay_timeout(min, max, step, &delay, &timeout);
			result = SusiWDSetConfigEx(WDTOUT0, delay, timeout);
			break;
		case 2:
			result = SusiWDTriggerEx(WDTOUT0);
			break;
		case 3:
			result = SusiWDDisableEx(WDTOUT0);
			break;
		default:
			printf("\nUnknown choice!\n\n");
			continue;
		}
		if (result == FALSE) {
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

