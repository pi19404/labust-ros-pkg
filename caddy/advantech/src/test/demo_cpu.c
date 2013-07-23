/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_cpu.c
*
* Programmers:
*	Neo Lo
*
****************************************************************************/

#include <stdio.h>
#include <unistd.h>
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

static int check_file(const char *file_name)
{
    /* check file exist */
	if ((access(file_name, F_OK|R_OK)) != 0) 
    {
	    return -1;
	}

    return 0;
}

static int show_all_cpu_speed(void)
{
    char buff[64];
    FILE *fd;
    int i;

    printf("Current CPU Speed:\n");

    //MAX support 4 CPU
    for (i = 0; i < 4; i++)
    {
        sprintf(buff, "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", i);
        if (check_file(buff) != 0)
    		break;
    	if ((fd = fopen(buff, "r")) == NULL)
            break;

        if (fgets(buff, 32, fd) != NULL)
            printf("  CPU%d Hz: %s", i, buff);
        
        fclose(fd);
    }

    if (i == 0)
        printf("Your system does not support CPU freq in sysfs.\nYou can use command [cat /proc/cpuinfo] to check it.\n");

	return 0;
}

int get_mode(void)
{
    BYTE CpuMode;

    if ((SusiPlusSpeedRead(&CpuMode, NULL) != 0) ||
        (CpuMode > 2))
    {
        printf("SusiPlusSpeedRead() failed\n");
        return -1;
    }

    printf("CPU SpeedStep Mode is (%d)", CpuMode);

    switch(CpuMode)
    {
        case CORE_CPU_FULL_SPEED:
            printf("CORE_CPU_FULL_SPEED\n");
            break;
        case CORE_CPU_LOW_SPEED:
            printf("CORE_CPU_LOW_SPEED\n");
            break;
        case CORE_CPU_DYNAMIC:
            printf("CORE_CPU_DYNAMIC\n");
            break;
        default:
            break;
    }

    return 0;
}

int set_mode(void)
{
    int CpuMode;

retry:
    printf("\nSet CPU SpeedStep Mode:\n");
	printf("0) CORE_CPU_FULL_SPEED\n");
	printf("1) CORE_CPU_LOW_SPEED\n");
	printf("2) CORE_CPU_DYNAMIC\n");
    printf("Enter your choice: ");

	if (scanf("%i", &CpuMode) <= 0)
		return 1;
	
    if (CpuMode > 2)
    {
        printf("CpuMode must be 0, 1, 2.\n");
        goto retry;
    }

    if (SusiPlusSpeedWrite(CpuMode, 0) != 0)
    {
        printf("SusiPlusSpeedWrite() failed\n");
        return -1;
    }

    return 0;
}

void show_menu(void)
{
	printf("\n");
    show_all_cpu_speed();
	printf("0) Terminate this program\n");
	printf("1) Get CPU SpeedStep Mode\n");
	printf("2) Set CPU SpeedStep Mode\n");
	printf("Enter your choice: ");
}

int main(void)
{
	int result;
	int done, op;

	result = SusiDllInit();
	if (result == FALSE) {
		printf("SusiDllInit() failed\n");
		return 1;
	}

	result = SusiCoreAvailable();
	if (result == 0) {
		printf("SusiCoreAvailable() failed\n");
		SusiDllUnInit();
		return 1;
	}

	show_platform_info();

	result = SusiPlusSpeedSetActive();
	if (result != 0) {
		printf("SusiPlusSpeedSetActive() failed\n");
        printf("CPU or OS does not support speedstep!!\n");
		SusiDllUnInit();
		return 1;
	}

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
			result = get_mode();
			break;
		case 2:
			result = set_mode();
			break;
		default:
			printf("\nUnknown choice!\n\n");
			continue;
		}
		if (result != 0) {
			SusiDllUnInit();
			return 1;
		}
	}

	result = SusiPlusSpeedSetInactive();
	if (result != 0) {
		printf("SusiPlusSpeedSetInactive() failed\n");
		SusiDllUnInit();
		return 1;
	}

	result = SusiDllUnInit();
	if (result == FALSE) {
		printf("SusiDllUnInit() failed\n");
		return 1;
	}

	return 0;
}

