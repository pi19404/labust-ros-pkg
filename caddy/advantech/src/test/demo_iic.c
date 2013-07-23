/****************************************************************************
* Copyright (c) Advantech Co., Ltd. All Rights Reserved
*
* File Name:
*	demo_iic.c
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
	printf("1) Read for i2c\n");
	printf("2) Write to i2c\n");
	printf("3) Write Read Combine\n");
	printf("Enter your choice: ");
}

// Return 0 on success and 1 on failure.
int read_byte(void)
{
	int result, data, i, Len = -1;
	BYTE address, *storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Len: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	Len = data;

    if ( ( Len > 0 ) && ((storage = (BYTE *)malloc(Len)) != 0) )
    {
        result = SusiIICRead(SUSI_IIC_TYPE_PRIMARY, address, storage, Len);
    	//result = SusiIICRead(address, offset, &storage);
    	if (result == FALSE) {
    		printf("SusiIICRead() failed\n");
    		return 1;
    	}
    	else
    	{
            printf("Data read:");
            for ( i = 0; i < Len; i++)
            {
                
    		    printf("0x%02x, ", storage[i]);
            }
            printf("\n");
            printf("Read successful.\n");
    	}

        free(storage);
    	return 0;
    }
    return 1;
}

// Return 0 on success and 1 on failure.
int write_byte(void)
{
	int result, data, i, Len = -1;
	BYTE address, *storage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Len: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	Len =  data;

    if ( (Len > 0) && ((storage = (BYTE *)malloc(Len)) != 0) )
    {
        for ( i = 0; i < Len; i++)
        {
        	printf("data to write[0x%x]: 0x", i);
        	if (scanf("%x", &data) <= 0)
				return 1;
        	storage[i] = (BYTE) data;
        }

        result = SusiIICWrite(SUSI_IIC_TYPE_PRIMARY, address, storage, Len);
    	//result = SusiIICWrite(address, offset, storage);
    	if (result == FALSE) {
    		printf("SusiIICWrite() failed\n");
    		return 1;
    	}
        printf("Write successful.\n");
    	return 0;
    }
    return 1;
}

int write_read_combine(void)
{
	int result, data, i, writeLen = -1, readLen = -1;
	BYTE address, *wstorage, *rstorage;

	printf("Address of the slave device: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	address = (BYTE) data;

	printf("Write Len: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	writeLen = data;
    
	printf("Read Len: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	readLen =  data;

    if ( (writeLen > 0) && ((wstorage = (BYTE *)malloc(writeLen)) != 0) &&
          (readLen > 0) && ((rstorage = (BYTE *)malloc(readLen)) != 0))
    {
        for ( i = 0; i < writeLen; i++)
        {
        	printf("data to write[0x%x]: 0x", i);
        	if (scanf("%x", &data) <= 0)
			return 1;
        	wstorage[i] = (BYTE) data;
        }

        result = SusiIICWriteReadCombine(SUSI_IIC_TYPE_PRIMARY, address, wstorage, writeLen, rstorage, readLen);
    	if (result == FALSE) {
    		printf("SusiIICWriteReadCombine() failed\n");
    		return 1;
    	}
    	else
    	{
            printf("Data read:");
            for ( i = 0; i < readLen; i++)
            {
                
    		    printf("0x%02x, ", rstorage[i]);
            }
            printf("\n");
            printf("Read successful.\n");
    	}

        free(wstorage);
        free(rstorage);
        
    	return 0;
        
    }

    return 1;
}

// Return 0 on success and 1 on failure.
int scan_i2c(void)
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

	printf("One byte data to write: 0x");
	if (scanf("%x", &data) <= 0)
		return 1;
	storage = (BYTE) data;

    result = SusiIICWrite(SUSI_IIC_TYPE_PRIMARY, address+offset, &storage, sizeof(BYTE));
	//result = SusiIICWrite(address, offset, storage);
	if (result == FALSE) {
		printf("SusiIICWrite() failed\n");
		return 1;
	}
	return 0;
}

int main(void)
{
	int result, done, op;

	result = SusiInit();
	if (result == FALSE) {
		printf("SusiInit() failed\n");
		return 1;
	}

	result = SusiIICAvailable();
	if (result == 0) {
		printf("SusiIICAvailable() failed\n");
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
			result = write_byte();
			break;
		case 3:
            result = write_read_combine();
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

	result = SusiDllUnInit();
	if (result == FALSE) {
		printf("SusiDllUnInit() failed\n");
		return 1;
	}

	return 0;
}

