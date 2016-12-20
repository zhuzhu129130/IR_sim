/* File : ftcamif.c */

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <stdint.h> 
#include <unistd.h>

#include "ftd2xx.h"
#include "ftcamif.h"

static FT_HANDLE dev_handle = NULL;

static uint8_t rxbuffer[RXBUF_SIZE];
static int rxindex;
static ULONG rxvalid;

static FT_HANDLE dev_handle1 = NULL;

static uint8_t rxbuffer1[RXBUF_SIZE];
static int rxindex1;
static ULONG rxvalid1;

static void checkFtStatus(FT_STATUS st, char *msg)
{
	if (st != FT_OK) {
		printf ("%s Error: %d\n", msg, st);
	}
}

ftcamif::ftcamif()
{
}

int ftcamif::opendev(char *buf, int size)
{
	FT_STATUS ftStatus;
	printf("serial number: %s\n", buf);
	if((ftStatus = FT_OpenEx(buf, FT_OPEN_BY_SERIAL_NUMBER, &dev_handle)) != FT_OK) {
		printf("Error. FT_OpenEx() return: %d\n", (int)ftStatus);
		printf("Use lsmod to check if ftdi_sio (and usbserial) are present.\n");
		printf("If so, unload them using rmmod, as they conflict with ftd2xx.\n");
		return -1;
	}

	ftStatus = FT_ResetDevice(dev_handle);//给设备发送一个复位命令
	checkFtStatus(ftStatus, "FT_ResetDevice()");
    usleep(200000);

	ftStatus = FT_SetBitMode(dev_handle, 0xFF, 0x00);//设置设备的引脚都为输出（0xFF），并且reset
	checkFtStatus(ftStatus, "FT_SetBitMode0()");
	usleep(200000);

	ftStatus = FT_SetBitMode(dev_handle, 0xFF, 0x40);//设置设备的引脚都为输出，并为单通道同步245 FIFO Mode
	checkFtStatus(ftStatus, "FT_SetBitMode1");

	ftStatus = FT_SetTimeouts(dev_handle, 5000, 5000);	// set read timeout & write timeout to 100ms
	checkFtStatus(ftStatus, "FT_SetTimeouts()");
	
	ftStatus = FT_SetLatencyTimer(dev_handle, 255);	// !!! 2~255设置延时定时器的值
	checkFtStatus(ftStatus, "FT_SetLatencyTimer()");

	ftStatus = FT_SetUSBParameters(dev_handle, 0x10000, 0x10000);//设置USB请求传输器大小
	checkFtStatus(ftStatus, "FT_SetUSBParameters()");
	
	ftStatus = FT_SetFlowControl(dev_handle,FT_FLOW_RTS_CTS, 0, 0);//设置设备的流控制
	checkFtStatus(ftStatus, "FT_SetFlowControl()");

	rxindex = 0;
	rxvalid = 0;

    return 0;
}

int ftcamif::closedev()
{
    if (!dev_handle) {
        printf ("Device not opened.\n");
        return -1;
    }

	FT_Close(dev_handle);
    return 0;
}

int ftcamif::sendCommand(char *buf, int size)
{
	DWORD bytesWriten;
	FT_STATUS ftStatus;
    if (!dev_handle) {
        printf("Device not opened.\n"); 
        return -1;
    }
	ftStatus = FT_Write(dev_handle, buf, size, &bytesWriten);
	if (ftStatus != FT_OK) {
		printf("Error FT_Write(%d)\n", (int)ftStatus);
		return -1;
	}
	return bytesWriten;
}

int ftcamif::findHead(int retry)
{
	int i, j;
	FT_STATUS ftStatus;

	enum {
		ST_NOT_FOUND = 0,
		ST_SIGN_FF,
		ST_SIGN_0
	} state = ST_NOT_FOUND;

	DWORD rxn, txn, event;

	for (i = 0; i < retry; i ++) {
		if (rxindex >= rxvalid) {
			ftStatus = FT_GetStatus(dev_handle, &rxn, &txn, &event);
			//ftStatus = FT_Read(dev_handle, (uint8_t *)rxbuffer, 
			//				RXBUF_SIZE * sizeof(uint16_t), &rxvalid);
			ftStatus = FT_Read(dev_handle, (uint8_t *)rxbuffer, 
							RXBUF_SIZE * sizeof(uint8_t), &rxvalid);
			//printf("rxn = %d, event = 0x%x, rxvalid = %d.\n", rxn, event, rxvalid);
			if (ftStatus != FT_OK) {
				printf("ERROR reading.\n");
				rxvalid = 0;
				continue;
			}
			rxindex = 0;
			//rxvalid >>= 1;	// byte to words
			/*
			for (j = 0; j < 16; j ++) 
				printf("0x%04x, ", rxbuffer[j]);
			printf("\n");
			*/
		}

		for ( ; rxindex < rxvalid; rxindex ++) {
			switch (state) {
				case ST_NOT_FOUND:
					if (rxbuffer[rxindex] == 0xFF) {
						//printf ("0x%04x, 0x%04x, 0x%04x\n", 
						//		rxbuffer[rxindex -1], rxbuffer[rxindex], rxbuffer[rxindex + 1]);
						state = ST_SIGN_FF;
					}
					break;
				case ST_SIGN_FF:
					if (rxbuffer[rxindex] == 0xFF) {
						state = ST_SIGN_0;
					} else {
						state = ST_NOT_FOUND;
					}
					break;
				case ST_SIGN_0:
					if (rxbuffer[rxindex] == 0x00) {	// found header
						rxindex ++;
						return 0;
					} else if (rxbuffer[rxindex] == 0xFF) {
						state = ST_SIGN_0;
					} else {
						state = ST_NOT_FOUND;
					}
					break;
				default:
					state = ST_NOT_FOUND;
					break;
			}
		}
	}
	return -1;
}

int ftcamif::getFrame(char *buf, int size, int retry)
{
	FT_STATUS ftStatus;
	int retry_cnt;
	int copysize;
	DWORD rxn, txn, event;

    if (!dev_handle) {
        printf ( "Device not opened.\n" ); 
        return -1;
    }

	ftStatus = FT_RestartInTask(dev_handle);
	checkFtStatus(ftStatus, "FT_RestartInTask()");

    if (findHead(retry) < 0) {	// not found
		printf("miss header.\n");
		ftStatus = FT_StopInTask(dev_handle);
		checkFtStatus(ftStatus, "FT_StopInTask0()");
		return -1;
	}
	//printf("memcpy: %02x %02x %02x %02x \n", 
	//		rxbuffer[rxindex], rxbuffer[rxindex + 1], rxbuffer[rxindex + 2], rxbuffer[rxindex + 3]);

	// copy data
	retry_cnt = 0;
	while(retry_cnt < retry && size > 0) {
		if (rxindex < rxvalid) {
			//if (size > (rxvalid - rxindex) * sizeof(uint16_t))
			if (size > (rxvalid - rxindex) * sizeof(uint8_t))
				//copysize = (rxvalid - rxindex) * sizeof(uint16_t);
				copysize = (rxvalid - rxindex) * sizeof(uint8_t);
			else
				copysize = size;
			memcpy(buf, &rxbuffer[rxindex], copysize);
			buf += copysize;
			size -= copysize;
			//rxindex += (copysize >> 1);
			rxindex += copysize;
		}

		//ftStatus = FT_GetStatus(dev_handle, &rxn, &txn, &event);
		//ftStatus = FT_Read(dev_handle, (unsigned char*)rxbuffer, 
		//				RXBUF_SIZE * sizeof(uint16_t), &rxvalid);
		ftStatus = FT_Read(dev_handle, (unsigned char*)rxbuffer, 
						RXBUF_SIZE * sizeof(uint8_t), &rxvalid);
		if (ftStatus != FT_OK) {
			printf ("ERROR D.\n");
			rxvalid = 0;
			retry_cnt ++;
			continue;
		}
		rxindex = 0;
		//rxvalid >>= 1;	// byte to words
	}

	if (size != 0) {
		printf("not all data recieved, %d left.\n", size);
	}

	//rxindex = 0;
	//rxvalid = 0;

	FT_StopInTask(dev_handle);
	checkFtStatus(ftStatus, "FT_StopInTask1()");

    return size;	// '0' means all request bytes are get
}

ftcamif1::ftcamif1()
{
}
int ftcamif1::opendev(char *buf, int size)
{
    FT_STATUS ftStatus;
    printf("serial number: %s\n", buf);
    if((ftStatus = FT_OpenEx(buf, FT_OPEN_BY_SERIAL_NUMBER, &dev_handle1)) != FT_OK) {
        printf("Error. FT_OpenEx() return: %d\n", (int)ftStatus);
        printf("Use lsmod to check if ftdi_sio (and usbserial) are present.\n");
        printf("If so, unload them using rmmod, as they conflict with ftd2xx.\n");
        return -1;
    }

    ftStatus = FT_ResetDevice(dev_handle1);
    checkFtStatus(ftStatus, "FT_ResetDevice()");
    usleep(200000);

    ftStatus = FT_SetBitMode(dev_handle1, 0xFF, 0x00);
    checkFtStatus(ftStatus, "FT_SetBitMode0()");
    usleep(200000);

    ftStatus = FT_SetBitMode(dev_handle1, 0xFF, 0x40);
    checkFtStatus(ftStatus, "FT_SetBitMode1");

    ftStatus = FT_SetTimeouts(dev_handle1, 5000, 5000);	// set read timeout & write timeout to 100ms
    checkFtStatus(ftStatus, "FT_SetTimeouts()");

    ftStatus = FT_SetLatencyTimer(dev_handle1, 255);	// !!! 2~255
    checkFtStatus(ftStatus, "FT_SetLatencyTimer()");

    ftStatus = FT_SetUSBParameters(dev_handle1, 0x10000, 0x10000);
    checkFtStatus(ftStatus, "FT_SetUSBParameters()");

    ftStatus = FT_SetFlowControl(dev_handle1,FT_FLOW_RTS_CTS, 0, 0);
    checkFtStatus(ftStatus, "FT_SetFlowControl()");

    rxindex1 = 0;
    rxvalid1 = 0;

    return 0;
}

int ftcamif1::closedev()
{
    if (!dev_handle1) {
        printf ("Device not opened.\n");
        return -1;
    }

    FT_Close(dev_handle1);
    return 0;
}

int ftcamif1::sendCommand(char *buf, int size)
{
    DWORD bytesWriten;
    FT_STATUS ftStatus;
    if (!dev_handle1) {
        printf("Device not opened.\n");
        return -1;
    }
    ftStatus = FT_Write(dev_handle1, buf, size, &bytesWriten);
    if (ftStatus != FT_OK) {
        printf("Error FT_Write(%d)\n", (int)ftStatus);
        return -1;
    }
    return bytesWriten;
}

int ftcamif1::findHead(int retry)
{
    int i, j;
    FT_STATUS ftStatus;

    enum {
        ST_NOT_FOUND = 0,
        ST_SIGN_FF,
        ST_SIGN_0
    } state = ST_NOT_FOUND;

    DWORD rxn, txn, event;

    for (i = 0; i < retry; i ++) {
        if (rxindex1 >= rxvalid1) {
            ftStatus = FT_GetStatus(dev_handle1, &rxn, &txn, &event);
            //ftStatus = FT_Read(dev_handle, (uint8_t *)rxbuffer,
            //				RXBUF_SIZE * sizeof(uint16_t), &rxvalid);
            ftStatus = FT_Read(dev_handle1, (uint8_t *)rxbuffer1,
                            RXBUF_SIZE * sizeof(uint8_t), &rxvalid1);
            //printf("rxn = %d, event = 0x%x, rxvalid = %d.\n", rxn, event, rxvalid);
            if (ftStatus != FT_OK) {
                printf("ERROR reading.\n");
                rxvalid1 = 0;
                continue;
            }
            rxindex1 = 0;
            //rxvalid >>= 1;	// byte to words
            /*
            for (j = 0; j < 16; j ++)
                printf("0x%04x, ", rxbuffer[j]);
            printf("\n");
            */
        }

        for ( ; rxindex1 < rxvalid1; rxindex1 ++) {
            switch (state) {
                case ST_NOT_FOUND:
                    if (rxbuffer1[rxindex1] == 0xFF) {
                        //printf ("0x%04x, 0x%04x, 0x%04x\n",
                        //		rxbuffer[rxindex -1], rxbuffer[rxindex], rxbuffer[rxindex + 1]);
                        state = ST_SIGN_FF;
                    }
                    break;
                case ST_SIGN_FF:
                    if (rxbuffer1[rxindex1] == 0xFF) {
                        state = ST_SIGN_0;
                    } else {
                        state = ST_NOT_FOUND;
                    }
                    break;
                case ST_SIGN_0:
                    if (rxbuffer1[rxindex1] == 0x00) {	// found header
                        rxindex1 ++;
                        return 0;
                    } else if (rxbuffer1[rxindex1] == 0xFF) {
                        state = ST_SIGN_0;
                    } else {
                        state = ST_NOT_FOUND;
                    }
                    break;
                default:
                    state = ST_NOT_FOUND;
                    break;
            }
        }
    }
    return -1;
}

int ftcamif1::getFrame(char *buf, int size, int retry)
{
    FT_STATUS ftStatus;
    int retry_cnt;
    int copysize;
    DWORD rxn, txn, event;

    if (!dev_handle1) {
        printf ( "Device not opened.\n" );
        return -1;
    }

    ftStatus = FT_RestartInTask(dev_handle1);
    checkFtStatus(ftStatus, "FT_RestartInTask()");

    if (findHead(retry) < 0) {	// not found
        printf("miss header.\n");
        ftStatus = FT_StopInTask(dev_handle1);
        checkFtStatus(ftStatus, "FT_StopInTask0()");
        return -1;
    }
    //printf("memcpy: %02x %02x %02x %02x \n",
    //		rxbuffer[rxindex], rxbuffer[rxindex + 1], rxbuffer[rxindex + 2], rxbuffer[rxindex + 3]);

    // copy data
    retry_cnt = 0;
    while(retry_cnt < retry && size > 0) {
        if (rxindex1 < rxvalid1) {
            //if (size > (rxvalid - rxindex) * sizeof(uint16_t))
            if (size > (rxvalid1 - rxindex1) * sizeof(uint8_t))
                //copysize = (rxvalid - rxindex) * sizeof(uint16_t);
                copysize = (rxvalid1 - rxindex1) * sizeof(uint8_t);
            else
                copysize = size;
            memcpy(buf, &rxbuffer1[rxindex1], copysize);
            buf += copysize;
            size -= copysize;
            //rxindex += (copysize >> 1);
            rxindex1 += copysize;
        }

        //ftStatus = FT_GetStatus(dev_handle, &rxn, &txn, &event);
        //ftStatus = FT_Read(dev_handle, (unsigned char*)rxbuffer,
        //				RXBUF_SIZE * sizeof(uint16_t), &rxvalid);
        ftStatus = FT_Read(dev_handle1, (unsigned char*)rxbuffer1,
                        RXBUF_SIZE * sizeof(uint8_t), &rxvalid1);
        if (ftStatus != FT_OK) {
            printf ("ERROR D.\n");
            rxvalid1 = 0;
            retry_cnt ++;
            continue;
        }
        rxindex1 = 0;
        //rxvalid >>= 1;	// byte to words
    }

    if (size != 0) {
        printf("not all data recieved, %d left.\n", size);
    }

    //rxindex = 0;
    //rxvalid = 0;

    FT_StopInTask(dev_handle1);
    checkFtStatus(ftStatus, "FT_StopInTask1()");

    return size;	// '0' means all request bytes are get
}

