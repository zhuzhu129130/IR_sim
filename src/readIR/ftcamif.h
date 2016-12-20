#ifndef __FTCAMIF_H__
#define __FTCAMIF_H__
#include "ftd2xx.h"
#include "ftcamif.h"
//#define RXBUF_SIZE 16384
#define RXBUF_SIZE 81920+5//65536  //这里设置的是一帧图像的大小


class ftcamif
{
public:
    ftcamif();
    int opendev(char *buf, int size);
    int closedev();
    int sendCommand(char *buf, int size);
    int findHead(int retry);
    int getFrame(char *buf, int size, int retry);

public:

};

class ftcamif1
{
public:
    ftcamif1();
    int opendev(char *buf, int size);
    int closedev();
    int sendCommand(char *buf, int size);
    int findHead(int retry);
    int getFrame(char *buf, int size, int retry);

public:

};

#endif //__FTCAMIF_H__
