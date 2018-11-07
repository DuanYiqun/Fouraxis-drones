#ifndef __I2C_H
#define __I2C_H
//I2C软件模拟时序驱动
extern unsigned char ack;	              //应答标志位 
unsigned char ISendStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no);
// 向有子地址器件发送多字节数据函数
unsigned char IRcvStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no);
//向有子地址器件读取多字节数据函数
unsigned char ISendChar(unsigned char sla,unsigned char suba,unsigned char s);

#endif
