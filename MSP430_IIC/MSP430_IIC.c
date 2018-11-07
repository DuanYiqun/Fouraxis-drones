
#include "i2c.h"
int msp430_i2c_write(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char const *data)
{
	ISendStr(slave_addr,reg_addr,data,length);
    return 0;
}

int msp430_i2c_read(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char *data)
{
	IRcvStr(slave_addr,reg_addr,data,length);
	return 0;
}
