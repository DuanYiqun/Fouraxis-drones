int msp430_i2c_write(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char const *data);
int msp430_i2c_read(unsigned char slave_addr,unsigned char reg_addr,unsigned char length,unsigned char *data);
