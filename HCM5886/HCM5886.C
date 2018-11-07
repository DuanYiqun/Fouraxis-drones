/*
 * HCM5886.C
 *
 *  Created on: 2015-3-14
 * 
 */

#include<msp430f5529.h>
#include<i2c.h>
#include<math.h>
unsigned char Dat[7] = {0};

typedef struct
{
	int HMC_X,HMC_Y,HMC_Z;
	float Angle;
	long int Angle_Add;
	unsigned char count;

}YallAngle;
extern YallAngle Yall_Angle;

void Get_Angle()
{
	IRcvStr(0x3C,0x03,Dat,6);
	Yall_Angle.HMC_X = (unsigned int)Dat[0]<<8 | (unsigned int)Dat[1];
	Yall_Angle.HMC_Y = (unsigned int)Dat[4]<<8 | (unsigned int)Dat[5];
	Yall_Angle.Angle_Add += (long int)((atan2((double)Yall_Angle.HMC_Y,(double)Yall_Angle.HMC_X) * (180 / 3.14159265))*100);
	if(++Yall_Angle.count == 5)
	{
		Yall_Angle.count = 0;
		Yall_Angle.Angle = Yall_Angle.Angle_Add/500.0;
		Yall_Angle.Angle_Add = 0;
	}

}

void HMC5583_Init()
{
	ISendChar(0x3c,0x00,0x78);
	ISendChar(0x3c,0x01,0x20);
	ISendChar(0x3c,0x02,0x00);

}
