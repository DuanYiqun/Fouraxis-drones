/*
 * bluth.h
 *
 *  Created on: 2015Äê5ÔÂ23ÈÕ
 */

#ifndef BLUTH_BLUTH_H_
#define BLUTH_BLUTH_H_

void Bluth_UART_Init();
void Data_Receive_Anl(unsigned char *data_buf,unsigned char num);
void Data_Receive_Ctl(unsigned char *data_buf,unsigned char num);
void Data_Send_Status(void);
void Data_Send_Senser(void);
void Data_Send_PID1(void);
void Data_Send_MotoPWM(void);

#endif /* BLUTH_BLUTH_H_ */
