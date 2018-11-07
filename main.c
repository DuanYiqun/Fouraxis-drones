
#include<msp430f5529.h>
#include<stdio.h>
#include"int_clk.h"
#include"sto_int.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include <bluth.h>
#include"i2c.h"
#include <HCM5886.H>
#define DEFAULT_MPU_HZ  (100)
#define uchar unsigned char
#define uint unsigned int
#define STILL 0
#define DELTA_TIME 0.003
//#define Roll_Moto_Offset 55
//#define Roll_Moto_Offset_84 35
//#define Pitch_Moto_Offset 70
uchar Start_Flag = 0;
uchar CpuTimePoll;
uchar Uart_Work_Flag = 0;
uchar Sta_Ctl_Flag = 0;
uchar Hcm_Cal_Flag = 0;
uchar time = 0;
uchar time1 = 0;   //此位用来看串口是否一直正常通信。
uint Pitch_Moto_Offset = 0;
uint Roll_Moto_Offset = 0;
float Yall_Angle_Offset = 0;


/*****************************************变量定义区***************************/
/******************************************变量定义区***************************/
/******************************************变量定义区***************************/
typedef struct
{
	int HMC_X,HMC_Y,HMC_Z;
	float Angle;
	long int Angle_Add;
	unsigned char count;
}YallAngle;
YallAngle Yall_Angle={0,0,0,0,0,0};  //磁力计的数据结构


typedef struct
{
	float yall;
	float pitch;
	float roll;    //陀螺仪的三个角度 偏航 俯仰 翻滚
}AttAngle;
AttAngle Att_Angle;

typedef struct
{
	short Acc_X;
	short Acc_Y;
	short Acc_Z;
	short Gyro_X;
	short Gyro_Y;
	short Gyro_Z;
}SensorDat;
SensorDat Sensor_Dat;


float w,x,y,z;           //四元数
float temp1,temp2;
float temp3,temp4,temp5;      //四元数转欧拉角过程变量
float gyro_roll;


typedef struct
{
	uint CtlThro;	//映射后油门百分比
	uint Thro;			//油门具体占空比
	int motor_front_pwm,motor_back_pwm,motor_left_pwm,motor_right_pwm; //四个电机最终输出的占空比
	uchar CtlYall;  //控制方向
	uchar CtlRoll;  //控制ROLL倾斜角度
	uchar CtlPitch;	//控制Pitch倾斜角度
	int CtlXdirPwm;
	int CtlYdirPwm;
	float XdirPwm;
	float YdirPwm;
	int XdirAngle;
	int YdirAngle;
	int ZdirAngle;
}PwmPara;
PwmPara Pwm_Para = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


typedef struct
{
	int pid_pitch_pwm;
	int pid_roll_pwm;  //PID的输出值
	int pid_yall_pwm;
}PidOut;
PidOut Pid_Out ={0,0,0};


typedef struct  //飞控PID参数以及初始化
{
	float In_KP;
	float In_KI;
	float In_KD;
	float Pos_Max;
	float Neg_Max;
	float Last_output;
	float Pid_LastError;  //Pid上一次于目标的差值
	float integrator;  //积分项
	float derivative;  //微分项
}PidPara;
PidPara Pid_Para_Roll ={0,0,0,300,-300,0,0,0,0};
PidPara Pid_Para_Pitch ={0,0,0,300,-300,0,0,0,0};
PidPara Pid_Para_Yall ={0,0,0,300,-300,0,0,0,0};

//
static signed char gyro_orientation[9] = {-1, 0, 0,0,-1, 0,0, 0, 1};  //#原文有注释参见(motion_driver_test.c第79行)
volatile unsigned char new_gyro;  //#原为hal.new_gyro(参见motion_driver_test.c的55行)

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
static inline void run_self_test(void);
//
unsigned long sensor_timestamp;
unsigned char more;
short gyro[3],accel[3],sensors;
long quad[4];

unsigned short dmp_features;
//ESP6288


uchar temp_buf;
uchar DatRx[32] = {0};
uchar Pid_Rx_Flag = 0 ;
uchar Pid_Set_OK = 0;
uchar Ctl_Rx_Flag = 0;
uchar Pid_Rx_Count = 0;
uchar Ctl_Rx_Count = 0;
uchar Dat_num = 0;


/**************************************PWM******************************************/
/**************************************PWM******************************************/
/**************************************PWM******************************************/

void PWM_INIT()
{
	TA0CTL |= TASSEL_1;   //用ACLK
	TA0CCTL1 |= OUTMOD_6; //p1.2PWM口
	TA0CCTL2 |= OUTMOD_6; //p1.3PWM口
	TA0CCTL3 |= OUTMOD_6; //p1.4WM口
	TA0CCTL4 |= OUTMOD_6; //p1.5PWM口     toggle/reset
	TA0CCR0 = 12500;        //增计数上限  周期为0.5ms SMCLK=25000000hz   2KHZ
	TA0CTL |= MC_1;      //增计数模式
	P1DIR |= BIT2 + BIT3 + BIT4 + BIT5; //p1.2345输出
	P1SEL |= BIT2 + BIT3 + BIT4 + BIT5;  //p1.2345 作为第二功能口TA0.1234
}

void PID_ROLL()
{
	float Pid_Output=0;  //PID总输出
	float Pid_Error;   //Pid于目标的差值
	//计算外环角度Pid输出
	Pid_Error = Att_Angle.roll - STILL - Pwm_Para.YdirAngle;  //减去平衡点零点误差 以及倾斜飞行角度

	//计算P项
	Pid_Output += Pid_Para_Roll.In_KP*Pid_Error;

	//计算D项
	Pid_Para_Roll.derivative = Sensor_Dat.Gyro_X/10;                 //(Pid_Error - Pid_LastError) / DELTA_TIME;  //△T等于中断时间
	Pid_Output += Pid_Para_Roll.In_KD*Pid_Para_Roll.derivative;
	Pid_Para_Roll.Pid_LastError = Pid_Error;

	//计算I项
	Pid_Para_Roll.integrator += Pid_Para_Roll.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Roll.integrator > Pid_Para_Roll.Pos_Max)
		Pid_Para_Roll.integrator = Pid_Para_Roll.Pos_Max;
	else if(Pid_Para_Roll.integrator < Pid_Para_Roll.Neg_Max)
		Pid_Para_Roll.integrator = Pid_Para_Roll.Neg_Max;       //积分项限幅

	Pid_Output +=  Pid_Para_Roll.integrator;

	Pid_Out.pid_roll_pwm = Pid_Output;


}


void PID_PITCH()
{
	float Pid_Output = 0;
	float Pid_Error;
	Pid_Error = Att_Angle.pitch - STILL - Pwm_Para.XdirAngle;  //减去平衡点零点误差 以及倾斜飞行角度

	//计算P项
	Pid_Output += Pid_Para_Pitch.In_KP*Pid_Error;

	//计算D项
	Pid_Para_Pitch.derivative = -Sensor_Dat.Gyro_Y/10;			//(Pid_Error - Pid_LastError) / DELTA_TIME;  //△T等于中断时间
	Pid_Output += Pid_Para_Pitch.In_KD*Pid_Para_Pitch.derivative;
	Pid_Para_Pitch.Pid_LastError = Pid_Error;

	//计算I项
	Pid_Para_Pitch.integrator += Pid_Para_Pitch.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Pitch.integrator > Pid_Para_Pitch.Pos_Max)
		Pid_Para_Pitch.integrator = Pid_Para_Pitch.Pos_Max;
	else if(Pid_Para_Pitch.integrator < Pid_Para_Pitch.Neg_Max)
		Pid_Para_Pitch.integrator = Pid_Para_Pitch.Neg_Max;       //积分项限幅

	Pid_Output +=  Pid_Para_Pitch.integrator;

	Pid_Out.pid_pitch_pwm = Pid_Output;

}


void PID_YALL()
{
	float Pid_Output = 0;
	float Pid_Error;
	Pid_Error = Att_Angle.yall - 45 - Pwm_Para.ZdirAngle;  //减去平衡点零点误差 以及倾斜飞行角度
//	if(abs(Pid_Error) > 180)  //确定是否为最优旋转方向
//	{
//		if(Pid_Error < 0)
//		{
//			Pid_Error = abs(Pid_Error)%360;
//		}
//		else
//		{
//			Pid_Error = -abs(Pid_Error)%360;
//		}
//	}

	//计算P项
	Pid_Output += Pid_Para_Yall.In_KP*Pid_Error;

	//计算D项
	Pid_Para_Yall.derivative = -Sensor_Dat.Gyro_Z/10;			//(Pid_Error - Pid_LastError) / DELTA_TIME;  //△T等于中断时间
	Pid_Output += Pid_Para_Yall.In_KD*Pid_Para_Yall.derivative;
	Pid_Para_Yall.Pid_LastError = Pid_Error;

	//计算I项
	Pid_Para_Yall.integrator += Pid_Para_Yall.In_KI*Pid_Error*DELTA_TIME;

	if(Pid_Para_Yall.integrator > Pid_Para_Yall.Pos_Max)
		Pid_Para_Yall.integrator = Pid_Para_Yall.Pos_Max;
	else if(Pid_Para_Yall.integrator < Pid_Para_Yall.Neg_Max)
		Pid_Para_Yall.integrator = Pid_Para_Yall.Neg_Max;       //积分项限幅

	Pid_Output +=  Pid_Para_Yall.integrator;

	Pid_Out.pid_yall_pwm = Pid_Output;

}

void PWM_CALCULATE()
{

	Pwm_Para.Thro = Pwm_Para.CtlThro;    //油门PWM的映射         PWM被映射到0-1000  15625是最大PWM的计数周期

	Pwm_Para.motor_front_pwm = Pwm_Para.Thro - Pid_Out.pid_pitch_pwm - Pid_Out.pid_yall_pwm;
	Pwm_Para.motor_back_pwm = Pwm_Para.Thro + Pid_Out.pid_pitch_pwm - Pid_Out.pid_yall_pwm + Pitch_Moto_Offset;
	Pwm_Para.motor_left_pwm =Pwm_Para.Thro - Pid_Out.pid_roll_pwm + Pid_Out.pid_yall_pwm + Roll_Moto_Offset;
	Pwm_Para.motor_right_pwm=Pwm_Para.Thro + Pid_Out.pid_roll_pwm + Pid_Out.pid_yall_pwm;            //电机偏置 每架飞机得自己调

	if(Pwm_Para.motor_front_pwm > 1000)
		Pwm_Para.motor_front_pwm = 1000;
	else if(Pwm_Para.motor_front_pwm < 0)
		Pwm_Para.motor_front_pwm = 0;

	if(Pwm_Para.motor_back_pwm > 1000)
		Pwm_Para.motor_back_pwm = 1000;
	else if(Pwm_Para.motor_back_pwm < 0)
		Pwm_Para.motor_back_pwm = 0;

	if(Pwm_Para.motor_left_pwm > 1000)
		Pwm_Para.motor_left_pwm = 1000;
	else if(Pwm_Para.motor_left_pwm < 0)
		Pwm_Para.motor_left_pwm = 0;

	if(Pwm_Para.motor_right_pwm > 1000)
		Pwm_Para.motor_right_pwm = 1000;
	else if(Pwm_Para.motor_right_pwm < 0)
		Pwm_Para.motor_right_pwm = 0;            //不能超过最大占空比



	if(!Sta_Ctl_Flag)  //如果收不到控制信息，停飞
	{
		Pwm_Para.motor_front_pwm =0;
		Pwm_Para.motor_back_pwm =0;
		Pwm_Para.motor_left_pwm =0;
		Pwm_Para.motor_right_pwm =0;
	}
}

void PWM_SET()
{
		TA0CCR1 = Pwm_Para.motor_front_pwm*12.5;//motor_front_pwm;  3125为TCCR0的十分之一
		TA0CCR2 = Pwm_Para.motor_back_pwm*12.5;//motor_left_pwm;
		TA0CCR3 = Pwm_Para.motor_left_pwm*12.5;//motor_right_pwm;
		TA0CCR4 = Pwm_Para.motor_right_pwm*12.5;//motor_back_pwm;
}

/**********************************MPU6050*************************************/
void MPU6050_INIT()
{
	mpu_init();
	mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);//#(INV_XYZ_GYRO | INV_XYZ_ACCEL)在inv_mpu.h定义
	mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	mpu_set_sample_rate(DEFAULT_MPU_HZ);//#本文件开头定义
	dmp_load_motion_driver_firmware();
	dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//#本文定义
	dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |
	DMP_FEATURE_SEND_CAL_GYRO |DMP_FEATURE_GYRO_CAL;//#全部在(inv_mpu_dmp_motion_driver.h)中定义
	dmp_enable_feature(dmp_features);
	dmp_set_fifo_rate(DEFAULT_MPU_HZ);
	run_self_test();
	mpu_set_dmp_state(1);
	ISendChar(0xd0,0x37,0x02); //设置为BYPASS模式
}

/******************************************三角度计算*******************************/
/******************************************三角度计算*******************************/
/******************************************三角度计算*******************************/
void ANGLE_CALCULATE()
{
    w = (float)quad[0]/1073741824.0f ;
    x = (float)quad[1]/1073741824.0f ;
    y = (float)quad[2]/1073741824.0f ;
    z = (float)quad[3]/1073741824.0f ;
    temp1 = 2*y*x + 2*w*z;
    temp2 = w*w + x*x - y*y -z*z;

    Att_Angle.yall = 57.3*atan2(temp1,temp2);  // - Yall_Angle_Offset; //偏航
//    if(Att_Angle.yall<0)
//    	Att_Angle.yall = Att_Angle.yall + 360;
//    Att_Angle.yall =(unsigned int)(Att_Angle.yall*100)%36000;
//    Att_Angle.yall /= 100.0;
    temp3 = 2*x*z - 2*w*y;
    Att_Angle.pitch = 57.3*asin(temp3);   //俯仰

    temp4 = 2*y*z + 2*w*x;
    temp5 = -2*x*x - 2*y*y + 1;
    Att_Angle.roll = 57.3*atan2(temp4,temp5);  //翻滚

    Sensor_Dat.Acc_X = accel[0];  //缩放100倍
    Sensor_Dat.Acc_Y = accel[1];
    Sensor_Dat.Acc_Z = accel[2];

    Sensor_Dat.Gyro_X = gyro[0];
    Sensor_Dat.Gyro_Y = gyro[1];
    Sensor_Dat.Gyro_Z = gyro[2];

}

/********************************************************************************/

/*********************************************TIME1中断计时*************************/
void TIMEA1_INIT()
{

	TA1CTL |= TASSEL_2;  //用SCLK 1562500hz
	TA1CCTL1 |= CCIE;    //TA1CCR1中断打开
	TA1CCR0 = 1562500/300;     //增计数上限    3ms定时  1562500/333
	TA1CCR1 = 2000;      //随意设置小雨15625/2即可
	TA1CTL |= MC_1;      //增计数模式 开中断
	_EINT();
}

void TIMEA2_INIT()
{

	TA2CTL |= TASSEL_1;  //用ACLK 25000000
	TA2CCR0 = 0;     //
	TA2CTL |= MC_0;      //关闭计数
}

void HC_RS04_Init()
{
	P2DIR |= BIT0;       //trig 发送端口
	P2OUT &=~ BIT0;      //发送端口拉低等待高电平

	P2DIR &=~ BIT2;     //ECHO  接收端口
	P2IFG = 0x00;      //清楚p2所有中断标志位
	P2IES |= BIT2;     //P2.2上升沿触发
	P2IE |= BIT2;       //P2.2中断打开
	P2IN &=~ BIT2;     //输入拉低，等会外部高电平将其拉高产生上升沿
	P2OUT |= BIT2;
	P2REN |= BIT2;     //以上两句用来设置输入的上拉电阻
	_EINT();
}
void HC_RS04_Send()
{
	P2OUT |= BIT0;  //TRIG 发送高电平
	delay_us(10);
	P2OUT &=~ BIT0;
	TA2CTL |= MC_2;      //持续计数模式 CCR0计数到0XFFFF 然后复位
}
/*
CPU分配 NMS时间轮 进行控制，其余时间为串口的中断和发送。
*/

void main(void)
{
	int_clk();         //时钟初始化
	P2DIR|=BIT4+BIT5;  //陀螺仪I2C端口初始化
	MPU6050_INIT();
	Bluth_UART_Init();	//蓝牙串口
	while(!Pid_Set_OK);//等待设置PID

	PWM_INIT();    //四路PWM初始化
	TIMEA1_INIT(); //PID控制5MS中断初始化  中断初始化必须在 ESP模块后面 因为代码中嵌套了对串口中断的使用
	HMC5583_Init();
    P7DIR |= BIT4;
	while(1)
	{
		dmp_read_fifo(gyro, accel, quad, &sensor_timestamp, &sensors, &more);  //读取角速度角速度四元数  经过测试发现该函数执行的时间竟然是不确定的，不适合放在中断，
		if(Hcm_Cal_Flag)
		{
			Get_Angle();
			Hcm_Cal_Flag = 0;
		}
	}
}
#pragma vector = PORT2_VECTOR     //固定格式，声明中断向量地址
__interrupt void HC_SR06_ISR(void)
{
	P2IFG = 0x00;
	TA2CTL |= MC_0;      //持续计数模式 CCR0计数到0XFFFF 然后复位

}

#pragma vector = TIMER1_A1_VECTOR  //定时器中断3ms定时  以为时间轮poll所有功能
__interrupt void PID_CONTROL()
{


	TA1CCTL1 &=~ CCIFG;
	switch(CpuTimePoll)
	{
	//第一个3.33ms给控制  每6.66ms控制一次
	case 0 :
		ANGLE_CALCULATE(); //四元数转欧拉角 这个执行了1.2MS左右
		PID_ROLL();           //间接计算PID占空比    PID时间 不长
		PID_PITCH();
		PID_YALL();
		PWM_CALCULATE();  //PID输出值转换为四路电机的PWM占空比
		PWM_SET();         //PWM占空比设置
		//以上执行2.2ms
		break;

	//第二给3.33ms给接收控制信息  发送周期为3.33ms其中包
	case 1 :
		Hcm_Cal_Flag = 1;  //打开磁力计
		if(time1 % 4 == 0)
		{
			Data_Send_PID1();
		}
		else if(time1 % 4 == 1)
		{
			Data_Send_MotoPWM(); //设置电机

		}
		else if(time1 % 4 == 2)
		{
			Data_Send_Status();
		}
		else if(time1 % 4 == 3)
		{
			Data_Send_Senser();
		}
		if(time1++ == 3)
			time1 = 0;
		break;
	}

	if(CpuTimePoll++ == 1)
		CpuTimePoll = 0;
}



#pragma vector = USCI_A0_VECTOR  //蓝牙发送接受中断（UART0）
__interrupt void Dat_SendReceive()
{

	temp_buf = UCA0RXBUF;  //读BUF不可清除IFG 要主动清除0
	if(temp_buf == 0xaa)  //判断帧头  PID设置帧
	{
		Pid_Rx_Flag = 1;
	}
	if(Pid_Rx_Flag)
	{
		if(Pid_Rx_Count == 3)  //第四个数据是数据的长度
		{
			Dat_num = temp_buf;
		}
		DatRx[Pid_Rx_Count++] = temp_buf;
		if(Pid_Rx_Count == Dat_num + 4 + 1) //接受了四位帧头 Len位数据 1位Sum
		{
			Data_Receive_Anl(DatRx,Pid_Rx_Count);  //发送数据以及数据总长度
			Pid_Rx_Count = 0;
			Pid_Rx_Flag = 0;
		}
	}


	if(temp_buf == 0xcc)  //控制帧
	{
		Ctl_Rx_Flag = 1;
		TA1CCTL1 &=~ CCIE;
//		Uart_Work_Flag = 1;      //告诉单片串口仍旧正常工作并且能接收到控制帧，收到上位机的控制信息
	}
	if(Ctl_Rx_Flag)
	{
		DatRx[Ctl_Rx_Count++] = temp_buf;
		if(Ctl_Rx_Count == 14)
		{
			TA1CCTL1 |= CCIE;
			Data_Receive_Ctl(DatRx,14);
			Sta_Ctl_Flag = DatRx[13];
			Ctl_Rx_Count = 0;
			Ctl_Rx_Flag = 0;
		}
	}
	UCA0IFG &=~ UCRXIFG;  //清除BUF

}


/*******************************

*******************************/
/* These next two functions converts the orienta
 * tion matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */

static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

/*******************************

*******************************/
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;




    return scalar;
}

/*******************************

*******************************/
static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }


    /* Report results. */  //#报告结果 改为N5110
    //test_packet[0] = 't';
    //test_packet[1] = result;
//    N5110_Set_XY(0,0);//#
//    N5110_Write_Char('T');
//    N5110_Write_Char(':');
//    N5110_Write_Char(result+48);
    //send_packet(PACKET_TYPE_MISC, test_packet);
}

/*******************************

*******************************/




