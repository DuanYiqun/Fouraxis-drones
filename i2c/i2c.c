/*-----------------------------------------------
  名称：IIC协议 EEPROM24c02
  内容：此程序用于检测EEPROM性能，测试方法如下：写入24c02一些数据，然后在内存中清除这些数据，
        掉电后主内存将失去这些信息，然后从24c02中调入这些数据。看是否与写入的相同。		
------------------------------------------------*/  
  
              
#include<msp430f5529.h>
#include"int_clk.h"                       //精确延时函数与时钟初始化
#include"msp430_clock.h"


// 常,变量定义区
#define DATDIR P2DIR               //I2C端口方向寄存器
#define DATIN P2IN             //输入寄存器
#define DATOUT P2OUT           //输出寄存器
#define SDA BIT4            //模拟I2C数据传送位
#define SCL BIT5            //模拟I2C时钟控制位
                          
unsigned char ack;	              //应答标志位   
/*------------------------------------------------
                    启动总线
------------------------------------------------*/
void Start_I2c()
{
  DATDIR|=SDA;      //开输出
  DATDIR|=SCL;
  DATOUT|=SDA;//SDA=1;   //发送起始条件的数据信号
  delay_us(1);
  DATOUT|=SCL;//SCL=1;
  delay_us(5);    //起始条件建立时间大于4.7us,延时    
  DATOUT&=~SDA;//SDA=0;     //发送起始信号
  delay_us(5);    //起始条件锁定时间大于4μ       
  DATOUT&=~SCL;//SCL=0;    //钳住I2C总线，准备发送或接收数据
  delay_us(2);
}
/*------------------------------------------------
                    结束总线
------------------------------------------------*/
void Stop_I2c()
{
  DATOUT&=~SDA;//SDA=0;    //发送结束条件的数据信号
  delay_us(1);   //发送结束条件的时钟信号
  DATOUT|=SCL;//SCL=1;    //结束条件建立时间大于4μ
  delay_us(5);
  DATOUT|=SDA;//SDA=1;    //发送I2C总线结束信号
  delay_us(4);
}




/*----------------------------------------------------------------
                 字节数据传送函数               
函数原型: void  SendByte(unsigned char c);
功能:  将数据c发送出去,可以是地址,也可以是数据,发完后等待应答,并对
     此状态位进行操作.(不应答或非应答都使ack=0 假)     
     发送数据正常，ack=1; ack=0表示被控器无应答或损坏。
------------------------------------------------------------------*/
void  SendByte(unsigned char c)
{
 unsigned char BitCnt;
 
 for(BitCnt=0;BitCnt<8;BitCnt++)  //要传送的数据长度为8位
    {
     if((c<<BitCnt)&0x80)
       DATOUT|=SDA;//SDA=1;   //判断发送位
     else  DATOUT&=~SDA;//SDA=0;                
     delay_us(1);
     DATOUT|=SCL;//SCL=1;               //置时钟线为高，通知被控器开始接收数据位      
     delay_us(5);             //保证时钟高电平周期大于4μ         
     DATOUT&=~SCL;//SCL=0; 
    }
    delay_us(2);
    DATOUT|=SDA;//SDA=1;               //8位发送完后释放数据线，准备接收应答位      
    DATDIR&=~SDA;
    delay_us(2);
    DATOUT|=SCL;//SCL=1;
    delay_us(3);
    if(DATIN&SDA)ack=0;     
    else ack=1;        //判断是否接收到应答信号
    DATOUT&=~SCL;//SCL=0;
    delay_us(2);
    DATDIR|=SDA;        //方向寄存器常保持输出状态
}







/*----------------------------------------------------------------
                 字节数据传送函数               
函数原型: unsigned char  RcvByte();
功能:  用来接收从器件传来的数据,并判断总线错误(不发应答信号)，
     发完后请用应答函数。  
------------------------------------------------------------------*/	
unsigned char  RcvByte()
{
  unsigned char retc;
  unsigned char BitCnt;
  
  retc=0; 
  DATDIR&=~SDA;//SDA=1;             //置数据线为输入方式
  for(BitCnt=0;BitCnt<8;BitCnt++)
      {
        delay_us(1);           
        DATOUT&=~SCL;//SCL=0;       //置时钟线为低，准备接收数据位


        delay_us(5);      //时钟低电平周期大于4.7us
        DATOUT|=SCL;//SCL=1;       //置时钟线为高使数据线上数据有效
        delay_us(2);
        retc=retc<<1;
        if(DATIN&SDA)retc=retc+1; //读数据位,接收的数据位放入retc中
        delay_us(2);
      }
  DATOUT&=~SCL;//SCL=0;    
  delay_us(2);
  DATDIR|=SDA;
  return(retc);
}



/*----------------------------------------------------------------
                     应答子函数
原型:  void Ack_I2c(void);
 
----------------------------------------------------------------*/
void Ack_I2c(void)
{
  
  DATOUT&=~SDA;//SDA=0;     
  delay_us(3);     
  DATOUT|=SCL;//SCL=1;
  delay_us(5);              //时钟低电平周期大于4μ
  DATOUT&=~SCL;//SCL=0;               //清时钟线，钳住I2C总线以便继续接收
  delay_us(2);  
}
/*----------------------------------------------------------------
                     非应答子函数
原型:  void NoAck_I2c(void);
 
----------------------------------------------------------------*/
void NoAck_I2c(void)
{
  
  DATOUT|=SDA;//SDA=1;
  delay_us(3);      
  DATOUT|=SCL;//SCL=1;
  delay_us(5);              //时钟低电平周期大于4μ
  DATOUT&=~SCL;//SCL=0;                //清时钟线，钳住I2C总线以便继续接收
  delay_us(2);  
}
unsigned char ISendChar(unsigned char sla,unsigned char suba,unsigned char s)
{
   unsigned char i;
   Start_I2c();               //启动总线
   SendByte(sla);             //发送器件地址
     if(ack==0)return(0);
   SendByte(suba);            //发送器件子地址
     if(ack==0)return(0);
   SendByte(s);            //发送数据
     if(ack==0)return(0);
   Stop_I2c();                  //结束总线
   delay_us(10);
   return(1);
}

/*----------------------------------------------------------------
                    向有子地址器件发送多字节数据函数               
函数原型: bit  ISendStr(unsigned char sla,unsigned char suba,ucahr *s,unsigned char no);  
功能:     从启动总线到发送地址，子地址,数据，结束总线的全过程,从器件
          地址sla，子地址suba，发送内容是s指向的内容，发送no个字节。
           如果返回1表示操作成功，否则操作有误。
注意：    使用前必须已结束总线。
----------------------------------------------------------------*/
unsigned char ISendStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no)
{
   unsigned char i;

   Start_I2c();               //启动总线
   SendByte(sla);             //发送器件地址
     if(ack==0)return(0);
   SendByte(suba);            //发送器件子地址
     if(ack==0)return(0);
   for(i=0;i<no;i++)
    {   
     SendByte(*s);            //发送数据
     if(ack==0)return(0);
     s++;
    } 
    Stop_I2c();                  //结束总线
    delay_us(10);
    return(1);
}

/*----------------------------------------------------------------
                    向有子地址器件读取多字节数据函数               
函数原型: bit  ISendStr(unsigned char sla,unsigned char suba,ucahr *s,unsigned char no);  
功能:     从启动总线到发送地址，子地址,读数据，结束总线的全过程,从器件
          地址sla，子地址suba，读出的内容放入s指向的存储区，读no个字节。
           如果返回1表示操作成功，否则操作有误。
注意：    使用前必须已结束总线。
----------------------------------------------------------------*/
unsigned char IRcvStr(unsigned char sla,unsigned char suba,unsigned char *s,unsigned char no)
{
   unsigned char i;

   Start_I2c();               //启动总线
   SendByte(sla);             //发送器件地址
     if(ack==0)return(0);
   SendByte(suba);            //发送器件子地址
     if(ack==0)return(0);

   Start_I2c();
   SendByte(sla+1);
      if(ack==0)return(0);

   for(i=0;i<no-1;i++)
    {   
     *s=RcvByte();              //发送数据
      Ack_I2c();                //发送就答位 
     s++;
    } 
   *s=RcvByte();
    NoAck_I2c();                 //发送非应位
   Stop_I2c();                    //结束总线
  return(1);
}
