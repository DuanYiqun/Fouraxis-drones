/*函数功能：数值按位正序存储到数组
将数值n各位数字分别取出，并顺序放置于数组st中，n最大为整数4位，pst为存储初始位*/
#include<math.h>
void sto_int(int n,unsigned char *st,unsigned char pst)
{
        unsigned int n_int;                   //n_int为倒序取数的临时数值
        unsigned char i=0,j=0;                   //i为存储数组的元素序号，j为倒序取数循环变量和正序取数元素序号
        int k;                                 //k为正序转换循环变量
        unsigned char symbol_n=0;    //数值符号标记
        unsigned char tp[10]={' '};      //倒序取数临时数组
        if(n < 0)                   //数值为负，标记置1，数值取绝对值
        	{
        		symbol_n=1;
        		n = -n;
        	}
        n_int=n;
        for(j=0;j<5;j++)                        //倒序取整数，最大8位，取完退出循环
        {
                //if(n_int == 0)break;
                tp[i++] ='0'+n_int%10;
                n_int = n_int/10;
        }
		if(symbol_n==1) tp[i]='-'; //符号位为1，正序数组首位为符号
		else tp[i]='+';
		//顺序转换
        for(k=j;k>=0;k--)         //整数部分正序转换
        {
          st[pst++] = tp[k];
        }   
}
