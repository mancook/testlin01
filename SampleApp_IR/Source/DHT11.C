#include <ioCC2530.h>
#include <OnBoard.h>
#define uint unsigned int
#define uchar unsigned char
#include <SampleApp.h>

#define wenshi P0_6

//温湿度定义
st_uint8 ucharFLAG,uchartemp;
st_uint8 g_st_shidu_shi,g_st_shidu_ge,g_st_wendu_shi,g_st_wendu_ge;
st_uint8 ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
st_uint8 ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
st_uint8 ucharcomdata;
st_uint8 g_st_shidu_xiaoshu,g_st_wendu_xiaoshu;
st_uint16 shidu_temp,wendu_temp;


void Delay_us(void);
void Delay_10us(void);
void Delay_ms(uint Time);
void COM(void);
void st_temperature(void);

uchar temp[2]={0,0}; 
uchar temp1[5]="temp=";
uchar humidity[2]={0,0};
uchar humidity1[9]="humidity=";

/****************************
        延时函数
*****************************/
void Delay_us(void) //1 us延时
{
  MicroWait(1); 
}

void Delay_10us(void) //10 us延时
{
  MicroWait(10);  
}

void Delay_ms(uint Time)//n ms延时
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}


/***********************
   温湿度传感
***********************/
void COM(void)	// 温湿写入
{     
    uchar i;         
    for(i=0;i<8;i++)    
    {
     ucharFLAG=2; 
     while((!wenshi)&&ucharFLAG++);
     Delay_10us();
     Delay_10us();
     Delay_10us();
     uchartemp=0;
     if(wenshi)uchartemp=1;
     ucharFLAG=2;
     while((wenshi)&&ucharFLAG++);   
     if(ucharFLAG==1)break;    
     ucharcomdata<<=1;
     ucharcomdata|=uchartemp; 
     }    
}

void st_temperature(void)   //温湿传感启动
{
    wenshi=0;
    Delay_ms(19);  //>18MS
    wenshi=1; 
    P0DIR &= ~0x40; //重新配置IO口方向
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
     if(!wenshi) 
     {
      ucharFLAG=2; 
      while((!wenshi)&&ucharFLAG++);
      ucharFLAG=2;
      while((wenshi)&&ucharFLAG++); 
      COM();
      ucharRH_data_H_temp=ucharcomdata;
      COM();
      ucharRH_data_L_temp=ucharcomdata;
      COM();
      ucharT_data_H_temp=ucharcomdata;
      COM();
      ucharT_data_L_temp=ucharcomdata;
      COM();
      ucharcheckdata_temp=ucharcomdata;
      wenshi=1; 
      uchartemp=(ucharT_data_H_temp+ucharT_data_L_temp+ucharRH_data_H_temp+ucharRH_data_L_temp);
       if(uchartemp==ucharcheckdata_temp)
      {
          ucharRH_data_H=ucharRH_data_H_temp;
          ucharRH_data_L=ucharRH_data_L_temp;
          ucharT_data_H=ucharT_data_H_temp;
          ucharT_data_L=ucharT_data_L_temp;
          ucharcheckdata=ucharcheckdata_temp;
       }
         wendu_temp = ucharT_data_H*256+ucharT_data_L;
         g_st_wendu_shi = wendu_temp/100; 
         g_st_wendu_ge = (wendu_temp%100 - wendu_temp%10)/10;
         g_st_wendu_xiaoshu = wendu_temp%10;
	 
         shidu_temp = ucharRH_data_H*256+ucharRH_data_L;
         g_st_shidu_shi = shidu_temp/100; 
         g_st_shidu_ge = (shidu_temp%100 - shidu_temp%10)/10;   
         g_st_shidu_xiaoshu = shidu_temp%10;
    } 
    else //没用成功读取，返回0
    {
         g_st_wendu_shi=0; 
         g_st_wendu_ge=0;
	 
         g_st_shidu_shi=0; 
         g_st_shidu_ge=0;  
    } 
}
