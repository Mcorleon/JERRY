#include "fuzzy_pid.h"
#include "include.h"
const int16 EMAXMIN[8][2] = //输入误差最大最小值
{
  //   1         2          3          4          5          6          7          8 
 // {70,-70},{70,-70},{70,-70},{70,-70},{70,-70},{70,-70},{70,-70},{70,-70}
 {130,-130},{130,-130},{130,-130},{130,-130},{130,-130},{130,-130},{130,-130},{130,-130}
  //{90,-90},{90,-90},{90,-90},{90,-90},{90,-90},{90,-90},{90,-90},{90,-90}
  //{80,-80},{80,-80},{80,-80},{80,-80},{80,-80},{80,-80},{80,-80},{80,-80}
};
uint8 PEC = 0;
uint8 PE = 0;
const int16 ECMAXMIN[8][2] = //输入误差变化率最大最小值
{
  //   1         2          3          4          5          6          7          8 
  {65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65}
  //{40,-40},{40,-40},{40,-40},{40,-40},{40,-40},{40,-40},{40,-40},{40,-40}
  //{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50}
 // {30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30}
};
const int16 OME_ERRMAX =  10;
const int16 OME_ERRMIN = -10;
const int16 SMAX = 50; // 两个参数都统一到 -50 ～ 50的值
const int16 SMIN = -50;
const int16 FMAX = 50;//语言值的满幅值
                   /*NB,  NM,  NS, ZO, PS, PM, PB*/
		   /* 0,   1,   2,  3,  4,  5,  6*/ 
const int8 EF[7]  = {-45, -30, -15, 0, 15, 30, 45};//误差,用于比较确定隶属度
const int8 ECF[7] = {-45, -30, -15, 0, 15, 30, 45};//误差变化率

//输出P值表

int16 UP1[7] = {700, 680, 550, 450, 550, 680, 700}; //输出值的100倍，计算得到的值需除以100
int16 UP2[7] = {1200, 1000, 800, 600, 800, 1000, 1200}; 
int16 UP3[7] = {1000, 900, 800, 700, 800,900, 1000};
int16 UP4[7] = {1150, 1000, 850, 700, 850,1000, 1150};
//输出D值表
int16 UD1[7] = {2000, 1700, 1450, 1200, 1450, 1700, 2000};//输出值的100倍，计算得到的值需除以100
int16 UD2[7] = {240, 160, 80, 0, 80, 160, 240};//输出值的100倍，计算得到的值需除以100
int16 UD3[7] = {200, 190, 140, 100, 140, 190, 200};//输出值的100倍，计算得到的值需除以100


//建立档位数组指针
//int16 *UP = UP1;
//int16 *UD = UD1;

int16 *UP;

int16 *UD;
//建立Kp的模糊规则表
int8 ruleP[7][7] = 
{
   /*EC */                  //  NB, NM, NS, ZO, PS, PM, PB    E
  /* NB */                   {  6,  6,  5,  5,  4,  3,  3,} , //U语言数组里面的下标
  /* NM */                   {  6,  6,  5,  4,  4,  3,  2,} ,      
  /* NS */                   {  5,  5,  5,  4,  3,  2,  2,} ,
  /* ZO */                   {  5,  5,  4,  3,  2,  1,  1,} ,
  /* PS */                   {  4,  4,  3,  2,  2,  1,  1,} ,
  /* PM */                   {  4,  3,  2,  1,  1,  1,  0,} , 
  /* PB */                   {  3,  3,  1,  1,  1,  0,  0,}
};
//建立Kd的模糊规则表
int8 ruleD[7][7] = 
{                               
   /*OM */                  //  NB, NM, NS, ZO, PS, PM, PB    E
  /* NB */                   {  0,  0,  1,  1,  2,  3,  3,} , //U语言数组里面的下标
  /* NM */                   {  0,  0,  1,  2,  2,  3,  3,} ,      
  /* NS */                   {  0,  1,  2,  2,  3,  4,  4,} ,
  /* ZO */                   {  1,  1,  2,  3,  4,  5,  5,} ,
  /* PS */                   {  1,  2,  3,  4,  4,  5,  6,} ,
  /* PM */                   {  3,  3,  4,  4,  5,  6,  6,} , 
  /* PB */                   {  3,  3,  4,  5,  5,  6,  6,}
};
float Fuzzy_Direction_P(float E,float EC)
{
	uint8 En = 0,ECn = 0;//规则表中的区域值
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};//输入及输出的隶属度及其补集
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAXMIN[PE][0]) E = EMAXMIN[PE][0];
	else if(E<EMAXMIN[PE][1]) E=EMAXMIN[PE][1];
	E = (E - EMAXMIN[PE][1])/(EMAXMIN[PE][0] - EMAXMIN[PE][1])*(SMAX - SMIN) + SMIN;//归一化
	
	if(EC>ECMAXMIN[PEC][0]) EC = ECMAXMIN[PEC][0];
	else if(EC<ECMAXMIN[PEC][1]) EC=ECMAXMIN[PEC][1];
	EC = (EC - ECMAXMIN[PEC][1])/(ECMAXMIN[PEC][0] - ECMAXMIN[PEC][1])*(SMAX - SMIN) + SMIN;//归一化	
	
	/*确定隶属度*/
	if(E>EF[0]&&E<EF[6])
	{
		if(E<=EF[1])
		{
			En = 1;
			//EL[0] = (uint16)(FMAX*(EF[1] - E)/(EF[1]-EF[0]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+45)*(E+45)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+30)*(E+30)));
                        
                }
		else if(E<=EF[2])
		{
			En = 2;
			//EL[0] = (uint16)(FMAX*(EF[2] - E)/(EF[2]-EF[1]));
	                EL[0] = (uint16)(FMAX*exp(-0.03*(E+30)*(E+30)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+15)*(E+15)));
                        
                }
		else if(E<=EF[3])
		{
			En = 3;
			//EL[0] = (uint16)(FMAX*(EF[3] - E)/(EF[3]-EF[2]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+15)*(E+15)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+0)*(E+0)));
                        
                }
		else if(E<=EF[4])
		{
			En = 4;
		//	EL[0] = (uint16)(FMAX*(EF[4] - E)/(EF[4]-EF[3]));
                        EL[0] = (uint16)(FMAX*exp(-0.03*(E)*(E)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-15)*(E-15)));
                        
		}
		else if(E<=EF[5])
		{
			En = 5;
		//	EL[0] = (uint16)(FMAX*(EF[5] - E)/(EF[5]-EF[4]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E-15)*(E-15)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-30)*(E-30)));
                        
                }
		else if(E<=EF[6])
		{
			En = 6;
			//EL[0] = (uint16)(FMAX*(EF[6] - E)/(EF[6]-EF[5]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E-30)*(E-30)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-45)*(E-45)));
                        
                        
                }
	}
	else if(E<=EF[0])
	{
		En = 1;
		EL[0] = FMAX;
                EL[1] = 0;  //
	}
	else if(E>=EF[6])
	{
		En = 6;
		EL[0] = 0;
                EL[1] = FMAX;   //
	}
	//EL[1] = FMAX - EL[0];
	
	if(EC>ECF[0]&&EC<ECF[6])
	{
		if(EC<=ECF[1])
		{
			ECn = 1;
			//ECL[0] = (uint16)(FMAX*(ECF[1] - EC)/(ECF[1]-ECF[0]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+45)*(EC+45)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+30)*(EC+30)));
                        
                }
		else if(EC<=ECF[2])
		{
			ECn = 2;
			//ECL[0] = (uint16)(FMAX*(ECF[2] - EC)/(ECF[2]-ECF[1]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+30)*(EC+30)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+15)*(EC+15)));
                       
                }
		else if(EC<=ECF[3])
		{
			ECn = 3;
			//ECL[0] = (uint16)(FMAX*(ECF[3] - EC)/(ECF[3]-ECF[2]));
	                ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+15)*(EC+15)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+0)*(EC+0)));
                       
                }
		else if(EC<=ECF[4])
		{
			ECn = 4;
			//ECL[0] = (uint16)(FMAX*(ECF[4] - EC)/(ECF[4]-ECF[3]));
                        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+0)*(EC+0)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-15)*(EC-15)));
		}
		else if(EC<=ECF[5])
		{
			ECn = 5;
			//ECL[0] = (uint16)(FMAX*(ECF[5] - EC)/(ECF[5]-ECF[4]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC-15)*(EC-15)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-30)*(EC-30)));
                       
                }
		else if(EC<=ECF[6])
		{
			ECn = 6;
			//ECL[0] = (uint16)(FMAX*(ECF[6] - EC)/(ECF[6]-ECF[5]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC-30)*(EC-30)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-45)*(EC-45)));
                       
                }
	}
	else if(EC<=ECF[0])
	{
		ECn = 1;
		ECL[0] = FMAX;
                ECL[1] = 0;
	}
	else if(EC>=ECF[6])
	{
		ECn = 6;
		ECL[0] =  0;
                ECL[1] =  FMAX;
	}
//	ECL[1] = FMAX - ECL[0];
	
       /*使用误差范围优化后的规则表rule[7][7]*/
      /*输出值使用13个隶属函数,中心值由UFF[7]指定*/ 
      /*一般都是四个规则有效*/	
	Un[0] = ruleP[En - 1][ECn - 1];
	Un[1] = ruleP[En][ECn - 1];
	Un[2] = ruleP[En - 1][ECn];
	Un[3] = ruleP[En][ECn];  
	//计算R表，隶属函数求交集?
	UL[0] = MIN(EL[0],ECL[0]);
	UL[1] = MIN(EL[1],ECL[0]);
	UL[2] = MIN(EL[0],ECL[1]);
	UL[3] = MIN(EL[1],ECL[1]);
	/*同隶属函数输出语言值求大*/
	if(Un[0] == Un[1])
	{
		if(UL[0]>UL[1])
			UL[1] = 0;
		else 
			UL[0] = 0;
	}
	if(Un[0] == Un[2])
	{
		if(UL[0]>UL[2])
			UL[2] = 0;
		else 
			UL[0] = 0;		
	}
	if(Un[0] == Un[3])
	{
		if(UL[0]>UL[3])
			UL[3] = 0;
		else 
			UL[0] = 0;		
	}
	if(Un[1] == Un[2])
	{
		if(UL[1]>UL[2])
			UL[2] = 0;
		else 
			UL[1] = 0;		
	}
        if(Un[1] == Un[3])
	{
		if(UL[1]>UL[3])
			UL[3] = 0;
		else 
			UL[1] = 0;		
	}
	if(Un[2] == Un[3])
	{
		if(UL[2]>UL[3])
			UL[3]  = 0;
		else 
			UL[2] = 0;		
	}
	/*重心法(按隶属度加权平均)反模糊*/
    /*Un[]原值为输出隶属函数标号，转换为隶属函数值*/
	Un[0] = UP[Un[0]];
	Un[1] = UP[Un[1]];
	Un[2] = UP[Un[2]];
	Un[3] = UP[Un[3]];
	
        temp1 = UL[0] * Un[0] + UL[1] * Un[1] + UL[2] * Un[2] + UL[3] * Un[3];
	temp2 = UL[0] + UL[1] + UL[2] + UL[3];
	return temp1 / temp2;
}
float Fuzzy_Direction_D(float E,float EC)
{
	uint8 En = 0,ECn = 0;//规则表中的区域值
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};//输入及输出的隶属度及其补集
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAXMIN[PE][0]) E = EMAXMIN[PE][0];
	else if(E<EMAXMIN[PE][1]) E=EMAXMIN[PE][1];
	E = (E - EMAXMIN[PE][1])/(EMAXMIN[PE][0] - EMAXMIN[PE][1])*(SMAX - SMIN) + SMIN;//归一化
	
	if(EC>OME_ERRMAX) EC = OME_ERRMAX;
	else if(EC<OME_ERRMIN) EC=OME_ERRMIN;
	EC = (EC - OME_ERRMIN)/(OME_ERRMAX - OME_ERRMIN)*(SMAX - SMIN) + SMIN;//归一化	
	
	/*确定隶属度*/
	if(E>EF[0]&&E<EF[6])
	{
		if(E<=EF[1])
		{
			En = 1;
			//EL[0] = (uint16)(FMAX*(EF[1] - E)/(EF[1]-EF[0]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+45)*(E+45)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+30)*(E+30)));
                }
		else if(E<=EF[2])
		{
			En = 2;
			//EL[0] = (uint16)(FMAX*(EF[2] - E)/(EF[2]-EF[1]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+30)*(E+30)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+15)*(E+15)));
                }
		else if(E<=EF[3])
		{
			En = 3;
	  //		EL[0] = (uint16)(FMAX*(EF[3] - E)/(EF[3]-EF[2]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+15)*(E+15)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E+0)*(E+0)));
                }
		else if(E<=EF[4])
		{
			En = 4;
		//	EL[0] = (uint16)(FMAX*(EF[4] - E)/(EF[4]-EF[3]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E+0)*(E+0)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-15)*(E-15)));
                }
		else if(E<=EF[5])
		{
			En = 5;
			//EL[0] = (uint16)(FMAX*(EF[5] - E)/(EF[5]-EF[4]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E-15)*(E-15)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-30)*(E-30)));
                }
		else if(E<=EF[6])
		{
			En = 6;
			//EL[0] = (uint16)(FMAX*(EF[6] - E)/(EF[6]-EF[5]));
		        EL[0] = (uint16)(FMAX*exp(-0.03*(E-30)*(E-30)));
                        EL[1] = (uint16)(FMAX*exp(-0.03*(E-45)*(E-45)));
                }
	}
	else if(E<=EF[0])
	{
		En = 1;
		EL[0] = FMAX;
                EL[1] = 0;
	}
	else if(E>=EF[6])
	{
		En = 6;
		EL[0] = 0;
                EL[1] = FMAX;
	}
//	EL[1] = FMAX - EL[0];
	
	if(EC>ECF[0]&&EC<ECF[6])
	{
		if(EC<=ECF[1])
		{
			ECn = 1;
			//ECL[0] = (uint16)(FMAX*(ECF[1] - EC)/(ECF[1]-ECF[0]));
	                ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+45)*(EC+45)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+30)*(EC+30)));
                }
		else if(EC<=ECF[2])
		{
			ECn = 2;
			//ECL[0] = (uint16)(FMAX*(ECF[2] - EC)/(ECF[2]-ECF[1]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+30)*(EC+30)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+15)*(EC+15)));
                }
		else if(EC<=ECF[3])
		{
			ECn = 3;
			//ECL[0] = (uint16)(FMAX*(ECF[3] - EC)/(ECF[3]-ECF[2]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+15)*(EC+15)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC+0)*(EC+0)));
                
                }
		else if(EC<=ECF[4])
		{
			ECn = 4;
			//ECL[0] = (uint16)(FMAX*(ECF[4] - EC)/(ECF[4]-ECF[3]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC+0)*(EC+0)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-15)*(EC-15)));
                }
		else if(EC<=ECF[5])
		{
			ECn = 5;
		   //	ECL[0] = (uint16)(FMAX*(ECF[5] - EC)/(ECF[5]-ECF[4]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC-15)*(EC-15)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-30)*(EC-30)));
                }
		else if(EC<=ECF[6])
		{
			ECn = 6;
		//	ECL[0] = (uint16)(FMAX*(ECF[6] - EC)/(ECF[6]-ECF[5]));
		        ECL[0] = (uint16)(FMAX*exp(-0.03*(EC-30)*(EC-30)));
                        ECL[1] = (uint16)(FMAX*exp(-0.03*(EC-45)*(EC-45)));
                }
	}
	else if(EC<=ECF[0])
	{
		ECn = 1;
		ECL[0] = FMAX;
                ECL[1] = 0;
	}
	else if(EC>=ECF[6])
	{
		ECn = 6;
		ECL[0] = 0;
                ECL[1] = FMAX;
 	}
//	ECL[1] = FMAX - ECL[0];
	
       /*使用误差范围优化后的规则表rule[7][7]*/
      /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
      /*一般都是四个规则有效*/	
	Un[0] = ruleD[En - 1][ECn - 1];
	Un[1] = ruleD[En][ECn - 1];
	Un[2] = ruleD[En - 1][ECn];
	Un[3] = ruleD[En][ECn];  
	//计算R表，隶属函数求交集
	UL[0] = MIN(EL[0],ECL[0]);
	UL[1] = MIN(EL[1],ECL[0]);
	UL[2] = MIN(EL[0],ECL[1]);
	UL[3] = MIN(EL[1],ECL[1]);
	/*同隶属函数输出语言值求大*/
	if(Un[0] == Un[1])
	{
		if(UL[0]>UL[1])
			UL[1] = 0;
		else 
			UL[0] = 0;
	}
	if(Un[0] == Un[2])
	{
		if(UL[0]>UL[2])
			UL[2] = 0;
		else 
			UL[0] = 0;		
	}
	if(Un[0] == Un[3])
	{
		if(UL[0]>UL[3])
			UL[3] = 0;
		else 
			UL[0] = 0;		
	}
	if(Un[1] == Un[2])
	{
		if(UL[1]>UL[2])
			UL[2] = 0;
		else 
			UL[1] = 0;		
	}
        if(Un[1] == Un[3])
	{
		if(UL[1]>UL[3])
			UL[3] = 0;
		else 
			UL[1] = 0;		
	}
	if(Un[2] == Un[3])
	{
		if(UL[2]>UL[3])
			UL[3]  = 0;
		else 
			UL[2] = 0;		
	}
	/*重心法(按隶属度加权平均)反模糊*/
    /*Un[]原值为输出隶属函数标号，转换为隶属函数值*/
	Un[0] = UD[Un[0]];
	Un[1] = UD[Un[1]];
	Un[2] = UD[Un[2]];
	Un[3] = UD[Un[3]];
	
        temp1 = UL[0] * Un[0] + UL[1] * Un[1] + UL[2] * Un[2] + UL[3] * Un[3];
	temp2 = UL[0] + UL[1] + UL[2] + UL[3];
	return temp1 / temp2;
}