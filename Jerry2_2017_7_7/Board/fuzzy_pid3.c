#include "fuzzy_pid.h"
#include "include.h"
const int16 EMAXMIN[8][2] = //������������Сֵ
{
  //   1         2          3          4          5          6          7          8 
  {120,-120},{120,-120},{120,-120},{120,-120},{120,-120},{120,-120},{120,-120},{120,-120}
 //{150,-150},{150,-150},{150,-150},{150,-150},{150,-150},{150,-150},{150,-150},{150,-150}
  //{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50},{50,-50}
  //{180,-180},{180,-180},{180,-180},{180,-180},{180,-180},{180,-180},{180,-180},{180,-180}
  //{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200}
};
uint8 PEC = 0;
uint8 PE = 0;
const int16 ECMAXMIN[8][2] = //�������仯�������Сֵ
{
  //   1         2          3          4          5          6          7          8 
 // {25,-25},{25,-25},{25,-25},{25,-25},{25,-25},{25,-25},{25,-25},{25,-25}
  //{100,-100},{100,-100},{100,-100},{100,-100},{100,-100},{100,-100},{100,-100},{100,-100}
  //{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200},{200,-200}
  {65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65},{65,-65}
  //{85,-85},{85,-85},{85,-85},{85,-85},{85,-85},{85,-85},{85,-85},{85,-85}
  //{30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30},{30,-30}
};
const int16 OME_ERRMAX =  10;
const int16 OME_ERRMIN = -10;
const int16 SMAX = 50; // ����������ͳһ�� -50 �� 50��ֵ
const int16 SMIN = -50;
const int16 FMAX = 50;//����ֵ������ֵ
                   /*NB,  NM,  NS, ZO, PS, PM, PB*/
		   /* 0,   1,   2,  3,  4,  5,  6*/ 
const int8 EF[7]  = {-45, -30, -15, 0, 15, 30, 45};//���,���ڱȽ�ȷ��������
const int8 ECF[7] = {-45, -30, -15, 0, 15, 30, 45};//���仯��

//���Pֵ��

int16 UP1[7] = {700, 650, 520, 400, 520, 650, 700}; //���ֵ��100��������õ���ֵ�����100
int16 UP2[7] = {720, 680, 550, 400, 550, 680, 720}; 
int16 UP3[7] = {600, 580, 480, 400, 480,580, 600};
int16 UP4[7] = {600, 550, 450, 350, 450,550, 600};
int16 UP5[7] = {700, 550, 350, 150, 350,550, 700};
//���Dֵ��
int16 UD1[7] = {200, 120, 50, 0, 50, 120, 200};//���ֵ��100��������õ���ֵ�����100
int16 UD2[7] = {300, 200, 100, 0, 100, 200, 300};//���ֵ��100��������õ���ֵ�����100
int16 UD3[7] = {5500, 5000, 4500, 4000, 4500, 5000, 5500};//���ֵ��100��������õ���ֵ�����100


//������λ����ָ��
int16 *UP = UP1;
int16 *UD = UD1;
//����Kp��ģ�������
int8 ruleP[7][7] = 
{
   /*EC */                  //  NB, NM, NS, ZO, PS, PM, PB    E
  /* NB */                   {  6,  6,  5,  5,  4,  3,  3,} , //U��������������±�
  /* NM */                   {  6,  6,  5,  4,  4,  3,  2,} ,      
  /* NS */                   {  5,  5,  5,  4,  3,  2,  2,} ,
  /* ZO */                   {  5,  5,  4,  3,  2,  1,  1,} ,
  /* PS */                   {  4,  4,  3,  2,  2,  1,  1,} ,
  /* PM */                   {  4,  3,  2,  1,  1,  1,  0,} , 
  /* PB */                   {  3,  3,  1,  1,  1,  0,  0,}
};
//����Kd��ģ�������
int8 ruleD[7][7] = 
{                               
   /*OM */                  //  NB, NM, NS, ZO, PS, PM, PB    E
  /* NB */                   {  0,  0,  1,  1,  2,  3,  3,} , //U��������������±�
  /* NM */                   {  0,  0,  1,  2,  2,  3,  3,} ,      
  /* NS */                   {  0,  1,  2,  2,  3,  4,  4,} ,
  /* ZO */                   {  1,  1,  2,  3,  4,  5,  5,} ,
  /* PS */                   {  1,  2,  3,  4,  4,  5,  6,} ,
  /* PM */                   {  3,  3,  4,  4,  5,  6,  6,} , 
  /* PB */                   {  3,  3,  4,  5,  5,  6,  6,}
};
float Fuzzy_Direction_P(float E,float EC)   
{
	uint8 En = 0,ECn = 0;//������е�����ֵ
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};//���뼰����������ȼ��䲹��
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAXMIN[PE][0]) E = EMAXMIN[PE][0];//���E����E�����ֵ����E����E�����ֵ
	else if(E<EMAXMIN[PE][1]) E=EMAXMIN[PE][1];//���EС��E����Сֵ����E����E����Сֵ
	E = (E - EMAXMIN[PE][1])/(EMAXMIN[PE][0] - EMAXMIN[PE][1])*(SMAX - SMIN) + SMIN;//��һ�� E�ı仯��Χ��Ϊ-50~+50
	//ͬ��
	if(EC>ECMAXMIN[PEC][0]) EC = ECMAXMIN[PEC][0];
	else if(EC<ECMAXMIN[PEC][1]) EC=ECMAXMIN[PEC][1];
	EC = (EC - ECMAXMIN[PEC][1])/(ECMAXMIN[PEC][0] - ECMAXMIN[PEC][1])*(SMAX - SMIN) + SMIN;//��һ��	
	
	/*ȷ��������*/
	if(E>EF[0]&&E<EF[6])
	{
		if(E<=EF[1])
		{
			En = 1;
			//EL[0] = (uint16)(FMAX*(EF[1] - E)/(EF[1]-EF[0]));
                        if(E<=-37.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1+log(-E-36.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(-log(-E-36.5)/4.2802+0.5));
                        }
                        if(E>-37.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1-log(E+38.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(log(E+38.5)/4.2802+0.5));
                        
                        }
                        
                        
		}
		else if(E<=EF[2])
		{
			En = 2;
                        if(E<=-22.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1+log(-E-21.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(-log(-E-21.5)/4.2802+0.5));
                        }
                        if(E>-22.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1-log(E+23.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(log(E+23.5)/4.2802+0.5));
                        
                        }
                        //EL[0] = (uint16)(FMAX*(EF[2] - E)/(EF[2]-EF[1]));
		}
		else if(E<=EF[3])
		{
			En = 3;
			EL[0] = (uint16)(FMAX*(EF[3] - E)/(EF[3]-EF[2]));
                        EL[1] = FMAX - EL[0];
		}
		else if(E<=EF[4])
		{
			En = 4;
			EL[0] = (uint16)(FMAX*(EF[4] - E)/(EF[4]-EF[3]));
                        EL[1] = FMAX - EL[0];
		}
		else if(E<=EF[5])
		{
			En = 5;
			if(E<=22.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1+log(-E+23.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(-log(-E+23.5)/4.2802+0.5));
                        }
                        if(E>22.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1-log(E-21.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(log(E-21.5)/4.2802+0.5));
                        }
                        //EL[0] = (uint16)(FMAX*(EF[5] - E)/(EF[5]-EF[4]));
		}
		else if(E<=EF[6])
		{
			En = 6;
                        
			
			if(E<=37.5)
                        {
                        EL[0] = (uint16)(FMAX*(1+log(-E+38.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(-log(-E+38.5)/4.2802+0.5));
                        }
                        if(E>37.5)
                        {
                        EL[0]=  (uint16)(FMAX*(1-log(E-36.5)/4.2802-0.5));
                        EL[1] = (uint16)(FMAX*(log(E-36.5)/4.2802+0.5));
                        }
			//EL[0] = (uint16)(FMAX*(EF[6] - E)/(EF[6]-EF[5]));
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
	//EL[1] = FMAX - EL[0];
	
	if(EC>ECF[0]&&EC<ECF[6])
	{
		if(EC<=ECF[1])
		{
			ECn = 1;
		        if(EC<=-37.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1+log(-EC-36.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(-log(-EC-36.5)/4.2802+0.5));
                        }
                        if(EC>-37.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1-log(EC+38.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(log(EC+38.5)/4.2802+0.5));
                        
                        }
                        //ECL[0] = (uint16)(FMAX*(ECF[1] - EC)/(ECF[1]-ECF[0]));
		}
		else if(EC<=ECF[2])
		{
			ECn = 2;
			if(EC<=-22.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1+log(-EC-21.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(-log(-EC-21.5)/4.2802+0.5));
                        }
                        if(EC>-22.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1-log(EC+23.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(log(EC+23.5)/4.2802+0.5));
                        
                        }
                        //ECL[0] = (uint16)(FMAX*(ECF[2] - EC)/(ECF[2]-ECF[1]));
		}
		else if(EC<=ECF[3])
		{
			ECn = 3;
			ECL[0] = (uint16)(FMAX*(ECF[3] - EC)/(ECF[3]-ECF[2]));
                        ECL[1] = FMAX - ECL[0];
		}
		else if(EC<=ECF[4])
		{
			ECn = 4;
			ECL[0] = (uint16)(FMAX*(ECF[4] - EC)/(ECF[4]-ECF[3]));
                        ECL[1] = FMAX - ECL[0];
		}
		else if(EC<=ECF[5])
		{
			ECn = 5;
			if(EC<=22.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1+log(-EC+23.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(-log(-EC+23.5)/4.2802+0.5));
                        }
                        if(EC>22.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1-log(EC-21.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(log(EC-21.5)/4.2802+0.5));
                        }
                        //ECL[0] = (uint16)(FMAX*(ECF[5] - EC)/(ECF[5]-ECF[4]));
		}
		else if(EC<=ECF[6])
		{
			ECn = 6;
                        if(EC<=37.5)
                        {
                        ECL[0] = (uint16)(FMAX*(1+log(-EC+38.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(-log(-EC+38.5)/4.2802+0.5));
                        }
                        if(EC>37.5)
                        {
                        ECL[0]=  (uint16)(FMAX*(1-log(EC-36.5)/4.2802-0.5));
                        ECL[1] = (uint16)(FMAX*(log(EC-36.5)/4.2802+0.5));
                        }
			//ECL[0] = (uint16)(FMAX*(ECF[6] - EC)/(ECF[6]-ECF[5]));
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
	//ECL[1] = FMAX - ECL[0];
	
       /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
      /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
      /*һ�㶼���ĸ�������Ч*/	
	Un[0] = ruleP[En - 1][ECn - 1];
	Un[1] = ruleP[En][ECn - 1];
	Un[2] = ruleP[En - 1][ECn];
	Un[3] = ruleP[En][ECn];  
	//����R�����������󽻼�
	UL[0] = MIN(EL[0],ECL[0]);
	UL[1] = MIN(EL[1],ECL[0]);
	UL[2] = MIN(EL[0],ECL[1]);
	UL[3] = MIN(EL[1],ECL[1]);
	/*ͬ���������������ֵ���*/
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
	/*���ķ�(�������ȼ�Ȩƽ��)��ģ��*/
    /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
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
	uint8 En = 0,ECn = 0;
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAXMIN[PE][0]) E = EMAXMIN[PE][0];
	else if(E<EMAXMIN[PE][1]) E=EMAXMIN[PE][1];
	E = (E - EMAXMIN[PE][1])/(EMAXMIN[PE][0] - EMAXMIN[PE][1])*(SMAX - SMIN) + SMIN;//��һ��
	
	if(EC>OME_ERRMAX) EC = OME_ERRMAX;
	else if(EC<OME_ERRMIN) EC=OME_ERRMIN;
	EC = (EC - OME_ERRMIN)/(OME_ERRMAX - OME_ERRMIN)*(SMAX - SMIN) + SMIN;//��һ��	
	
	/*ȷ��������*/
	if(E>EF[0]&&E<EF[6])
	{
		if(E<=EF[1])
		{
			En = 1;
			EL[0] = (uint16)(FMAX*(EF[1] - E)/(EF[1]-EF[0]));
		}
		else if(E<=EF[2])
		{
			En = 2;
			EL[0] = (uint16)(FMAX*(EF[2] - E)/(EF[2]-EF[1]));
		}
		else if(E<=EF[3])
		{
			En = 3;
			EL[0] = (uint16)(FMAX*(EF[3] - E)/(EF[3]-EF[2]));
		}
		else if(E<=EF[4])
		{
			En = 4;
			EL[0] = (uint16)(FMAX*(EF[4] - E)/(EF[4]-EF[3]));
		}
		else if(E<=EF[5])
		{
			En = 5;
			EL[0] = (uint16)(FMAX*(EF[5] - E)/(EF[5]-EF[4]));
		}
		else if(E<=EF[6])
		{
			En = 6;
			EL[0] = (uint16)(FMAX*(EF[6] - E)/(EF[6]-EF[5]));
		}
	}
	else if(E<=EF[0])
	{
		En = 1;
		EL[0] = FMAX;
	}
	else if(E>=EF[6])
	{
		En = 6;
		EL[0] = 0;
	}
	EL[1] = FMAX - EL[0];
	
	if(EC>ECF[0]&&EC<ECF[6])
	{
		if(EC<=ECF[1])
		{
			ECn = 1;
			ECL[0] = (uint16)(FMAX*(ECF[1] - EC)/(ECF[1]-ECF[0]));
		}
		else if(EC<=ECF[2])
		{
			ECn = 2;
			ECL[0] = (uint16)(FMAX*(ECF[2] - EC)/(ECF[2]-ECF[1]));
		}
		else if(EC<=ECF[3])
		{
			ECn = 3;
			ECL[0] = (uint16)(FMAX*(ECF[3] - EC)/(ECF[3]-ECF[2]));
		}
		else if(EC<=ECF[4])
		{
			ECn = 4;
			ECL[0] = (uint16)(FMAX*(ECF[4] - EC)/(ECF[4]-ECF[3]));
		}
		else if(EC<=ECF[5])
		{
			ECn = 5;
			ECL[0] = (uint16)(FMAX*(ECF[5] - EC)/(ECF[5]-ECF[4]));
		}
		else if(EC<=ECF[6])
		{
			ECn = 6;
			ECL[0] = (uint16)(FMAX*(ECF[6] - EC)/(ECF[6]-ECF[5]));
		}
	}
	else if(EC<=ECF[0])
	{
		ECn = 1;
		ECL[0] = FMAX;
	}
	else if(EC>=ECF[6])
	{
		ECn = 6;
		ECL[0] = 0;
	}
	ECL[1] = FMAX - ECL[0];
	
       /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
      /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
      /*һ�㶼���ĸ�������Ч*/	
	Un[0] = ruleD[En - 1][ECn - 1];
	Un[1] = ruleD[En][ECn - 1];
	Un[2] = ruleD[En - 1][ECn];
	Un[3] = ruleD[En][ECn];  
	//����R�����������󽻼�
	UL[0] = MIN(EL[0],ECL[0]);
	UL[1] = MIN(EL[1],ECL[0]);
	UL[2] = MIN(EL[0],ECL[1]);
	UL[3] = MIN(EL[1],ECL[1]);
	/*ͬ���������������ֵ���*/
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
	/*���ķ�(�������ȼ�Ȩƽ��)��ģ��*/
    /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
	Un[0] = UD[Un[0]];
	Un[1] = UD[Un[1]];
	Un[2] = UD[Un[2]];
	Un[3] = UD[Un[3]];
	
        temp1 = UL[0] * Un[0] + UL[1] * Un[1] + UL[2] * Un[2] + UL[3] * Un[3];
	temp2 = UL[0] + UL[1] + UL[2] + UL[3];
	return temp1 / temp2;
}