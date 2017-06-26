
#include "include.h"
const int16 EMAX = 130; //����������ֵ
const int16 EMIN = 0;//������Сֵ
const int16 ECMAX = 65;//�������仯�����ֵ
const int16 ECMIN = 0; // ������Сֵ
const int16 SMAX = 100; // ����������ͳһ�� -50 �� 50��ֵ
const int16 SMIN = 0;
const int16 FMAX = 100;//����ֵ������ֵ


const int16 PMAX = 130 ;     
const int16 PMIN = 0 ;       
const int16 DMAX = 65;       
const int16 DMIN = 0;     
const int16 SSMAX = 100;     
const int16 SSMIN = 0 ; 
const int16 FFMAX = 100 ;     


                   /*NB,  NM,  NS, ZO, PS, PM, PB*/
				   /* 0,   1,   2,  3,  4,  5,  6*/ 
const int8 EF[7]  = { 0, 16, 32, 48, 64, 80, 96 };   //���,���ڱȽ�ȷ��������
const int8 ECF[7] = { 0, 16, 32, 48, 64, 80, 96 };   //���仯��

const int8 PFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //ͳһ�����ֵ�� ��Сֵ��
const int8 DFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //

//���Pֵ��
int16 UP0[7] = {570, 590, 610, 630, 640, 650, 660}; //����
int16 UP1[7] = {620, 630, 640, 650, 660, 670, 680}; 
int16 UP2[7] = {520, 540, 560, 580, 600, 625, 650};
int16 UP3[7] = {530, 560, 590, 620, 640, 660, 670};
int16 UP4[7] = {600, 620, 640, 650, 660, 670, 680};//ok

int16 UD0[7] = {1500, 1550, 1600, 1650, 1700, 1750, 1800}; 
int16 UD1[7] = {1550, 1600, 1650, 1700, 1750, 1800, 1850}; 
int16 UD2[7] = {1600, 1650, 1700, 1750, 1800, 1850, 1900}; 
int16 UD3[7] = {95, 100, 105, 110, 115, 120, 130}; 
                                                  //˫������:
//int16 UFF0[7] = {  68, 70, 73, 76, 79, 82, 105 };//���� 
//int16 UFF1[7] = {  78, 80, 83, 86, 89, 92, 112 };//�е�
//int16 UFF2[7] = {  88, 90, 93, 96, 99, 102, 120 };//�쵵
int16 UFF0[7] = {  68, 70, 73, 76, 79, 82, 100};//���� 
int16 UFF1[7] = {  70, 73, 76, 79, 82, 85  ,105};//�е�
int16 UFF2[7] = {  78, 80, 83, 86, 89, 92, 112  };//�쵵

int16 UFF3[7] = {  83, 85, 88, 91, 94, 97, 118}; 
int16 UFF4[7] = {  80, 85, 88, 91, 94, 100, 120}; 
int16 UFF5[7] = {  83, 86, 90, 93, 96, 100, 122 };////���� ·������ 
int16 UFF6[7] = {  86, 90, 94, 98, 102, 105, 122 };//ץ�������Բ����� ������������
int16 *UP = UP0;
int16 *UD = UD0;
int16 *UFF = UFF0; 

//����Kp��ģ�������
//int8 ruleD[7][7] = 
//{                               
//   /*OM */                  //  NB, NM, NS, ZO, PS, PM, PB    E
//  /* NB */                   {  0,  0,  1,  1,  2,  3,  3,} , //U��������������±�
//  /* NM */                   {  0,  0,  1,  2,  2,  3,  3,} ,      
//  /* NS */                   {  0,  1,  2,  2,  3,  4,  4,} ,
//  /* ZO */                   {  1,  1,  2,  3,  4,  5,  5,} ,
//  /* PS */                   {  1,  2,  3,  4,  4,  5,  6,} ,
//  /* PM */                   {  3,  3,  4,  4,  5,  6,  6,} , 
//  /* PB */                   {  3,  3,  4,  5,  5,  6,  6,}
//};
//����Kp��ģ�������
//int8 ruleP[7][7] = 
//{
//   /*EC */                  //  NB, NM, NS, ZO, PS, PM, PB    E
//  /* NB */                   {  6,  6,  5,  5,  4,  3,  3,} , //U��������������±�
//  /* NM */                   {  6,  6,  5,  4,  4,  3,  2,} ,      
//  /* NS */                   {  5,  5,  5,  4,  3,  2,  2,} ,
//  /* ZO */                   {  5,  5,  4,  3,  2,  1,  1,} ,
//  /* PS */                   {  4,  4,  3,  2,  2,  1,  1,} ,
//  /* PM */                   {  4,  3,  2,  1,  1,  1,  0,} , 
//  /* PB */                   {  3,  3,  1,  1,  1,  0,  0,}
//};

int8 ruleP[7][7] = 
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



//uint16 ruleSD[7][7] = 
//{
//    /*Track_complexity */  //  0,  1,  2,  3,  4,  5,  6    Prospect_See
//  /* 0 */                   {  6,  5,  4,  3,  2,  2,  1,} , //U��������������±�
//  /* 1 */                   {  5,  5,  4,  3,  2,  1,  1,} ,      
//  /* 2 */                   {  4,  4,  3,  2,  1,  1,  0,} ,
//  /* 3 */                   {  3,  3,  2,  2,  1,  0,  0,} ,
//  /* 4 */                   {  2,  2,  1,  1,  1,  0,  0,} ,
//  /* 5 */                   {  2,  1,  1,  0,  0,  0,  0,} , 
//  /* 6 */                   {  1,  1,  0,  0,  0,  0,  0,}
//};

uint16 ruleSD[7][7] = 
{
    /*Track_complexity */  //  0,  1,  2,  3,  4,  5,  6    Prospect_See
  /* 0 */                   {  6,  5,  4,  3,  2,  2,  1,} , //U��������������±�
  /* 1 */                   {  5,  5,  4,  3,  2,  1,  1,} ,      
  /* 2 */                   {  4,  4,  3,  2,  1,  1,  0,} ,
  /* 3 */                   {  3,  3,  2,  2,  1,  0,  0,} ,
  /* 4 */                   {  2,  2,  1,  1,  1,  0,  0,} ,
  /* 5 */                   {  2,  1,  1,  0,  0,  0,  0,} , 
  /* 6 */                   {  1,  1,  0,  0,  0,  0,  0,}
};

float Fuzzy_Direction_P(float E,float EC)
{
	uint8 En = 0,ECn = 0;//������е�����ֵ
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};//���뼰����������ȼ��䲹��
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAX) E = EMAX;
	else if(E<EMIN) E=EMIN;
	E = (E - EMIN)/(EMAX - EMIN)*(SMAX - SMIN) + SMIN;//��һ��
	
	if(EC>ECMAX) EC = ECMAX;
	else if(EC<ECMIN) E=ECMIN;
	EC = (EC - ECMIN)/(ECMAX - ECMIN)*(SMAX - SMIN) + SMIN;//��һ��	
	
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
	uint8 En = 0,ECn = 0;//������е�����ֵ
	int16 EL[2] = {0},ECL[2] = {0},UL[4] = {0};//���뼰����������ȼ��䲹��
	int16 Un[4];
	float temp1,temp2;
	if(E>EMAX) E = EMAX;
	else if(E<EMIN) E=EMIN;
	E = (E - EMIN)/(EMAX - EMIN)*(SMAX - SMIN) + SMIN;//��һ��
	
	if(EC>ECMAX) EC = ECMAX;
	else if(EC<ECMIN) EC=ECMIN;
	EC = (EC - ECMIN)/(ECMAX - ECMIN)*(SMAX - SMIN) + SMIN;//��һ��	
	
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


int16 FuzzySet_Speed(int16 P, int16 D)          //ģ���������棬�����ٶ�ֵ 
{
   int16 U;       /*ƫ��Լ����ֵ�ľ�ȷ�� */
   uint16 PF[2];
   uint16 DF[2]; 
   uint16 UF[4];  /*ƫ�ƫ��΢���Լ����ֵ��������PF[1]��P�������ȣ�PF[0]�������ȵĲ��� */
   int16 Pn = 0, Dn = 0;
   int16 Un[4];
   int32 temp1,temp2;
   
   if(P < PMIN)  P = PMIN;    
   else if(P > PMAX)  P = PMAX;
        
   if(D < DMIN)  D = DMIN;  
   else if( D > DMAX)  D = DMAX;
          
   P = (int16)((double)(P - PMIN) / (PMAX - PMIN) * (SSMAX - SSMIN) + SSMIN); //��һ����SMIN ~ SMAX
   D = (int16)(((double)D )/ (DMAX - DMIN) * (SSMAX - SSMIN) + SSMIN) ;    //��һ����SMIN ~ SMAX

   /*�����ȵ�ȷ��*/
   /*����PD��ָ�����Ի����Ч��������*/   
   if(P > PFF[0] && P < PFF[6])
   {
        if (P <= PFF[1])
        {
            Pn = 1;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[1] - P) / (PFF[1] - PFF[0])));
        }
        else if (P <= PFF[2])
        {
            Pn = 2;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[2] - P) / (PFF[2] - PFF[1])));
        }
        else if (P <= PFF[3])
        {
            Pn = 3;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[3] - P) / (PFF[3] - PFF[2])));
        }
        else if (P <= PFF[4])
        {
            Pn = 4;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[4] - P) / (PFF[4] - PFF[3])));
        }
        else if (P <= PFF[5])
        {
            Pn = 5;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[5] - P) / (PFF[5] - PFF[4])));
        }
        else if (P <= PFF[6])
        {
            Pn = 6;
            PF[0] = (uint16)(FFMAX * ((float)(PFF[6] - P) / (PFF[6] - PFF[5])));
        }
   }
   else if (P <= PFF[0])
   {
        Pn = 1;
        PF[0] = (uint16)(FFMAX);
    }
   else if (P >= PFF[6])
   {
        Pn = 6; 
        PF[0] = 0;
   }
   PF[1] = (uint16)(FFMAX - PF[0]);
     
     
   if (D > DFF[0] && D < DFF[6])
   {
      if (D <= DFF[1])
      {
          Dn = 1; 
          DF[0] = (uint16)(FFMAX * ((float)(DFF[1] - D) / (DFF[1] - DFF[0])));
      }
      else if (D <= DFF[2])
      {
          Dn = 2;
          DF[0] = (uint16)(FFMAX * ((float)(DFF[2] - D) / (DFF[2] - DFF[1])));
      }
      else if (D <= DFF[3])
      {
          Dn = 3;
          DF[0] = (uint16)(FFMAX * ((float)(DFF[3] - D) / (DFF[3] - DFF[2])));
      }
      else if (D <= DFF[4])
      {
          Dn = 4;
          DF[0] = (uint16)(FFMAX * ((float)(DFF[4] - D) / (DFF[4] - DFF[3])));
      }
      else if (D <= DFF[5])
      {
          Dn = 5;
          DF[0] = (uint16)(FFMAX * ((float)(DFF[5] - D) / (DFF[5] - DFF[4])));
      }
      else if (D <= DFF[6])
      {
          Dn = 6;
          DF[0] = (uint16)(FFMAX * ((float)(DFF[6] - D) / (DFF[6] - DFF[5])));
      }
   }
   else if (D <= DFF[0])
   {
      Dn = 1;
      DF[0] = (uint16)(FFMAX);
   }
   else if (D >= DFF[6])
   {
      Dn = 6;
      DF[0] = 0;
   }
   DF[1] = (uint16)(FFMAX - DF[0]);
    
   
   /*ʹ����Χ�Ż���Ĺ����rule[7][7]*/
   /*���ֵʹ��13����������,����ֵ��UFF[7]ָ��*/
   /*һ�㶼���ĸ�������Ч*/
   Un[0] = ruleSD[Pn - 1][ Dn - 1];
   Un[1] = ruleSD[Pn][ Dn - 1];
   Un[2] = ruleSD[Pn - 1][ Dn];
   Un[3] = ruleSD[Pn][ Dn];
   
   
   UF[0] = MIN(PF[0],DF[0]);
   UF[1] = MIN(PF[1],DF[0]);
   UF[2] = MIN(PF[0],DF[1]);
   UF[3] = MIN(PF[1],DF[1]);
   
   
   /*ͬ���������������ֵ���*/      
   if (Un[0] == Un[1])
   {
     if (UF[0] > UF[1])
       UF[1] = 0;
     else
       UF[0] = 0;
   }
   if (Un[0] == Un[2])
   {
     if (UF[0] > UF[2])
       UF[2] = 0;
     else
       UF[0] = 0;
   }
   if (Un[0] == Un[3])
   {
     if (UF[0] > UF[3])
       UF[3] = 0;
     else
       UF[0] = 0;
   }
   if (Un[1] == Un[2])
   {
     if (UF[1] > UF[2])
       UF[2] = 0;
     else
       UF[1] = 0;
   }
   if (Un[1] == Un[3])
   {
     if (UF[1] > UF[3])
       UF[3] = 0;
     else
       UF[1] = 0;
   }
   if (Un[2] == Un[3])
   {
     if (UF[2] > UF[3])
       UF[3] = 0;
     else
       UF[2] = 0;
   }
   
   /*���ķ�(�������ȼ�Ȩƽ��)��ģ��*/
   /*Un[]ԭֵΪ�������������ţ�ת��Ϊ��������ֵ*/
   // if (Un[0] >= 0)
   Un[0] = UFF[Un[0]];
   Un[1] = UFF[Un[1]];
   Un[2] = UFF[Un[2]];
   Un[3] = UFF[Un[3]];
   
   temp1 = UF[0] * Un[0] + UF[1] * Un[1] + UF[2] * Un[2] + UF[3] * Un[3];
   temp2 = UF[0] + UF[1] + UF[2] + UF[3];
   U = (int16)(temp1 / temp2);
   return U;
}
