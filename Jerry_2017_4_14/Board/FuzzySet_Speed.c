/********************************�������**********************************
*
*����   �� ɳ���Ѿ�
*�ļ��� �� FuzzySet_Speed.c
*����   �� ģ���㷨�趨�ٶ�C�ļ�
*ʱ��   �� 2016/4/14
˵��    ���ο�FreeCarsģ����������  ʹ��ɽ��V5.3��
*
****************************************************************************/
#include "FuzzySet_Speed.h"
#include "include.h"

//PMAX = 420 
//PMIN = 120 
//DMAX = 580

const int16 PMAX = 120 ;       //����440��ֵ�������� 440
const int16 PMIN = 0 ;       //С��PMIN��ֵ��������      ����Խ�򵥣��ٶ�Խ�죬����Խ�Ǹ��Ӿ�Խ��
const int16 DMAX = 65;       //���ֵ��ʾͼ���DMAX�пյĶ��٣��յ�Խ�࣬�ٶ�Խ��
const int16 DMIN = 0;       //
const int16 SSMAX = 100;      //����������ͳһ��0~100��ֵ����һ����
const int16 SSMIN = 0 ; 
const int16 FFMAX = 100 ;     //����ֵ������ֵ
              // 0  1   2   3   4   5   6
int16 PFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //ͳһ�����ֵ�� ��Сֵ��
int16 DFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //

/*�����U����ֵ������    0    1    2    3    4    5    6 */
int16 UFF0[7] =     {  80, 83, 86, 90, 93, 98, 105 };   //�����������ֵΪ�ٶ�
int16 UFF1[7] =     {  85, 87, 90, 92, 95, 98, 108 };
int16 UFF2[7] =     {  90, 92, 95, 97, 100, 105, 115 };
int16 UFF3[7] =     {  90, 94, 98, 102, 108, 112, 125 };

int16 UFF4[7] =     {  90, 92, 95, 97, 100, 105, 110 };  //preSee   
int16 UFF5[7] =     {  85, 90, 98, 105, 110, 115, 130 }; 
int16 UFF6[7] =     {  230, 235, 240, 250, 265, 280, 300 };

uint16 rule[7][7] = 
{
    /*Track_complexity */  //  0,  1,  2,  3,  4,  5,  6    Prospect_See
  /* 0 */                   {  6,  5,  4,  3,  2,  1,  1,} , //U��������������±�
  /* 1 */                   {  5,  5,  4,  3,  2,  1,  1,} ,      
  /* 2 */                   {  4,  4,  3,  2,  1,  1,  0,} ,
  /* 3 */                   {  3,  3,  3,  2,  1,  1,  0,} ,
  /* 4 */                   {  2,  2,  2,  1,  1,  0,  0,} ,
  /* 5 */                   {  2,  1,  1,  0,  0,  0,  0,} , 
  /* 6 */                   {  1,  1,  0,  0,  0,  0,  0,}
};

int16 *UFF;  //��������ѡ���õ�U��������
/*  P �����������ӳ̶�
D ��ʾǰհ
ģ�������㷨ͨ���������������Լ�����������ǰӦ��������ٶ�
*/
int16 FuzzySet_Speed(int16 P, int16 D)          //ģ���������棬�����ٶ�ֵ 
{
   int16 U;       /*ƫ��Լ����ֵ�ľ�ȷ�� */
   uint16 PF[2];
   uint16 DF[2]; 
   uint16 UF[4];  /*ƫ�ƫ��΢���Լ����ֵ��������PF[1]��P�������ȣ�PF[0]�������ȵĲ��� */
   int16 Pn = 0, Dn = 0;
   int16 Un[4];
   int32 temp1,temp2;
   
   if(P < PMIN)
     P = PMIN;
   else
     if(P > PMAX)
       P = PMAX;
   
   P = (int16)((double)(P - PMIN) / (PMAX - PMIN) * (SSMAX - SSMIN) + SSMIN); //��һ����SMIN ~ SMAX
   
   if(D < DMIN)
     D = DMIN;
   else 
     if( D > DMAX)
       D = DMAX;
    
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
          Dn = 1; DF[0] = (uint16)(FFMAX * ((float)(DFF[1] - D) / (DFF[1] - DFF[0])));
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
      Un[0] = rule[Pn - 1][ Dn - 1];
      Un[1] = rule[Pn][ Dn - 1];
      Un[2] = rule[Pn - 1][ Dn];
      Un[3] = rule[Pn][ Dn];
      if (PF[0] <= DF[0])//����R�����������󽻼�
        UF[0] = PF[0];
      else
        UF[0] = DF[0];
      if (PF[1] <= DF[0])
        UF[1] = PF[1];
      else
        UF[1] = DF[0];
      if (PF[0] <= DF[1])
        UF[2] = PF[0];
      else
        UF[2] = DF[1];
      if (PF[1] <= DF[1])
        UF[3] = PF[1];
      else
        UF[3] = DF[1];
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
      // else
      //     Un[0] = (int16)(-UFF[-Un[0]]);
      // if (Un[1] >= 0)
      Un[1] = UFF[Un[1]];
      //  else
      //    Un[1] = (int16)(-UFF[-Un[1]]);
      // if (Un[2] >= 0)
      Un[2] = UFF[Un[2]];
      //  else
      //     Un[2] = (int16)(-UFF[-Un[2]]);
      // if (Un[3] >= 0)
      Un[3] = UFF[Un[3]];
      // else
      //     Un[3] = (int16)(-UFF[-Un[3]]);
      
      temp1 = UF[0] * Un[0] + UF[1] * Un[1] + UF[2] * Un[2] + UF[3] * Un[3];
      temp2 = UF[0] + UF[1] + UF[2] + UF[3];
      U = (int16)(temp1 / temp2);
      return U;
}
   







