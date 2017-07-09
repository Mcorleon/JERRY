/********************************电机驱动**********************************
*
*作者   ： 沙艺已觉
*文件名 ： FuzzySet_Speed.c
*描述   ： 模糊算法设定速度C文件
*时间   ： 2016/4/14
说明    ：参考FreeCars模糊控制例程  使用山外V5.3库
*
****************************************************************************/
#include "FuzzySet_Speed.h"
#include "include.h"

//PMAX = 420 
//PMIN = 120 
//DMAX = 580

const int16 PMAX = 120 ;       //大于440的值将被当成 440
const int16 PMIN = 0 ;       //小于PMIN的值将被当成      赛道越简单，速度越快，否则越是复杂就越慢
const int16 DMAX = 65;       //这个值表示图像后DMAX行空的多少，空的越多，速度越慢
const int16 DMIN = 0;       //
const int16 SSMAX = 100;      //两个参数都统一到0~100的值（归一化）
const int16 SSMIN = 0 ; 
const int16 FFMAX = 100 ;     //语言值的满幅值
              // 0  1   2   3   4   5   6
int16 PFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //统一到最大值； 最小值；
int16 DFF[7] = { 0, 16, 32, 48, 64, 80, 96 };     //

/*输出量U语言值特征点    0    1    2    3    4    5    6 */
int16 UFF0[7] =     {  80, 83, 86, 90, 93, 98, 105 };   //数组里面的数值为速度
int16 UFF1[7] =     {  85, 87, 90, 92, 95, 98, 108 };
int16 UFF2[7] =     {  90, 92, 95, 97, 100, 105, 115 };
int16 UFF3[7] =     {  90, 94, 98, 102, 108, 112, 125 };

int16 UFF4[7] =     {  90, 92, 95, 97, 100, 105, 110 };  //preSee   
int16 UFF5[7] =     {  85, 90, 98, 105, 110, 115, 130 }; 
int16 UFF6[7] =     {  230, 235, 240, 250, 265, 280, 300 };

uint16 rule[7][7] = 
{
    /*Track_complexity */  //  0,  1,  2,  3,  4,  5,  6    Prospect_See
  /* 0 */                   {  6,  5,  4,  3,  2,  1,  1,} , //U语言数组里面的下标
  /* 1 */                   {  5,  5,  4,  3,  2,  1,  1,} ,      
  /* 2 */                   {  4,  4,  3,  2,  1,  1,  0,} ,
  /* 3 */                   {  3,  3,  3,  2,  1,  1,  0,} ,
  /* 4 */                   {  2,  2,  2,  1,  1,  0,  0,} ,
  /* 5 */                   {  2,  1,  1,  0,  0,  0,  0,} , 
  /* 6 */                   {  1,  1,  0,  0,  0,  0,  0,}
};

int16 *UFF;  //这样可以选择不用的U语言数组
/*  P 代表赛道复杂程度
D 表示前瞻
模糊控制算法通过这两个跟定量以及规则表给出当前应该输出的速度
*/
int16 FuzzySet_Speed(int16 P, int16 D)          //模糊运算引擎，返回速度值 
{
   int16 U;       /*偏差，以及输出值的精确量 */
   uint16 PF[2];
   uint16 DF[2]; 
   uint16 UF[4];  /*偏差，偏差微分以及输出值的隶属度PF[1]是P的隶属度，PF[0]是隶属度的补集 */
   int16 Pn = 0, Dn = 0;
   int16 Un[4];
   int32 temp1,temp2;
   
   if(P < PMIN)
     P = PMIN;
   else
     if(P > PMAX)
       P = PMAX;
   
   P = (int16)((double)(P - PMIN) / (PMAX - PMIN) * (SSMAX - SSMIN) + SSMIN); //归一化到SMIN ~ SMAX
   
   if(D < DMIN)
     D = DMIN;
   else 
     if( D > DMAX)
       D = DMAX;
    
   D = (int16)(((double)D )/ (DMAX - DMIN) * (SSMAX - SSMIN) + SSMIN) ;    //归一化到SMIN ~ SMAX

   /*隶属度的确定*/
   /*根据PD的指定语言获得有效的隶属度*/
   
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
    
   
      /*使用误差范围优化后的规则表rule[7][7]*/
      /*输出值使用13个隶属函数,中心值由UFF[7]指定*/
      /*一般都是四个规则有效*/
      Un[0] = rule[Pn - 1][ Dn - 1];
      Un[1] = rule[Pn][ Dn - 1];
      Un[2] = rule[Pn - 1][ Dn];
      Un[3] = rule[Pn][ Dn];
      if (PF[0] <= DF[0])//计算R表，隶属函数求交集
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
      /*同隶属函数输出语言值求大*/
      
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
      
      /*重心法(按隶属度加权平均)反模糊*/
      /*Un[]原值为输出隶属函数标号，转换为隶属函数值*/
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
   







