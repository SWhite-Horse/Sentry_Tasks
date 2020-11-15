#define __FUNC_Limit_GLOBALS

#include "Func_Limit.h"
/******一些限制函数************/

/**
  * @brief  Float_Limit
  * @param  Data:数据值
  * @param  Data_Max:	限幅最大值
  * @param  Data_Min:	限幅最小值
  * @note float
  */
void Float_Limit(float *Data,float Data_Max,float Data_Min)
{
	if(*Data>Data_Max)
	{
   *Data=Data_Max;
	}
	else if(*Data<Data_Min)
	{
	 *Data=Data_Min;
	}
}
/**
  * @brief  Int_Limit
  * @param  Data:数据值
  * @param  Data_Max:	限幅最大值
  * @param  Data_Min:	限幅最小值
  * @note int16_t 
  */
void Short_Limit(int16_t *Data,int16_t Data_Max,int16_t Data_Min)
{
	if(*Data>Data_Max)
	{
   *Data=Data_Max;
	}
	else if(*Data<Data_Min)
	{
	 *Data=Data_Min;
	}
}

