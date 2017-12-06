#include "pid.h"


/*位置式PID浮点型*/
/**
  * @brief  PID calculate
  * @param  PID *pp            : Pointer to a PIDTypeDef structure
  *         double NextPoint   : Get the MPU6050's instance value 
  * @retval The last PID's calculating result
  */
float PIDCalc( PID *pp, float NextPoint )
{
        float dError, iError;
        iError = pp->SetPoint - NextPoint;          // 比例部分的  最新一次采样计算的偏差
        pp->SumError += iError;                     // 积分部分的  所有偏差的累加
        dError = iError - pp->LastError;            // 微分部分的  最新一次偏差和前一次偏差的减法计算
        pp->LastError = iError;                     // 微分部分的  更新最新偏差为当前偏差
                                                    // 比例项部分 + 积分项部分 + 微分项部分
        return (pp->Proportion * iError  +  pp->Integral * pp->SumError  +  pp->Derivative * dError); 
                                                    
}

/*位置式PID浮点型*/
/*
  * @brief  PID calculate
  * @param  PID_right *pp           : Pointer to a PIDTypeDef structure
  *         float NextPoint_right   : Get the MPU6050's instance value 
  * @retval The last PID's calculating result
  */

float PIDCalc_IN( PID *pp, float NextPoint )
{        
        float iError,iIncpid;
        iError = pp->SetPoint - NextPoint;//当前误差
        iIncpid = pp->Proportion * iError - pp->Integral * pp->LastError + pp->Derivative * pp->PrevError;//增量计算
        pp->PrevError = pp->LastError;
        pp->LastError = iError;
        return iIncpid;
}

/**
  * @brief  Get the param bofore PID calculating
  * @param  int vSet           : The value which user sets
  *         int v3             : Get the MPU6050's instance value 
  * @retval The last pwm value
  */
/*位置式PID*/
float Vol(PID *pp ,float Set,float InPut)
{
        float Out;
	pp->SetPoint = Set; // Set PID Setpoint
	Out = PIDCalc(pp ,InPut); // Perform PID Interation	 
	return Out;
} 

/**
  * @brief  Get the param bofore PID calculating
  * @param  int vSet           : The value which user sets
  *         int v3             : Get the MPU6050's instance value 
  * @retval The last pwm value
  */
/*增量式PID*/
float Vol_In(PID *pp ,float Set,float InPut)
{
    float Out;
	pp->SetPoint = Set;                 // Set PID Setpoint
	Out = PIDCalc_IN(pp ,InPut);        // Perform PID Interation
	return Out;
} 













