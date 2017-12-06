#include "pid.h"


/*λ��ʽPID������*/
/**
  * @brief  PID calculate
  * @param  PID *pp            : Pointer to a PIDTypeDef structure
  *         double NextPoint   : Get the MPU6050's instance value 
  * @retval The last PID's calculating result
  */
float PIDCalc( PID *pp, float NextPoint )
{
        float dError, iError;
        iError = pp->SetPoint - NextPoint;          // �������ֵ�  ����һ�β��������ƫ��
        pp->SumError += iError;                     // ���ֲ��ֵ�  ����ƫ����ۼ�
        dError = iError - pp->LastError;            // ΢�ֲ��ֵ�  ����һ��ƫ���ǰһ��ƫ��ļ�������
        pp->LastError = iError;                     // ΢�ֲ��ֵ�  ��������ƫ��Ϊ��ǰƫ��
                                                    // ������� + ������� + ΢�����
        return (pp->Proportion * iError  +  pp->Integral * pp->SumError  +  pp->Derivative * dError); 
                                                    
}

/*λ��ʽPID������*/
/*
  * @brief  PID calculate
  * @param  PID_right *pp           : Pointer to a PIDTypeDef structure
  *         float NextPoint_right   : Get the MPU6050's instance value 
  * @retval The last PID's calculating result
  */

float PIDCalc_IN( PID *pp, float NextPoint )
{        
        float iError,iIncpid;
        iError = pp->SetPoint - NextPoint;//��ǰ���
        iIncpid = pp->Proportion * iError - pp->Integral * pp->LastError + pp->Derivative * pp->PrevError;//��������
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
/*λ��ʽPID*/
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
/*����ʽPID*/
float Vol_In(PID *pp ,float Set,float InPut)
{
    float Out;
	pp->SetPoint = Set;                 // Set PID Setpoint
	Out = PIDCalc_IN(pp ,InPut);        // Perform PID Interation
	return Out;
} 













