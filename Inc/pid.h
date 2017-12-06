

/*-----------------------------------------------------------------------

    ����PID�ṹ�壬�����˼������õ��ĸ��ֱ���
==============================================================================*/
typedef struct pid
{
        float SetPoint;   // �趨Ŀ��ֵr(t) Desired Value

        float Proportion; // ��������k(p) Proportional Const
        float Integral;   // ���ֳ���k(i) Integral Const
        float Derivative; // ΢�ֳ���k(d) Derivative Const

        float LastError; // ΢�ֲ��ֵ� ����ƫ��e(k)    Error[-1]
        float PrevError; // ΢�ֲ��ֵ� ����ƫ��e(k-1)  Error[-2]
        float SumError;  // ���ֲ��ֵ� �ۼ�ƫ��   Sums of Errors

}PID;

float PIDCalc_IN( PID *pp, float NextPoint );
float PIDCalc( PID *pp, float NextPoint );
float Vol_In(PID *pp ,float Set,float InPut);
float Vol(PID *pp ,float Set,float InPut);


//������������ѣ���С����˳��顣
//���Ǳ�������֣�����ٰ�΢�ּӡ�
//�����𵴺�Ƶ������������Ҫ�Ŵ�
//����Ư���ƴ��䣬����������С�⡣
//����ƫ��ظ���������ʱ�����½���
//���߲������ڳ�������ʱ���ټӳ���
//������Ƶ�ʿ죬�Ȱ�΢�ֽ�������
//���������������΢��ʱ��Ӧ�ӳ���
//����������������ǰ�ߺ���ı�һ��
//һ�������������������������͡�
