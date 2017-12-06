

/*-----------------------------------------------------------------------

    定义PID结构体，包含了计算中用到的各种变量
==============================================================================*/
typedef struct pid
{
        float SetPoint;   // 设定目标值r(t) Desired Value

        float Proportion; // 比例常数k(p) Proportional Const
        float Integral;   // 积分常数k(i) Integral Const
        float Derivative; // 微分常数k(d) Derivative Const

        float LastError; // 微分部分的 最新偏差e(k)    Error[-1]
        float PrevError; // 微分部分的 最新偏差e(k-1)  Error[-2]
        float SumError;  // 积分部分的 累加偏差   Sums of Errors

}PID;

float PIDCalc_IN( PID *pp, float NextPoint );
float PIDCalc( PID *pp, float NextPoint );
float Vol_In(PID *pp ,float Set,float InPut);
float Vol(PID *pp ,float Set,float InPut);


//参数整定找最佳，从小到大顺序查。
//先是比例后积分，最后再把微分加。
//曲线震荡很频繁，比例度盘要放大。
//曲线漂浮绕大弯，比例度盘往小扳。
//曲线偏离回复慢，积分时间往下降。
//曲线波动周期长，积分时间再加长。
//曲线震荡频率快，先把微分降下来。
//动差大来波动慢，微分时间应加长。
//理想曲线两个波，前高后低四比一。
//一看二调多分析，调节质量不会低。
