#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
} extKalman_t;

float KalmanFilter(extKalman_t* p,float dat);
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
extern extKalman_t speedK;//卡尔曼滤波器结构体

#endif
