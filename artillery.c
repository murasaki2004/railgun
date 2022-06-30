#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#define x_def
#define b_def
#define l_def
#define m_def
#define i_def

#define r_g 9.81
#define V_0 10
//sqrt(2*x_def*b_def*i_def*l_def/m_def)


// 四舍五入
int rounded(double input){
    if (input < 0)
        return input;
    int re = (int)(input + 0.5);
    return re;
}

// 射程上限
double range_max(void){
    double v = V_0*V_0;
    v = v/r_g;
    return v;
}


// 平面角度
double angle(double x, double y){
    // 仅限一、二象限
    // 角度位相对X轴正方向的角度
    if (y > 0){
        bool tmp = false;
        // 去符号
        if (x < 0){
            x = sqrt(pow(x, 2.0));
            tmp = true;
        }
        // 计算另一条边长
        double c = sqrt(x*x + y*y);
        // 用反正弦计算相对X轴的偏转弧度
        double angle = asin(y / c);

        // 把弧度转换成角度后输出，第一象限补上90°
        if (tmp)
            return (angle * (180 / M_PI)) + 90;
        else
            return angle * (180 / M_PI);
    } else {
        return -1;
    }
}

double placement(double x, double y, double c){
    // 验证数据合法
    if (y < 0 || c < 0)
        return -1;
    else if (c+x+y == 0)
        return -1;

    // 如果传入xy则计算c并判断射程上限
    if (c == 0)
        c = sqrt(x*x + y*y);
    if (c > range_max())
        return -1;

    // 计算倾角
    double angle = asin((c * r_g) / pow(V_0, 2.0));
    angle = angle * (180 / M_PI) / 2;
    return angle;
}

int main(void){
    return 0;
}