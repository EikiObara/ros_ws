#include <stdio.h>
#include <math.h>
#include <sys/time.h>
#include "common_functions.h"

std::string GetTime(){
    timeval curTime;    //時間データに関する構造体
    gettimeofday(&curTime, NULL);   //現在のUNIX時間を取得
    int milli = curTime.tv_usec/1000;   //ミリ秒データを作る
    char buff[80];
    strftime(buff, 80, "%Y/%m/%d %H:%M:%S", localtime(&curTime.tv_sec)); //UNIX時間を日本時間に変換
    char currentTime[84] = "";
    sprintf(currentTime, "%s.%03d", buff, milli);   //文字列連結

    return std::string(currentTime);
}

double RadToDeg(const double rad){
    return (180 / M_PI) * rad;
}

double DegToRad(const double deg){
    return (M_PI / 180) * deg;
}

long RadToQuadCount(const double rad, const double resolution, const double reductionRatio){
    if(rad >= 0){
        return static_cast<long>((0.5 * M_PI) * (rad * 4 * resolution * reductionRatio) + 0.5/*四捨五入*/);
    } else {
        return static_cast<long>((0.5 * M_PI) * (rad * 4 * resolution * reductionRatio) - 0.5);
    }
}

long DegToQuadCount(const double deg, const double resolution, const double reductionRatio){
    if(deg >= 0){
        return static_cast<long>((deg * 4 * resolution * reductionRatio / 360) + 0.5/*四捨五入*/);
    } else {
        return static_cast<long>((deg * 4 * resolution * reductionRatio / 360) - 0.5);
    }
}

double QuadCountToRad(const long qc, const double resolution, const double reductionRatio){
    return (static_cast<double>(qc) * 2 * M_PI) / (4 * resolution * reductionRatio);
}

double QuadCountToDeg(const long qc, const double resolution, const double reductionRatio){
    return (static_cast<double>(qc) * 360) / (4 * resolution * reductionRatio);
}

//int型の数値をキーボード入力するための関数
//scanf()の無限ループバグを抑制するため関数化
int InputInt(){
    int num = 0;
    while(1){
        if(scanf("%d", &num) != 1){
            scanf("%*s");
            if(feof(stdin)){
                printf("入力エラー\n");
            }
            printf("入力が不正です．再入力してください．\n");
            continue;
        } else { return num; }
    }
}

//double型の数値をキーボード入力するための関数
//scanf()の無限ループバグを抑制するため関数化
double InputDouble(){
    double num = 0;
    while(1){
        if(scanf("%lf", &num) != 1){
            scanf("%*s");
            if(feof(stdin)){
                printf("入力エラー\n");
            }
            printf("入力が不正です．再入力してください．\n");
            continue;
        } else { return num; }
    }
}

//キーボードから1か0のみを入力するための関数
int InputBoolean(){
    int num=1;
    while(1){
        num = InputInt();
        if(num==1){
            return 1;
        } else if(num==0){
            return 0;
        } else {
            continue;
        }
    }
}
