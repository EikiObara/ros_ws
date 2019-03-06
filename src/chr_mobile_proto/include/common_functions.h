/*
単位変換・数値入力関数をまとめたファイル

佐々木　成海
*/

#ifndef COMMON_FUNCTIONS_H_
#define COMMON_FUNCTIONS_H_

#include<string>

//時間取得関数
std::string GetTime();

//単位変換関数
double RadToDeg(const double rad);
double DegToRad(const double deg);
long RadToQuadCount(const double rad, const double resolution, const double reductionRatio);
long DegToQuadCount(const double deg, const double resolution, const double reductionRatio);
double QuadCountToRad(const long qc, const double resolution, const double reductionRatio);
double QuadCountToDeg(const long qc, const double resolution, const double reductionRatio);

//キーボード入力関数
int InputInt();
double InputDouble();
int InputBoolean();

#endif  //COMMON_FUNCTIONS_H_
