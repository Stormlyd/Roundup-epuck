#ifndef PF_H
#define PF_H
/**
 * This is public function.
 * By: Luzhimin
 */

#pragma once

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <algorithm>
#include <QString>
#include <windows.h>
#include <QPoint>
using namespace std;

float pfMap(float x, float in_min, float in_max, float out_min, float out_max);

float pfRandFloat(float in_min, float in_max);

int pfRandInt(int in_min, int in_max);

float constrain(float value, float min_val, float max_val);

QString pfAppPath();

#endif // PF_H
