#include "pf.h"



QPoint batonpoint={0,0};
bool beFollowed=false;
/**
 * @brief pfMap
 * @param x
 * @param in_min
 * @param in_max
 * @param out_min
 * @param out_max
 * @return
 */
float pfMap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrain(float value, float min_val, float max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief fpRandFloat
 * @param in_min
 * @param in_max
 * @return
 */
float pfRandFloat(float in_min, float in_max)
{
    static bool RAND_FLAG1 = true;
    if(RAND_FLAG1){
         srand((int)(time(0)));
         RAND_FLAG1 = false;
    }

    if( in_min > in_max)
        swap(in_min, in_max);

    double diff = fabs(in_max - in_min);
    double randNumber=(double)(rand() % ((int)(diff*1000))) / 1000.0;
    double retval = in_min + randNumber;
    return retval;
}

/**
 * @brief fpRandInt
 * @param in_min
 * @param in_max
 * @return
 */
int pfRandInt(int in_min, int in_max)
{
    static bool RAND_FLAG2 = true;
    if(RAND_FLAG2){
         srand((int)(time(0)));
         RAND_FLAG2 = false;
    }

   if( in_min > in_max)
       swap(in_min, in_max);

    int diff = fabs(in_max - in_min);
    int randNumber = (rand() % diff);
    int retval = in_min + randNumber;
    return retval;
}


QString pfAppPath()
{
    const DWORD maximumWindowsPath = 32768;
    wchar_t filePath[maximumWindowsPath] = {0};
    const DWORD length = GetModuleFileNameW(nullptr, filePath, maximumWindowsPath);
    if (length == 0 || length >= maximumWindowsPath)
        return QStringLiteral(".\\");

    const QString executablePath = QString::fromWCharArray(filePath, static_cast<int>(length));
    const int separator = executablePath.lastIndexOf(QLatin1Char('\\'));
    return separator >= 0 ? executablePath.left(separator + 1) : QStringLiteral(".\\");
}
