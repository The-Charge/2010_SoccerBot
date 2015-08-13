#ifndef _TARGET_H
#define _TARGET_H
//THIS FILE WAS CHANGED TO ALLOW US TO SET SOME TARGET PARAMITER PROGRAMATICALLY 
#include <vector>
#include "Vision/HSLImage.h"

class Target
{
public:
    double m_majorRadius;
    double m_minorRadius;
    double m_rawScore;
    double m_xPos;
    double m_yPos;
    double m_score;
    double m_rotation;
    double m_xMax;
    bool m_bothFound;


    static void setthres(int x);    
    static void setminmatc(double x);    
    static void setonlyclos(int x);  
    static void setsubpix(int x);    
    static int getthres();    
    static double getminmatc();
    static int getonlyclos();
    static int getsubpix();
    static vector<Target> FindCircularTargets(HSLImage *image);
    double GetHorizontalAngle();
    void Print();
};

#endif
