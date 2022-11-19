
// Becasese the functions below might relate to file interfaces with other proprietary software packages, no copyright or GPL statement is made here.

// Utility.cpp : Utility functions used for reading and outputing

#include "stdafx.h"
#include "math.h"
#include "DTALite.h"
#include "GlobalData.h"

#include <algorithm>
#include <functional>

using namespace std;

extern CTime g_AppStartTime;
extern CTime g_AppLastIterationStartTime;
// polar form of the Box-Muller transformation to get two random numbers that follow a standard normal distribution 
unsigned int g_RandomSeedForVehicleGeneration = 101;
// Linear congruential generator 
#define g_LCG_a 17364
#define g_LCG_c 0
#define g_LCG_M 65521  

long g_precision_constant=100L;
long g_precision_constant2=100L;
extern float g_DemandGlobalMultiplier;
int g_ProgramStopFlag = 0;

#include<iostream>
#include<cmath>
using namespace std;



//******************************************************************************
// linear regression code is modified based on
// http://codesam.blogspot.com/2011/06/least-square-linear-regression-of-data.html
//********************************************************************************/



std::string CString2StdString(CString str)
{	 // Convert a TCHAR string to a LPCSTR
	CT2CA pszConvertedAnsiString (str);

	// construct a std::string using the LPCSTR input
	std::string strStd (pszConvertedAnsiString);

	return strStd;
}

std::string GetTimeClockString(int time)
{
	int hour = ((int)(time))/60;
	int min = time - hour*60;

	CString time_str;
	time_str.Format("%2d:%02d",hour, min);
	return (CString2StdString(time_str));
}

bool g_floating_point_value_less_than_or_eq_comparison(double value1, double value2)
{
	long lValue1 = (long) (value1*g_precision_constant);
	long lValue2 = (long) (value2*g_precision_constant);

	if( lValue1 < lValue2 + 2) // we take 2/100 sec accuracy
		return true;
	else 
		return false;

}

bool g_floating_point_value_less_than(double value1, double value2)
{
	long lValue1 = (long) (value1*g_precision_constant2);
	long lValue2 = (long) (value2*g_precision_constant2);


	if(lValue1<lValue2)
		return true;
	else 
		return false;

}



void g_ProgramStop()
{
	
	getchar();
	exit(0);
}