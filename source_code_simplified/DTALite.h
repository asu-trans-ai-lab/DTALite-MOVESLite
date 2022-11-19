#pragma once

#pragma warning(disable:4244)  // stop warning: "conversion from 'int' to 'float', possible loss of data"
#pragma warning(disable:4996)  // stop warning: 'MBCS_Support_Deprecated_In_MFC': MBCS support in MFC is deprecated 
#include "resource.h"


#include <math.h>
#include <deque>
#include <map>
#include <set>
#include <iostream>
#include <vector>
#include <list>
#include "CSVParser.h"


enum SPEED_BIN {VSP_0_25mph=0, VSP_25_50mph, VSP_GT50mph, MAX_SPEED_BIN};
enum VSP_BIN {VSP_LT0=0,VSP_0_3, VSP_3_6, VSP_6_9, VSP_9_12, VSP_12_18, VSP_18_24, VSP_24_30, VSP_GT30, MAX_VSP_BIN};

#ifdef _WIN64
#define MAX_LINK_NO 99999999
#endif 

#ifndef _WIN64
#define MAX_LINK_NO 65530
#endif

#define MAX_VEHICLE_TYPE_SIZE 10
#define MAX_CPU_SIZE 20
// Linear congruential generator 
#define LCG_a 17364
#define LCG_c 0
#define LCG_M 65521  // it should be 2^32, but we use a small 16-bit number to save memory

void g_ProgramStop();
class VehicleSpeedProfileData
{
public:
	float Speed;
	float Acceleration;
	int OperationMode;
	float VSP;
	VSP_BIN VSPBinNo;
	SPEED_BIN SpeedBinNo;
	float Energy, CO2, NOX, CO, HC;
	int TimeOnThisLinkInSecond;

	CString GetSpeedBinNoString()
	{
		CString str;
		switch (SpeedBinNo)
		{
		case VSP_0_25mph: str.Format("0_25 mph"); break;
		case VSP_25_50mph: str.Format("25_50 mph"); break;
		case VSP_GT50mph: str.Format(">=50 mph"); break;
		default: str.Format("Unknown"); break;
		}
		return str;

	}

	CString GetVSPBinNoString()
	{
		CString str;
		switch (VSPBinNo)
		{
		case VSP_LT0: str.Format("VSP_<=0"); break;
		case VSP_0_3: str.Format("VSP_0_3"); break;
		case VSP_3_6: str.Format("VSP_3_6"); break;
		case VSP_6_9: str.Format("VSP_6_9"); break;
		case VSP_9_12: str.Format("VSP_9_12"); break;
		case VSP_12_18: str.Format("VSP_12_18"); break;
		case VSP_18_24: str.Format("VSP_18_24"); break;
		case VSP_24_30: str.Format("VSP_24_30"); break;
		case VSP_GT30: str.Format("VSP_>=30"); break;
		default: str.Format("Unknown"); break;
		}
		return str;

	}

};

class DTAVehicleType
{
public:
	DTAVehicleType()
	{
		vehicle_type = 1;
		rollingTermA = 0.156461f;
		rotatingTermB = 0.00200193f;
		dragTermC = 0.000492646f;
		sourceMass = 1.4788f;


	}

	float rollingTermA;
	float rotatingTermB;
	float dragTermC;
	float sourceMass;

	int vehicle_type;
	string vehicle_type_name;
	std::vector<float> percentage_age_vector;
};

class EmissionStatisticsData
{
public: 

	float TotalEnergy;
	float TotalCO2;
	float TotalNOX;
	float TotalCO;
	float TotalHC;

	float AvgEnergy;
	float AvgCO2;
	float AvgNOX;
	float AvgCO;
	float AvgHC;

	float TotalMilesPerGallon;
	float AvgMilesPerGallon;

	float TotalMiles;
	float TotalGasolineGallon;


	EmissionStatisticsData()
	{
		Init();
	}
	void Init()
	{
		TotalEnergy = 0;
		TotalCO2 = 0;
		TotalNOX = 0;
		TotalCO = 0;
		TotalHC = 0;

		AvgEnergy = 0;
		AvgCO2  = 0;
		AvgNOX  = 0;
		AvgCO = 0;
		AvgHC = 0;
		TotalMilesPerGallon = 0;
		AvgMilesPerGallon = 0;

		TotalMiles = 0;
		TotalGasolineGallon = 0;
	}
};


class EmissionLaneData
{
public: 

	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;
	EmissionLaneData()
	{
		Energy = 0;
		CO2 = 0;
		NOX = 0;
		CO = 0;
		HC = 0;
	}

};



class DTAVehicle
{
public:

	double within_link_driving_distance;  // unit: mile
	double within_link_driving_speed_per_hour;  // unit: mile per hour

	bool b_already_output_flag;
	bool b_od_already_output_flag;



	int m_EvacuationTime_in_min;
	int m_EvacuationDestinationZone;
	bool m_bEvacuationMode;
	bool m_bEvacuationResponse;
	//std::map<int,DTAPath> Day2DayPathMap;
	//// multi-day equilibrium
	//bool m_bETTFlag;
	//int m_DayDependentLinkSize[MAX_DAY_SIZE];
	//std::map<int, int> m_DayDependentAryLink;  // first key: day*MAX_PATH_LINK_SIZE + index, second element; return link index
	//float m_DayDependentTripTime[MAX_DAY_SIZE];
	//int m_DayDependentNodeNumberSum[MAX_DAY_SIZE];  // used for comparing two paths
	//float m_DayDependentGap[MAX_DAY_SIZE];  // used for gap analysis
	//float m_AvgDayTravelTime;
	//float m_DayTravelTimeSTD;


	int m_NodeSize;
	int m_NodeNumberSum;  // used for comparing two paths

	float m_PrevSpeed;

	unsigned int m_RandomSeed;
	int m_VehicleID;  //range: +2,147,483,647
	int m_ExternalTripID;

	int m_OriginZoneID;  
	int m_DestinationZoneID; 

	int m_OriginNodeID;
	int m_DestinationNodeID;

	int m_DemandType;     // 1: passenger,  2, HOV, 3, truck, 3: bus
	int m_PricingType;     // 1: passenger,  2, HOV, 3, truck, 3: bus
	int m_VehicleType;    // for emissions analysis
	int m_InformationClass;  // 0: historical path (no change), 1: historical learning using time-dependent travel time from the previous day, 2: pre-trip, 3: en-route; 4: personalized info; 5: eco so info

	int m_SimLinkSequenceNo; //  range 0, 65535

	bool  m_bImpacted;

	double m_TimeToRetrieveInfo;

	bool m_bRadioMessageResponseFlag;

	float m_DepartureTime;
	float m_LeavingTimeFromLoadingBuffer;

	float m_PreferredDepartureTime;
	float m_Distance;

	float m_ArrivalTime;
	float m_TripTime;
	float m_TripFFTT;
	float m_BufferWaitingTime;
	float m_TravelTime;
	float m_EstimatedTravelTime;
	float m_Delay;

	bool m_bForcedSwitchAtFirstIteration; // used by agent model, if there is a newly added work zone, then we have to force the vehicles to switch (in order to avoid bloced links)
	bool m_bSwitch;  // switch route in assignment
	float m_gap;
	bool m_bMeetTarget;
	bool m_gap_update;
	bool m_bConsiderToSwitch;  //consider to switch route in assignment

	// used for simulation
	bool m_bLoaded; // be loaded into the physical network or not
	bool m_bComplete;

	bool m_bDetailedEmissionOutput;
	float Energy,CO2,NOX,CO,HC;

	int m_DestinationZoneID_Updated; 
	int m_attribute_update_time_in_min;

	int m_Age;
	float m_VOT;        // range 0 to 255
	float m_TollDollarCost;
	float m_Emissions;
	float m_MinCost;
	float m_MeanTravelTime;
	float m_TravelTimeVariance;
	unsigned short m_NumberOfSamples;  // when switch a new path, the number of samples starts with 0
	std::map<int, int> m_OperatingModeCount;
	std::map<int, int> m_SpeedCount;
	std::map<int, VehicleSpeedProfileData> m_SpeedProfile;

	DTAVehicle()
	{

		m_bMeetTarget = false;
		m_NodeNumberSum = 0;
		m_OriginNodeID = -1;
		m_DestinationNodeID = -1;


		b_already_output_flag = false;
		b_od_already_output_flag = false;

		m_ExternalTripID = 0;
		m_gap = 0;
		m_gap_update = false;
		m_bForcedSwitchAtFirstIteration  = false;
		m_bRadioMessageResponseFlag = false;

		m_DestinationZoneID_Updated = 0;
		m_attribute_update_time_in_min = -1;

		m_bEvacuationMode = false;
		m_bEvacuationResponse = false;
		m_EvacuationTime_in_min = 0;
		m_EvacuationDestinationZone = 0;

		m_Age = 0;
		Energy = CO2 = NOX = CO = HC = 0;
		m_PrevSpeed = 0;
		m_TimeToRetrieveInfo = -1;
		m_SimLinkSequenceNo = 0;

		m_NumberOfSamples =0;
		m_TollDollarCost = 0;
		m_Emissions = 0;

		m_MinCost = 0;

		m_NodeSize	= 0;
		m_bImpacted = false; 
		m_DemandType = 1;
		m_VehicleType = 1;
		m_PricingType = 0;
		m_Emissions = 0;
		m_ArrivalTime = 0;
		//      m_FinalArrivalTime = 0;
		m_bLoaded = false;
		m_bSwitch = false;
		m_bConsiderToSwitch = false;
		m_bComplete = false;
		m_TripTime = 900;  // default: for incomplete vehicles, they have an extremey long trip time
		m_TravelTime = 900;
		m_Distance =0;
		m_Delay = 0;

		m_bDetailedEmissionOutput = false;

	};
	~DTAVehicle()
	{

	};


};
