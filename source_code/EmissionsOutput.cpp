//  Portions Copyright 2010 Xuesong Zhou, Hao Lei

//   If you help write or modify the code, please also list your names here.
//   The reason of having copyright info here is to ensure all the modified version, as a whole, under the GPL 
//   and further prevent a violation of the GPL.

// More about "How to use GNU licenses for your own software"
// http://www.gnu.org/licenses/gpl-howto.html

//    This file is part of DTALite.

//    DTALite is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    DTALite is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with DTALite.  If not, see <http://www.gnu.org/licenses/>.

// DTALite.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "DTALite.h"
#include "GlobalData.h"
#include "CSVParser.h"

#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>


using namespace std;

const int NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND = 10;
#define _MAXIMUM_OPERATING_MODE_SIZE 41
#define _MAXIMUM_AGE_SIZE 31

int OperatingModeMap[MAX_SPEED_BIN][MAX_VSP_BIN] = {-1};

class CEmissionRate 
{
public:
	bool bInitialized;
	float meanBaseRate_TotalEnergy;
	float meanBaseRate_CO2;
	float meanBaseRate_NOX;
	float meanBaseRate_CO;
	float meanBaseRate_HC;
	int Age;

	CEmissionRate()
	{
		bInitialized = false;
		meanBaseRate_TotalEnergy = meanBaseRate_CO2 = meanBaseRate_NOX = meanBaseRate_CO = meanBaseRate_HC = 0;
		Age = 0;
	}
};

CEmissionRate EmissionRateData[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_OPERATING_MODE_SIZE][_MAXIMUM_AGE_SIZE];

class CCycleAverageEmissionFactor
{
public:
	float emissionFactor_CO2;
	float emissionFactor_NOX;
	float emissionFactor_CO;
	float emissionFactor_HC;
	float average_cycle_speed;

	CCycleAverageEmissionFactor()
	{
		this->emissionFactor_CO2 = this->emissionFactor_NOX = this->emissionFactor_CO = this->emissionFactor_HC = 0.0f;
	}
	CCycleAverageEmissionFactor(float emissionFactor_CO2, float emissionFactor_NOX, float emissionFactor_CO, float emissionFactor_HC)
	{
		this->emissionFactor_CO2 = emissionFactor_CO2;
		this->emissionFactor_NOX = emissionFactor_NOX;
		this->emissionFactor_CO = emissionFactor_CO;
		this->emissionFactor_HC = emissionFactor_HC;
	}
};

map<float,CCycleAverageEmissionFactor> CycleAverageEmissionFactorMap[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE];

class CVehicleEmissionResult
{
public:
	float Energy;
	float CO2;
	float NOX;
	float CO;
	float HC;

	static void CalculateEmissions(int vehicle_type, std::map<int, int>& OperatingModeCount, int age, CVehicleEmissionResult& emissionResult)
	{
		emissionResult.Energy = 0;
		emissionResult.CO2 = 0;
		emissionResult.NOX = 0;
		emissionResult.CO = 0;
		emissionResult.HC = 0;

		for(std::map<int, int>::iterator iterOP  = OperatingModeCount.begin(); iterOP != OperatingModeCount.end (); iterOP++)
		{
			int OpModeID = iterOP->first; int count = iterOP->second;

			emissionResult.Energy+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_TotalEnergy*count/3600;
			emissionResult.CO2+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO2*count/3600;
			emissionResult.NOX+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_NOX*count/3600;
			emissionResult.CO+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO*count/3600;
			emissionResult.HC+= EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_HC*count/3600;
		}
	}
};

SPEED_BIN GetSpeedBinNo(float speed_mph)
{
	//the precision of speed output is 2
	//if the speed is 49.995, it would round up to 50
	//if the speed is 49.994, it would round to 49.99
	int speed = floor(speed_mph + 0.005);

	if(speed <= 25)
		return VSP_0_25mph;

	if(speed < 50)
		return VSP_25_50mph;
	else 
		return VSP_GT50mph;

}

VSP_BIN GetVSPBinNo(float vsp, float speed_mph)
{
	//if(speed_mph < 50)
	//{
	if (vsp < 0) return VSP_LT0;

	if (vsp < 3) return VSP_0_3;

	if (vsp < 6) return VSP_3_6;

	if (vsp < 9) return VSP_6_9;

	if (vsp < 12) return VSP_9_12;

	if (vsp < 18) return VSP_12_18;

	if (vsp < 24) return VSP_18_24;

	if (vsp < 30) return VSP_24_30;

	return VSP_GT30;

	//}
	//else  // greate than 50
	//{
	//	if(vsp < 6) 
	//		return VSP_LT6;
	//	if(vsp < 12) 
	//		return VSP_6_12;
	//	if (vsp < 18)
	//		return VSP_12_18;
	//	if (vsp < 24)
	//		return VSP_18_24;
	//	if (vsp < 30)
	//		return VSP_24_30;
	//	else
	//		return VSP_GT30;
	//}
}

void ReadInputEmissionRateFile()
{
	CCSVParser parser_emission;

	int line=1;
	if (parser_emission.OpenCSVFile("input_vehicle_emission_rate.csv"))
	{

		while(parser_emission.ReadRecord())
		{ line++;

		if(line == 256)
			TRACE("");


			int vehicle_type;
			int opModeID;

			if(parser_emission.GetValueByFieldName("vehicle_type",vehicle_type) == false)
				break;
			if(parser_emission.GetValueByFieldName("OpModeID",opModeID) == false)
				break;

			CEmissionRate element;
			if(parser_emission.GetValueByFieldName("meanBaseRate_TotalEnergy_(KJ/hr)",element.meanBaseRate_TotalEnergy) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_CO2_(g/hr)",element.meanBaseRate_CO2) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_NOX_(g/hr)",element.meanBaseRate_NOX) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_CO_(g/hr)",element.meanBaseRate_CO) == false)
				break;
			if(parser_emission.GetValueByFieldName("meanBaseRate_HC_(g/hr)",element.meanBaseRate_HC) == false)
				break;


			if(parser_emission.GetValueByFieldName("age",element.Age) == false)
			{
				break;
			}


			if(element.Age< _MAXIMUM_AGE_SIZE && opModeID < _MAXIMUM_OPERATING_MODE_SIZE)
			{

			EmissionRateData[vehicle_type][opModeID][element.Age] = element;
			EmissionRateData[vehicle_type][opModeID][element.Age].bInitialized = true;
			}else
			{
			cout << "Reading error at line "  << line <<" at input_vehicle_emission_rate.csv" <<endl;
			g_ProgramStop();
			
			}

		}
	}
	else
	{
		cout << "Error: File input_vehicle_emission_rate.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	cout << "Read " << line << " lines from file input_vehicle_emission_rate.csv."<< endl;
}

//typedef struct
//{
//	int second;
//	float speed;
//	float position;
//} SecondData;
//
//class DrivingCycleSample
//{
//public:
//	int driving_cycle_id;
//	vector<SecondData> dataVector;
//};
//	//string s1 = "Cycle_PC_PT_LCT.csv";
//	//string s2 = "CYCLE_LHT.csv";
//	//string s3 = "Cycle_SST.csv";
////map<int, 
//void GenerateDrivingCycleSamples(int vehicle_type, string drivingCycleFileName)
//{
//	FILE* pFile;
//	fopen_s(&pFile, drivingCycleFileName.c_str(),"r");
//	vector<DrivingCycleSample> drivingCycleSamples;
//	if (pFile)
//	{
//		int curr_cycle_id = -1;
//		int cycle_id, second;
//		float speed;
//		DrivingCycleSample samples;
//		while (!feof(pFile))
//		{
//			fscanf(pFile,"%d,%d,%f\n",&cycle_id, &second, &speed);
//			if (cycle_id != curr_cycle_id)
//			{
//				samples.driving_cycle_id = cycle_id;
//			}
//			SecondData data;
//			data.second = second;
//			data.speed = speed;
//			samples.dataVector.push_back();
//		}
//	}
//}

enum Pollutants {EMISSION_CO2,EMISSION_NOX,EMISSION_CO,EMISSION_HC,MAX_POLLUTANT};

float BaseCycleFractionOfOperatingMode[MAX_VEHICLE_TYPE_SIZE][41] = {0.0f};
float BaseCycleEmissionRate[MAX_VEHICLE_TYPE_SIZE][_MAXIMUM_AGE_SIZE][MAX_POLLUTANT] = {0.0f};

void ReadFractionOfOperatingModeForBaseCycle()
{
	CCSVParser parser_emission_factor;
	if (parser_emission_factor.OpenCSVFile("input_base_cycle_fraction_of_OpMode.csv"))
	{
		int vehicle_type;
		float value;

		while(parser_emission_factor.ReadRecord())
		{			
			if (parser_emission_factor.GetValueByFieldName("vehicle_type",vehicle_type) == false)
			{
				break;
			}
			else
			{
				if (vehicle_type >= MAX_VEHICLE_TYPE_SIZE)
				{
					cout << "Warning: unknown vehicle_type " << vehicle_type << " !\n";
					continue;
				}

				if (parser_emission_factor.GetValueByFieldName("0", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][0] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("1", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][1] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("11", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][11] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("12", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][12] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("13", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][13] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("14", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][14] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("15", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][15] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("16", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][16] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("21", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][21] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("22", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][22] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("23", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][23] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("24", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][24] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("25", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][25] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("27", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][27] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("28", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][28] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("29", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][29] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("30", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][30] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("33", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][33] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("35", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][35] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("37", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][37] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("38", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][38] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("39", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][39] = value;
				}

				if (parser_emission_factor.GetValueByFieldName("40", value))
				{
					BaseCycleFractionOfOperatingMode[vehicle_type][40] = value;
				}
			}
		}
	}
	else
	{
		cout << "Error: File input_base_cycle_fraction_of_OpMode.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}

	for (int vType = 0; vType < MAX_VEHICLE_TYPE_SIZE; vType++)
	{
		for (int vAge = 0; vAge < _MAXIMUM_AGE_SIZE; vAge++)
		{
			for (int opMode = 0; opMode < 41;opMode++)
			{
				if (EmissionRateData[vType][opMode][vAge].bInitialized)
				{
					BaseCycleEmissionRate[vType][vAge][EMISSION_CO2] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_CO2;
					BaseCycleEmissionRate[vType][vAge][EMISSION_NOX] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_NOX;
					BaseCycleEmissionRate[vType][vAge][EMISSION_CO] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_CO;
					BaseCycleEmissionRate[vType][vAge][EMISSION_HC] += BaseCycleFractionOfOperatingMode[vType][opMode] * EmissionRateData[vType][opMode][vAge].meanBaseRate_HC;
				}
			}
		}
	}
	cout << "Reading file input_base_cycle_fraction_of_OpMode.csv..."<< endl;
}

CCycleAverageEmissionFactor* BaseCaseEmissionFactors(int vehicle_type, int vehicle_age, float average_speed)
{
	if ((vehicle_type >= 0 && vehicle_type < MAX_VEHICLE_TYPE_SIZE)
		&& (vehicle_age >= 0 && vehicle_age < _MAXIMUM_AGE_SIZE))
	{
		map<float,CCycleAverageEmissionFactor>& cycleAverageEmissionFactorMap = CycleAverageEmissionFactorMap[vehicle_type][vehicle_age];
		map<float,CCycleAverageEmissionFactor>::iterator iter, targetIterator;		
		targetIterator = iter = cycleAverageEmissionFactorMap.begin();
		float abs_speed_diff = abs(iter->first - average_speed);

		for (;iter != cycleAverageEmissionFactorMap.end(); iter++)
		{
			if (abs_speed_diff < abs(iter->first - average_speed))
			{
				break;
			}
			else
			{
				abs_speed_diff = abs(iter->first - average_speed);
				targetIterator = iter;
			}
		}

		return &(targetIterator->second);
	}
	else
	{
		return NULL;
	}
}

typedef struct
{
	int second;
	float speed;
	float position;
} SecondData;

class DrivingCycleSample
{
public:
	int driving_cycle_id;
	int defaultTimeSpan;
	vector<SecondData> dataVector;
	DrivingCycleSample(int cycle_id, int timeSpan = 300)
	{
		driving_cycle_id = cycle_id;
		defaultTimeSpan = timeSpan;
	}

	void SetSegmentTimeSpan(int timeSpan)
	{
		defaultTimeSpan = timeSpan;
	}

	float GetSpeedByTime(unsigned int segmentID, int time)
	{
		if (segmentID * defaultTimeSpan + time < dataVector.size())
		{
			return dataVector[segmentID * defaultTimeSpan + time].speed;
		}
		else
		{
			return 0.0f;
		}
	}

	float GetPositionByTime(unsigned int segmentID, int time)
	{
		if (segmentID * defaultTimeSpan + time >= dataVector.size())
		{
			return -1.0f;
		}

		float position_offset;
		if (segmentID == 0)
		{
			position_offset = 0.0f;
		}
		else
		{
			position_offset = dataVector[segmentID * defaultTimeSpan].position;
		}

		return max(0.0f, dataVector[segmentID * defaultTimeSpan + time].position - position_offset);
	}
};

typedef struct
{
	int cycle_id;
	int segment_id;
} DrivingCycleIdentifier;

class TrajectoryMatrix
{
public:
	int timeWindowSize;
	int spaceWindowSize;
private:
	vector<DrivingCycleIdentifier>** pMatrix;
public:
	TrajectoryMatrix(int n = 300, int m = 800)
	{
		timeWindowSize = n;
		spaceWindowSize = m;
		pMatrix = new vector<DrivingCycleIdentifier>*[n];
		for (int i=0;i<n;i++)
		{
			pMatrix[i] = new vector<DrivingCycleIdentifier>[m];
		}
	}

	void AddToMatrix(DrivingCycleSample& sample, unsigned int segment_id)
	{
		for (int i=0;i<sample.defaultTimeSpan;i++)
		{		
			int posIdx = (int)sample.GetPositionByTime(segment_id,i);
			if (posIdx < 0) break;
			DrivingCycleIdentifier identifier;
			identifier.cycle_id = sample.driving_cycle_id;
			identifier.segment_id = segment_id;
			if (i < timeWindowSize && posIdx < spaceWindowSize)
			{
				pMatrix[i][posIdx].push_back(identifier);
			}
		}
	}

	vector<DrivingCycleIdentifier>* GetIdentifier(int n, int m) const
	{
		if (n < timeWindowSize && m < spaceWindowSize)
		{
			return &pMatrix[n][m];
		}

		return NULL;
	}

	~TrajectoryMatrix()
	{
		for (int i=0;i<timeWindowSize;i++)
		{
			delete [] pMatrix[i];
		}

		delete [] pMatrix;
	}
};

//TrajectoryMatrix trajectoryMatrixBin[MAX_VEHICLE_TYPE_SIZE][7];

int GetDrivingCycleVehicleSpeedBin(float speed)
{
	if (speed < 15.0f) return 0;
	if (speed < 30.0f) return 1;
	if (speed < 40.0f) return 2;
	if (speed < 50.0f) return 3;
	if (speed < 60.0f) return 4;
	if (speed < 70.0f) return 5;

	return 6;
}
map<int,vector<DrivingCycleSample*>> drivingCycleMap;

//void ReadDrivingCycleSamplesByVehicleType(int vehicle_type, char* fileName, vector<DrivingCycleSample*>& drivingCycleSamples)
//{
//	FILE* pFile;
//	fopen_s(&pFile,fileName, "r");
//	int curr_cycle_id = -1;
//	int cycle_id, second;
//	float speed, position;
//	DrivingCycleSample* pDrivingCycleSample;
//	while (!feof(pFile))
//	{
//		fscanf_s(pFile,"%d,%d,%f\n",&cycle_id, &second, &speed);
//		if (curr_cycle_id != cycle_id)
//		{
//			curr_cycle_id = cycle_id;
//			position = 0.0f;
//			pDrivingCycleSample = new DrivingCycleSample(cycle_id);
//			drivingCycleSamples.push_back(pDrivingCycleSample);
//		}
//
//		SecondData data;
//		data.second = second;
//		data.speed = speed;
//		if (second == 0)
//		{
//			position = 0.0f;
//		}
//		else
//		{
//			position += speed * 5280.0f / 3600.0f * 0.3048f;
//		}
//		data.position = position;
//		pDrivingCycleSample->dataVector.push_back(data);
//	}
//	fclose(pFile);
//
//	if (vehicle_type < MAX_VEHICLE_TYPE_SIZE)
//	{
//		for (int i=0;i<drivingCycleSamples.size();i++)
//		{
//			int segment_id = 0;
//			for (int j=0;j<drivingCycleSamples[i]->dataVector.size();j=j+drivingCycleSamples[i]->defaultTimeSpan)
//			{
//				float initial_speed = drivingCycleSamples[i]->dataVector[i].speed;
//				int speed_bin = GetDrivingCycleVehicleSpeedBin(initial_speed);
//				trajectoryMatrixBin[vehicle_type][speed_bin].AddToMatrix(*drivingCycleSamples[i],segment_id);
//				segment_id++;
//			}
//		}
//	}
//}

int SmoothVehicleAcceleration(float link_ff_speed, float link_length, float initial_speed, int start_time, int end_time, float* position, int size)
{
	 
	const float Acceleration_Rate = 7.5f; //8mph/s
	const float Deceleration_Rate = 5.0f; //3.3mph/s
	float speed_above_ff_speed = 10.0; //mph

	//t1: acceleration time: from initial_speed to link_ff_speed + speed_above_ff_speed
	//t2: constant cruising time 
	//t3: deceleration time: from link_ff_speed + speed_above_ff_speed to link_ff_speed

	float t1 = (link_ff_speed + speed_above_ff_speed - initial_speed) / Acceleration_Rate; // in second
	float t3 = speed_above_ff_speed / Deceleration_Rate; // in second
	float t2 = (link_ff_speed * 5280.0f / 3600.0f * (t1 + t3) - (initial_speed * 5280.0f / 3600.0f * t1 + 0.5f * Acceleration_Rate * 5280.0f / 3600.0f * t1 * t1 + (link_ff_speed + speed_above_ff_speed) * (5280.0f / 3600.0f) * t3 - 0.5f * Deceleration_Rate * (5280.0f / 3600.0f) * t3 * t3)) / (speed_above_ff_speed * 5280.0f / 3600.0f);

	if ((int)((t1 + t2 + t3) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND) <= (end_time - start_time))
	{
		int end_of_duration_1 = start_time + (int)(t1 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		int end_of_duration_2 = end_of_duration_1 + (int)(t2 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		int end_of_duration_3 = end_of_duration_2 + (int)(t3 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		for (int t = start_time; t < end_of_duration_1; t++)
		{
			position[t % size] = initial_speed * 5280.0f / 3600.0f + 0.5f * Acceleration_Rate * 5280.0f / 3600.0f * (t - start_time + 1) * (t - start_time) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		}

		for (int t = end_of_duration_1; t < end_of_duration_2; t++)
		{
			position[t % size] = position[end_of_duration_1 -1] + (link_ff_speed + speed_above_ff_speed) * 5280.0f / 3600.0f * (t - end_of_duration_1 + 1) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
		}

		for (int t = end_of_duration_2; t < end_of_duration_3; t++)
		{
			position[t % size] = position[end_of_duration_2 - 1] + (link_ff_speed + speed_above_ff_speed) * 5280.0f / 3600.0f * (t - end_of_duration_2 + 1) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND - 0.5f * Deceleration_Rate * 5280.0f / 3600.0f * (t - end_of_duration_2) * (t - end_of_duration_2) / (NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);
		}

		return (t1+t2+t3);
	}
	else
	{
		return 0;
	}
}

void SetupOperatingModeVector()
{
	OperatingModeMap[VSP_0_25mph][VSP_LT0] = 11;
	OperatingModeMap[VSP_0_25mph][VSP_0_3] = 12;
	OperatingModeMap[VSP_0_25mph][VSP_3_6] = 13;
	OperatingModeMap[VSP_0_25mph][VSP_6_9] = 14;
	OperatingModeMap[VSP_0_25mph][VSP_9_12] = 15;
	OperatingModeMap[VSP_0_25mph][VSP_12_18] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_18_24] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_24_30] = 16;
	OperatingModeMap[VSP_0_25mph][VSP_GT30] = 16;

	OperatingModeMap[VSP_25_50mph][VSP_LT0] = 21;
	OperatingModeMap[VSP_25_50mph][VSP_0_3] = 22;
	OperatingModeMap[VSP_25_50mph][VSP_3_6] = 23;
	OperatingModeMap[VSP_25_50mph][VSP_6_9] = 24;
	OperatingModeMap[VSP_25_50mph][VSP_9_12] = 25;
	OperatingModeMap[VSP_25_50mph][VSP_12_18] = 27;
	OperatingModeMap[VSP_25_50mph][VSP_18_24] = 28;
	OperatingModeMap[VSP_25_50mph][VSP_24_30] = 29;
	OperatingModeMap[VSP_25_50mph][VSP_GT30] = 30;

	OperatingModeMap[VSP_GT50mph][VSP_LT0] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_0_3] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_3_6] = 33;
	OperatingModeMap[VSP_GT50mph][VSP_6_9] = 35;
	OperatingModeMap[VSP_GT50mph][VSP_9_12] = 35;
	OperatingModeMap[VSP_GT50mph][VSP_12_18] = 37;
	OperatingModeMap[VSP_GT50mph][VSP_18_24] = 38;
	OperatingModeMap[VSP_GT50mph][VSP_24_30] = 39;
	OperatingModeMap[VSP_GT50mph][VSP_GT30] = 40;
}

int GetOperatingMode(float vsp, float s_mph)
{
	SPEED_BIN SpeedBinNo = GetSpeedBinNo(s_mph);
	VSP_BIN VSPBinNo =  GetVSPBinNo(vsp, s_mph);

	int OPBin =  OperatingModeMap[SpeedBinNo][VSPBinNo];
	return OPBin;
}

int ComputeOperatingModeFromSpeed(float &vsp, float v /*meter per second*/,float a, float grade = 0, int vehicle_type =1)
{
	int vehicle_type_no = vehicle_type -1; // start from 0
	double TermA = g_VehicleTypeVector[vehicle_type_no].rollingTermA ;
	double TermB = g_VehicleTypeVector[vehicle_type_no].rotatingTermB ;
	double TermC = g_VehicleTypeVector[vehicle_type_no].dragTermC  ;
	double Mass =  g_VehicleTypeVector[vehicle_type_no].sourceMass;
	vsp = (TermA*v + TermB*v*v +  TermC*v*v*v + v*a*Mass)/Mass;
	//vsp = TermA/Mass*v + TermB/Mass*v*v+  TermC/Mass*v*v*v;
	float speed_mph = v * 2.23693629f;  //3600 seconds / 1606 meters per hour

	int OpMode = GetOperatingMode(vsp,speed_mph);

	return OpMode; 
}


bool vehicleCF_sort_function (VehicleCFData i,VehicleCFData j) 
{ 
	return (i.StartTime_in_SimulationInterval < j.StartTime_in_SimulationInterval );
}
bool matchMicroTrip(int vehicle_type, int start_time, float start_speed, float start_position, vector<float>& speedArray, vector<float>& position, 
					int arraySize, DrivingCycleIdentifier& matchedDrivingCycleIdentifier, int& exactSecond)
{
	//int speed_bin = GetDrivingCycleVehicleSpeedBin(start_speed);
	//TrajectoryMatrix& trajMatrix = trajectoryMatrixBin[speed_bin][vehicle_type];
	//bool isMatch = false;

	//int t = 0;
	//for (int n = start_time; n < arraySize && t < trajMatrix.timeWindowSize; n++,t++)
	//{
	//	int upperBound = min((int)(position[n] - start_position) + 10, trajMatrix.spaceWindowSize);
	//	int lowerBound = max(0, upperBound - 20);

	//	for (int m = lowerBound; m < upperBound; m++)
	//	{
	//		vector<DrivingCycleIdentifier>* pDrivingCycleIdentifierVector = trajMatrix.GetIdentifier(t, m);

	//		if (pDrivingCycleIdentifierVector && pDrivingCycleIdentifierVector->size() > 0)
	//		{
	//			float targetSpeed = speedArray[n+1];
	//			for (int k = 0; k < pDrivingCycleIdentifierVector->size();k++)
	//			{
	//				float speed = drivingCycleMap[vehicle_type][(*pDrivingCycleIdentifierVector)[k].cycle_id -1]->GetSpeedByTime((*pDrivingCycleIdentifierVector)[k].segment_id,n - start_time);
	//				if (abs(speed - targetSpeed) < 5)
	//				{
	//					matchedDrivingCycleIdentifier = (*pDrivingCycleIdentifierVector)[0];
	//					exactSecond = n - start_time;
	//					return true;
	//				}
	//			}
	//		}
	//	}

	//}

	//return false;
	///*for (int i=0;i<trajMatrix.timeWindowSize;i++)
	//{
	//	for (int j=0;j<trajMatrix.spaceWindowSize;j++)
	//	{

	//	}
	//}*/
	return true;
}
void ReconstructTrajectory(int vehicle_type, std::map<int, VehicleSpeedProfileData>& speedProfile)
{
	std::map<int, VehicleSpeedProfileData>::iterator iter = speedProfile.begin();
	int size = speedProfile.size();
	vector<float> speedArray;
	vector<float> position;
	position.push_back(0.0f);
	bool isUpdated = false;

	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		speedArray.push_back(iter->second.Speed);
		if (i == 0)
		{
			position[0] = 0.0f;
		}
		else
		{
			position.push_back(position[i-1] + iter->second.Speed * 0.44704f);
		}
	}

	vector<int> transitionPoints;
	for (int i=1;i<size;i++)
	{
		if (abs(speedArray[i] - speedArray[i-1]) > 10)
		{
			transitionPoints.push_back(i);
		}
	}

	if (transitionPoints.size() > 0)
	{
		for (int i=0;i<transitionPoints.size();i++)
		{
			int t = transitionPoints[i];
			int exactSecond;
			DrivingCycleIdentifier drivingCycleIdentifier;
			if (matchMicroTrip(vehicle_type,t,speedArray[t-1], position[t-1],speedArray,position, size, drivingCycleIdentifier, exactSecond))
			{
				std::cout << "==================Microtrip matched!====================\n";
				if (i != transitionPoints.size() - 1)
				{
					if (t + exactSecond < transitionPoints[i+1])
					{						
						for (int n=0;n<exactSecond;n++)
						{
							speedArray[i-1] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.cycle_id]->GetSpeedByTime(drivingCycleIdentifier.segment_id,n);
						}
						isUpdated = true;
					}
				}
				else
				{
					if (t + exactSecond < size)
					{
						for (int n=0;n<exactSecond;n++)
						{
							speedArray[i-1] = drivingCycleMap[vehicle_type][drivingCycleIdentifier.cycle_id]->GetSpeedByTime(drivingCycleIdentifier.segment_id,n);
						}

						isUpdated = true;
					}
				}
			}
			else
			{

			}
		}
	}

	if (isUpdated)
	{
		iter = speedProfile.begin();
		for (int i=0;iter != speedProfile.end();iter++,i++)
		{
			iter->second.Speed = speedArray[i];
		}
	}

}
enum TransitionType {ACC, DEC};
typedef struct
{
	int t;
	TransitionType type;

} TransitionPoint;


/* Acceleration and Deceleration Range
0-1			-2		0.6
1-5			-3.7	2.4
5-10		-5.1	5.6
10-20		-4.5	5
20-30		-3.7	3.6
30-40		-2.9	2.7
40-50		-1.6	1.7
50-60		-1		1
60-70		-1		1
70 and up	-1		1

*/

float maxAcceleration = (0.6f, 2.4f, 5.6f, 5.0f, 3.6f, 2.7f, 1.7f, 1.0f, 1.0f, 1.0f);
float maxDeceleration = (-2.0f, -3.7f, -5.1f, -4.5f, -3.7f, -2.9f, -1.6f, -1.0f, -1.0f, -1.0f);

float maxAccelerationBySpeed(float speedInMiles)
{
	if (speedInMiles < 1) return 0.6f;
	if (speedInMiles < 5) return 2.4f;
	if (speedInMiles < 10) return 5.6f;
	if (speedInMiles < 20) return 5.0f;
	if (speedInMiles < 30) return 3.6f;
	if (speedInMiles < 40) return 2.7f;
	if (speedInMiles < 50) return 1.7f;
	
	return 1.0f;
}

float maxDecelerationBySpeed(float speedInMiles)
{
	if (speedInMiles < 1) return -2.0f;
	if (speedInMiles < 5) return -3.7f;
	if (speedInMiles < 10) return -5.1f;
	if (speedInMiles < 20) return -4.5f;
	if (speedInMiles < 30) return -3.7f;
	if (speedInMiles < 40) return -2.9f;
	if (speedInMiles < 50) return -1.6f;
	
	return -1.0f;
}

void MovingAverage(std::vector<float>& p)
{
	size_t sz = p.size();

	if (sz >= 7)
	{
		p[0] = p[0];
		p[1] = (p[0] + p[1] + p[2]) / 3.0f;
		p[2] = (p[0] + p[1] + p[2] + p[3] + p[4]) / 5.0f;		

		for (int i = 3; i <= sz - 4; i++)
		{
			p[i] = (p[i - 3] + p[i - 2] + p[i - 1] + p[i] + p[i + 1] + p[i + 2] + p[i + 3]) / 7.0f;
		}

		p[sz - 3] = (p[sz - 5] + p[sz - 4] + p[sz - 3] + p[sz - 2] + p[sz - 1] ) / 5.0f;
		p[sz - 2] = (p[sz - 3] + p[sz - 2] + p[sz - 1]) / 3.0f;
		p[sz - 1] = p[sz - 1];
	}

	return;
}

void MovingAverage(float* p, int sz)
{
	if (p == NULL || sz == 0)
	{
		return;
	}

	if (sz >= 7)
	{
		p[0] = p[0];
		p[1] = (p[0] + p[1] + p[2]) / 3.0f;
		p[2] = (p[0] + p[1] + p[2] + p[3] + p[4]) / 5.0f;		

		for (int i = 3; i <= sz - 4; i++)
		{
			p[i] = (p[i - 3] + p[i - 2] + p[i - 1] + p[i] + p[i + 1] + p[i + 2] + p[i + 3]) / 7.0f;
		}

		p[sz - 3] = (p[sz - 5] + p[sz - 4] + p[sz - 3] + p[sz - 2] + p[sz - 1] ) / 5.0f;
		p[sz - 2] = (p[sz - 3] + p[sz - 2] + p[sz - 1]) / 3.0f;
		p[sz - 1] = p[sz - 1];
	}

	return;
}

vector<float> SpeedUpTo(int vehicle_id, float initial_speed, float targetSpeed)
{
	
	float speed = initial_speed;
	vector<float> speedVector;
	//if (initial_speed > targetSpeed) return speedVector;

	srand(vehicle_id);

	float currAccRate = maxAccelerationBySpeed(initial_speed) * (float)(rand() / double(RAND_MAX));

	while (speed + currAccRate < targetSpeed)
	{
		speed += currAccRate;
		speedVector.push_back(speed);
		currAccRate = maxAccelerationBySpeed(speed) * (float)(rand() / double(RAND_MAX));
	}
	speedVector.push_back(targetSpeed);
	MovingAverage(speedVector);
	return speedVector;
}

vector<float> SlowDownTo(int vehicle_id, float initial_speed, float targetSpeed)
{
	float speed = initial_speed;
	vector<float> speedVector;
	//if (initial_speed > targetSpeed) return speedVector;

	srand(vehicle_id);
	float maxDec = maxDecelerationBySpeed(initial_speed);
	float currDecRate = maxDec * (float)(rand() / double(RAND_MAX));

	while (speed + currDecRate > targetSpeed)
	{
		speed += currDecRate;
		speedVector.push_back(speed);
		maxDec = maxDecelerationBySpeed(initial_speed);
		currDecRate = maxDec * (float)(rand() / double(RAND_MAX));

	}
	speedVector.push_back(targetSpeed);
	MovingAverage(speedVector);
	return speedVector;
}




void SmoothVehicleTrajectory(int vehicle_id, std::map<int, VehicleSpeedProfileData>& speedProfile, int maxRun = 11)
{
	std::map<int, VehicleSpeedProfileData>::iterator iter = speedProfile.begin();
	int size = speedProfile.size();
	vector<float> speedArray;

	//speedArray.push_back(0.0f);
	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		speedArray.push_back(iter->second.Speed);
	}
	speedArray[speedArray.size() - 1] = 0.0f;

	//Find the transition points
	vector<TransitionPoint> transitionPoints;
	for (int i=1;i<size;i++)
	{
		if (speedArray[i] - speedArray[i-1] > 5.6 || speedArray[i] - speedArray[i-1] < -5.1)
		{
			if (i != size -1)
			{
				if (abs(speedArray[i] - speedArray[i-1]) == abs(speedArray[i] - speedArray[i+1]))
				{
					speedArray[i] = speedArray[i-1];
					i++;
					continue;
				}
			}

			TransitionPoint point;
			point.t = i;
			point.type = speedArray[i] - speedArray[i-1] > 0 ? ACC : DEC;
			transitionPoints.push_back(point);
		}
	}

	MovingAverage(speedArray);

	if (transitionPoints.size() <= 0) return;

	//Do the smoothing based on accelertion or deceleration
	for (int i=0;i<transitionPoints.size();i++)
	{
		TransitionPoint currPoint = transitionPoints[i];
		//if (i > 0)
		//{
		//	TransitionPoint prevPoint = transitionPoints[i-1];
		//	if (currPoint.t - prevPoint.t < 5)
		//	{
		//		continue;
		//	}
		//}
		float initialSpeed = speedArray[currPoint.t -1];
		float targetSpeed = speedArray[currPoint.t];
		vector<float> speedVector;
		if (currPoint.type == ACC)
		{
			speedVector = SpeedUpTo(vehicle_id, initialSpeed, targetSpeed);
			if (i != transitionPoints.size() - 1)
			{
				TransitionPoint nextPoint = transitionPoints[i+1];

				if (speedVector.size() > 0 /*&& nextPoint.t - currPoint.t > speedVector.size()*/)
				{
					if (currPoint.t >= speedVector.size())
					{
						for (int n = 0; n < speedVector.size(); n++)
						{
							speedArray[currPoint.t - speedVector.size() + 1 + n] = speedVector[n];
						}
					}
					else
					{
						for (int n = 0; n < speedVector.size(); n++)
						{
							speedArray[currPoint.t ] = speedVector[n];
						}

					}
				}
			}
		}
		else
		{
			speedVector = SlowDownTo(vehicle_id, initialSpeed, targetSpeed);
			if (speedVector.size() > 0 && currPoint.t >= speedVector.size())
			{
				for (int n = 0; n < speedVector.size(); n++)
				{
					speedArray[currPoint.t - speedVector.size() + 1 + n] = speedVector[n];
				}
			}
		}
		
	}

	MovingAverage(speedArray);

	iter = speedProfile.begin();


	for (int i=0;iter != speedProfile.end();iter++,i++)
	{
		iter->second.Speed = speedArray[i];
		if (i == 0)
		{
			iter->second.Acceleration = 0.0f;
		}
		else
		{
			iter->second.Acceleration = speedArray[i] - speedArray[i - 1];
			//if (iter->second.Acceleration > 0 && iter->second.Acceleration 
		}
	}

	if (maxRun > 0)
	{
		SmoothVehicleTrajectory(vehicle_id, speedProfile, --maxRun);
	}
		
}
void g_CalculateEmissionMOE()
{


	std::set<DTALink*>::iterator iterLink;
	int count = 0;
	// step 1: generate data for different lanes
	for (unsigned li = 0; li< g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		pLink->m_VehicleDataVector.clear();
		for (int LaneNo = 0; LaneNo < pLink->m_OutflowNumLanes; LaneNo++)
		{
			LaneVehicleCFData element(g_PlanningHorizon + 1);

			pLink->m_VehicleDataVector.push_back(element);
		}
	}

	// step 2: collect all vehicle passing each link

	int totalSamples = (int)(g_VehicleMap.size() * g_OutputSecondBySecondEmissionDataPercentage);
	int sampling_interval = (int)(g_VehicleMap.size() / max(1, totalSamples));
	int vehicleCounts = 0;

	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;

		pVehicle->m_OperatingModeCount.clear();

		if (pVehicle->m_NodeSize >= 2 && pVehicle->m_PricingType != 4)
		{
			for (int i = 0; i< pVehicle->m_NodeSize - 1; i++)
			{
				if (pVehicle->m_bComplete)  // for vehicles finish the trips
				{
					VehicleCFData element;
					element.VehicleID = pVehicle->m_VehicleID;
					int LinkNo = pVehicle->m_LinkAry[i].LinkNo;
					element.SequentialLinkNo = i;

					element.FreeflowDistance_per_SimulationInterval = g_LinkVector[LinkNo]->m_SpeedLimit* 1609.344f / 3600 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					//1609.344f: mile to meters ; 3600: # of seconds per hour
					//τ = 1/(wkj)
					// δ = 1/kj
					element.TimeLag_in_SimulationInterval = (int)(3600 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / (g_LinkVector[LinkNo]->m_BackwardWaveSpeed * g_LinkVector[LinkNo]->m_KJam) + 0.5);
					element.CriticalSpacing_in_meter = 1609.344f / g_LinkVector[LinkNo]->m_KJam;

					if (i == 0)
					{
						element.StartTime_in_SimulationInterval = pVehicle->m_DepartureTime * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					}
					else
					{
						element.StartTime_in_SimulationInterval = pVehicle->m_LinkAry[i - 1].AbsArrivalTimeOnDSN * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					}

					element.EndTime_in_SimulationInterval = pVehicle->m_LinkAry[i].AbsArrivalTimeOnDSN * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					element.LaneNo = element.VehicleID%g_LinkVector[LinkNo]->m_OutflowNumLanes;

					g_LinkVector[LinkNo]->m_VehicleDataVector[element.LaneNo].LaneData.push_back(element);
				}
			}

			if (pVehicle->m_bComplete)
			{
				vehicleCounts++;
				if (vehicleCounts % sampling_interval == 0)
				{
					pVehicle->m_bDetailedEmissionOutput = true;
				}
			}
			// for each vehicle
		}
	}


	// calclate link based VSP
	// step 2: collect all vehicle passing each link
	for (int li = 0; li< g_LinkVector.size(); li++)
	{
		TRACE("\n compute emissions for link %d", li);
		//		g_LogFile << "compute emissions for link " << li << endl;
		cout << "compute emissions for link " << li << endl;
		g_LinkVector[li]->ComputeVSP();
		//g_LinkVector[li]->ThreeDetectorVehicleTrajectorySimulation();
	}

	float Energy = 0;
	float CO2 = 0;
	float NOX = 0;
	float CO = 0;
	float HC = 0;

	CVehicleEmissionResult emissionResult;

	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;
		if (pVehicle->m_NodeSize >= 2 && pVehicle->m_OperatingModeCount.size() > 0)  // with physical path in the network
		{
			CVehicleEmissionResult::CalculateEmissions(pVehicle->m_VehicleType, pVehicle->m_OperatingModeCount, pVehicle->m_Age, emissionResult);

			pVehicle->m_Emissions = emissionResult.CO2;
			pVehicle->Energy = emissionResult.Energy;
			pVehicle->CO2 = emissionResult.CO2;
			pVehicle->NOX = emissionResult.NOX;
			pVehicle->CO = emissionResult.CO;
			pVehicle->HC = emissionResult.HC;

			Energy += emissionResult.Energy;
			CO2 += emissionResult.CO2;
			NOX += emissionResult.NOX;
			CO += emissionResult.CO;
			HC += emissionResult.HC;
		}
	}

	g_SimulationResult.Energy = Energy;
	g_SimulationResult.CO2 = CO2;
	g_SimulationResult.NOX = NOX;
	g_SimulationResult.CO = CO;
	g_SimulationResult.HC = HC;

	for (unsigned li = 0; li< g_LinkVector.size(); li++)
	{
		DTALink* pLink = g_LinkVector[li];

		int TimeSize = min(pLink->m_LinkMOEAry.size(), pLink->m_VehicleDataVector[0].m_LaneEmissionVector.size());
		for (int t = 0; t< TimeSize; t++)
		{
			for (int LaneNo = 0; LaneNo < pLink->m_OutflowNumLanes; LaneNo++)
			{
				pLink->m_LinkMOEAry[t].Energy += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].Energy;
				pLink->m_LinkMOEAry[t].CO2 += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO2;
				pLink->m_LinkMOEAry[t].NOX += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].NOX;
				pLink->m_LinkMOEAry[t].CO += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO;
				pLink->m_LinkMOEAry[t].HC += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].HC;

				pLink->TotalEnergy += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].Energy;
				pLink->TotalCO2 += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO2;
				pLink->TotalNOX += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].NOX;
				pLink->TotalCO += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].CO;
				pLink->TotalHC += pLink->m_VehicleDataVector[LaneNo].m_LaneEmissionVector[t].HC;
			}
		}
	}

}
void OutputEmissionData()
{

	//ReadDrivingCycleSamplesByVehicleType(1,"input_Cycle_PC_PT_LCT.csv",drivingCycleMap[1]);
	//ReadDrivingCycleSamplesByVehicleType(4,"input_Cycle_SST.csv",drivingCycleMap[4]);
	//ReadDrivingCycleSamplesByVehicleType(5,"input_CYCLE_LHT.csv",drivingCycleMap[5]);

	g_CalculateEmissionMOE();

		FILE* st2;
		fopen_s(&st2,"output_vehicle_operating_mode.csv","w");

		FILE* fpSampledEmissionsFile = NULL;
		fopen_s(&fpSampledEmissionsFile, "output_sampled_vehicle_operating_mode.csv", "w");

		FILE* fpSampledCycleEmissionRateFile = NULL;
		fopen_s(&fpSampledCycleEmissionRateFile, "output_sampled_vehicle_cycle_emission_rate.csv", "w");

		int numOfSamples = 0;
		//int numOfVehicles = 0;

			if (fpSampledEmissionsFile)
			{
				fprintf(fpSampledEmissionsFile, "Vehicle_id,Vehicle_Type,Vehicle_Age,Hour,Min,FromNodeNumber,ToNodeNumber,LinkType,TimeOnThisLinkInSecond,TripDurationInSecond,Speed(mph),Acceleration(mph/s),VSP,VSP_bin,Speed_bin,OperatingMode,Energy,CO2,NOX,CO,HC,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)\n");
			}

			if (fpSampledCycleEmissionRateFile)
			{
				fprintf(fpSampledCycleEmissionRateFile, "Vehicle_id,Vehicle_Type,Vehicle_Age,Average_Speed(MPH),CO2_Rate(g/mi),NOX_Rate(g/mi),CO_Rate((g/mi),HC_Rate(g/mi)\n");
			}

		if(st2 != NULL)
		{
			cout << "writing output_sampled_vehicle_operating_mode.csv..." << endl;
			std::map<int, DTAVehicle*>::iterator iterVM;
			//calculate the toll cost and emission cost
			for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
			{
				DTAVehicle* pVehicle = iterVM->second;

				if (pVehicle->m_bDetailedEmissionOutput == false)
					{
						continue;
					}

					numOfSamples++;

					if(numOfSamples % 500 == 0)
					{
						cout << " " << numOfSamples << " emission samples have been generated ..." << endl;
					}

					int VehicleSimulationStartTime;
					int prevFromNodeNumber = -1;
					int prevToNodeNumber = -1;
					int prevLinkType = -1;
					int currLinkType = -1;
					bool isNewLink = false;

					//ReconstructTrajectory(pVehicle->m_VehicleType, pVehicle->m_SpeedProfile);
					int speedProfileSize = pVehicle->m_SpeedProfile.size();
					int speedProfileCounter = 0;
					float prev_speed = 0.0f;

					float prev_acceleration = -99999.0f; //in meter/sec^2
					float prev_prev_acceleration = -99999.0f; //in meter/sec^2

					float totalCO2 = 0.0f;
					float totalNOX = 0.0f;
					float totalCO = 0.0f;
					float totalHC = 0.0f;

					float AverageSpeed = 0.0f;
					if (g_EmissionSmoothVehicleTrajectory)
					{
						SmoothVehicleTrajectory(pVehicle->m_VehicleID, pVehicle->m_SpeedProfile);
					}

					for(std::map<int, VehicleSpeedProfileData>::iterator iter_s  =  pVehicle->m_SpeedProfile.begin();
						iter_s != pVehicle->m_SpeedProfile.end();
						iter_s++, speedProfileCounter++)
					{  
						VehicleSpeedProfileData element = iter_s->second;

						if (fpSampledEmissionsFile)
						{
							//Get the link type by from and to node numbers
							if (prevFromNodeNumber != element.FromNodeNumber || prevToNodeNumber != element.ToNodeNumber)
							{
								if(g_LinkMap.find(GetLinkStringID(element.FromNodeNumber,element.ToNodeNumber)) != g_LinkMap.end())
								{
									DTALink* pLink = g_LinkMap[GetLinkStringID(element.FromNodeNumber,element.ToNodeNumber)];
									currLinkType = prevLinkType = pLink->m_link_type;
								}
								else
								{
									currLinkType = prevLinkType = -1;
								}
								//isNewLink = true;

								prevFromNodeNumber = element.FromNodeNumber;
								prevToNodeNumber = element.ToNodeNumber;
							}
							else
							{
								isNewLink = false;								
							}

							// If it is the first second on the first link or the last second on the last link, recalculate the acceleration rate, VSP and emissions
							//if (iter_s == pVehicle->m_SpeedProfile.begin() || speedProfileCounter + 1 == speedProfileSize) 
							//{
							//	isNewLink = false;

							VehicleSimulationStartTime = pVehicle->m_SpeedProfile.begin()->first;

							float acceleration = iter_s->second.Acceleration * 0.44704f;
								//if (iter_s == pVehicle->m_SpeedProfile.begin())
								//{
								//	acceleration = iter_s->second.Speed * 0.44704f; //mph to meter/sec^2
								//}

								//if (speedProfileCounter + 1 == speedProfileSize)
								//{
								//	acceleration = -iter_s->second.Speed * 0.44704f;
								//	element.Speed = iter_s->second.Speed = 0.0f;
								//}

								float vsp = 0;
								int OperatingMode = ComputeOperatingModeFromSpeed(vsp,iter_s->second.Speed * 0.44704f, acceleration,0,pVehicle->m_VehicleType);

								if (acceleration <= -2)
								{
									OperatingMode = 0;
								}
								else
								{
									if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
									{
										if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
										{
											OperatingMode = 0;
										}
									}
								}

								if (iter_s->second.Speed >= -1 && iter_s->second.Speed < 1)
								{
									OperatingMode = 1;
								}

								pVehicle->m_OperatingModeCount[element.OperationMode]--;
								pVehicle->m_OperatingModeCount[OperatingMode]++;							

								int vehicle_type = pVehicle->m_VehicleType;
								int age = pVehicle->m_Age;
								float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600;
								float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600;
								float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600;
								float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600;
								float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600;

								element.Acceleration = iter_s->second.Acceleration; //meter/sec^2 to feet/sec^2
								element.VSP = vsp;
								element.OperationMode = OperatingMode;
								element.VSPBinNo = GetVSPBinNo(vsp, element.Speed);
								element.SpeedBinNo = GetSpeedBinNo(element.Speed);
								element.Energy = Energy;
								element.CO2 = CO2;
								element.NOX = NOX;
								element.CO = CO;
								element.HC = HC;
							//}

							// First second on the new Link, recalculate the acceleration rate, VSP and emissions
							//if (isNewLink)
							//{
							//	std::map<int, VehicleSpeedProfileData>::iterator prev_iter_s = --iter_s;
							//	iter_s++;
							//	float acceleration = (iter_s->second.Speed - prev_speed) * 0.44704f;
							//	float vsp = 0;
							//	int OperatingMode = ComputeOperatingModeFromSpeed(vsp,iter_s->second.Speed * 0.44704f, acceleration,0,pVehicle->m_VehicleType);
							//	if (acceleration <= -2)
							//	{
							//		OperatingMode = 0;
							//	}
							//	else
							//	{
							//		if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
							//		{
							//			if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
							//			{
							//				OperatingMode = 0;
							//			}
							//		}
							//	}

							//	if (iter_s->second.Speed >= -1 && iter_s->second.Speed < 1)
							//	{
							//		OperatingMode = 1;
							//	}

							//	pVehicle->m_OperatingModeCount[element.OperationMode]--;
							//	pVehicle->m_OperatingModeCount[OperatingMode]++;							

							//	int vehicle_type = pVehicle->m_VehicleType;
							//	int age = pVehicle->m_Age;
							//	float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600;
							//	float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600;
							//	float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600;
							//	float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600;
							//	float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600;

							//	element.Acceleration = acceleration * 3.28084f; // meter/sec^2 to feet/sec^2
							//	element.VSP = vsp;
							//	element.OperationMode = OperatingMode;
							//	element.VSPBinNo = GetVSPBinNo(vsp, element.Speed);
							//	element.SpeedBinNo = GetSpeedBinNo(element.Speed);
							//	element.Energy = Energy;
							//	element.CO2 = CO2;
							//	element.NOX = NOX;
							//	element.CO = CO;
							//	element.HC = HC;
							//}

							fprintf(fpSampledEmissionsFile, "%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%5.2f",
								pVehicle->m_VehicleID,
								pVehicle->m_VehicleType,
								pVehicle->m_Age,
								(int)(iter_s->first / 3600 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),
								((int)(iter_s->first / 60 / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)) % 60,								
								element.FromNodeNumber,
								element.ToNodeNumber,
								(currLinkType == -1) ? "Unknown":g_LinkTypeMap[currLinkType].link_type_name.c_str(),
								element.TimeOnThisLinkInSecond,
								(int)((iter_s->first - VehicleSimulationStartTime) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND),
								element.Speed
								);

							if (iter_s != pVehicle->m_SpeedProfile.begin())
							{
								totalCO2 += element.CO2;
								totalNOX += element.NOX;
								totalCO += element.CO;
								totalHC += element.HC;

								fprintf(fpSampledEmissionsFile,",%5.2f,%f,%s,%s,%d,%f,%f,%f,%f,%f",
									element.Acceleration,
									element.VSP,
									element.GetVSPBinNoString(),
									element.GetSpeedBinNoString(),
									element.OperationMode,
									element.Energy,
									element.CO2, 
									element.NOX, 
									element.CO,
									element.HC
									);
							}

							AverageSpeed += element.Speed;

							if (speedProfileCounter + 1 == speedProfileSize)
							{
								fprintf(fpSampledCycleEmissionRateFile, "%d,%d,%d", 
									pVehicle->m_VehicleID,
									pVehicle->m_VehicleType,
									pVehicle->m_Age
									);

								float average_speed =  AverageSpeed / speedProfileSize /*pVehicle->m_Distance / (pVehicle->m_ArrivalTime - pVehicle->m_DepartureTime) * 60.0f*/;
								float base_average_speed;
								//Average speed for base cycle is 21.2 mph
								//CCycleAverageEmissionFactor* pBaseCaseEmissionsFactors = BaseCaseEmissionFactors(pVehicle->m_VehicleType, pVehicle->m_Age,21.2f);
								//if (pBaseCaseEmissionsFactors)
								//{
								//	float speed_ratio = 21.2 / average_speed;									

								//	float correctedCO2 = pBaseCaseEmissionsFactors->emissionFactor_CO2 * (totalCO2 / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_CO2] * speed_ratio;
								//	float correctedNOX = pBaseCaseEmissionsFactors->emissionFactor_NOX * (totalNOX / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_NOX] * speed_ratio;
								//	float correctedCO = pBaseCaseEmissionsFactors->emissionFactor_CO * (totalCO / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_CO] * speed_ratio;
								//	float correctedHC = pBaseCaseEmissionsFactors->emissionFactor_HC * (totalHC / speedProfileSize * 3600) / BaseCycleEmissionRate[pVehicle->m_VehicleType][pVehicle->m_Age][EMISSION_HC] * speed_ratio;

								//	fprintf(fpSampledEmissionsFile,",%f,%f,%f,%f,%f", average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);
								//	fprintf(fpSampledCycleEmissionRateFile,",%f,%f,%f,%f,%f", average_speed, correctedCO2, correctedNOX, correctedCO, correctedHC);
								//}
								//else
								{
									fprintf(fpSampledEmissionsFile,",%f,%f,%f,%f,%f", average_speed, totalCO2 / pVehicle->m_Distance, totalNOX / pVehicle->m_Distance, totalCO / pVehicle->m_Distance, totalHC / pVehicle->m_Distance);
									fprintf(fpSampledCycleEmissionRateFile,",%f,%f,%f,%f,%f", average_speed, totalCO2 / pVehicle->m_Distance, totalNOX / pVehicle->m_Distance, totalCO / pVehicle->m_Distance, totalHC / pVehicle->m_Distance);
								}

								fprintf(fpSampledCycleEmissionRateFile,"\n");
							}
							fprintf(fpSampledEmissionsFile,"\n");

							prev_speed = element.Speed;
							prev_prev_acceleration = prev_acceleration;
							prev_acceleration = element.Acceleration / 3.28084f;
						}
					}
				

				fprintf(st2, "Vehicle=,%d,Type=,%d\n", pVehicle->m_VehicleID,pVehicle->m_VehicleType);
				fprintf(st2, "# of operating mode data points =,%d\n", pVehicle->m_OperatingModeCount.size());
				fprintf(st2, "Energy:, %f\n", pVehicle->Energy);
				fprintf(st2, "CO2:, %f\n", pVehicle->CO2 );
				fprintf(st2, "CO:, %f\n", pVehicle->CO );
				fprintf(st2, "HC:, %f\n", pVehicle->HC );
				fprintf(st2, "NOX:, %f\n", pVehicle->NOX);

				for(std::map<int, int>::iterator iterOP  =  pVehicle->m_OperatingModeCount.begin(); iterOP != pVehicle->m_OperatingModeCount.end (); iterOP++)
				{
					fprintf(st2, "op=,%d,count=,%d\n", iterOP->first ,iterOP->second );
				}

				for(std::map<int, int>::iterator iter_speed  =  pVehicle->m_SpeedCount.begin(); iter_speed != pVehicle->m_SpeedCount.end (); iter_speed++)
				{
					fprintf(st2, "speed=,%d,count=,%d\n", iter_speed->first ,iter_speed->second );
				}

				fprintf(st2,"\n");
			}

			fclose(fpSampledEmissionsFile);
			fclose(fpSampledCycleEmissionRateFile);
			fclose(st2);
			cout << numOfSamples << " vehicles are sampled to output_sampled_vehicle_operating_mode.csv and output_sampled_vehicle_cycle_emission_rate.csv." << endl;

		}



}

//void SmoothCumulativeCounts(float* countArray, int size)
//{
//    if (countArray != NULL && size > 0)
//    {
//        float value = countArray[0];
//        int start_idx = 0;
//        int end_index = 0;
//        for (int n = 1; n < size; n++)
//        {
//            if (countArray[n] == value)
//            {
//                countArray[n] = 0;
//                end_index++;
//            }
//            else
//            {
//                if (start_idx < end_index)
//                {
//                    float increment = (countArray[n] - value) / (end_index - start_idx + 1);
//                    for (int i = start_idx + 1; i <= end_index; i++)
//                    {
//                        countArray[i] = countArray[i - 1] + increment;
//                    }
//                }
//                value = countArray[n];
//                start_idx = end_index = n;
//            }
//        }
//    }
//}

//void DTALink::ThreeDetectorVehicleTrajectorySimulation()
//{
//	int initialTime = -1;
//	int endTime = -1;
//
//
//	for (int LaneNo = 0; LaneNo < m_OutflowNumLanes; LaneNo++)
//	{
//		sort(m_VehicleDataVector[LaneNo].LaneData.begin(), m_VehicleDataVector[LaneNo].LaneData.end(), vehicleCF_sort_function);
//		initialTime = m_VehicleDataVector[LaneNo].LaneData[0].StartTime_in_SimulationInterval;
//		endTime = m_VehicleDataVector[LaneNo].LaneData[m_VehicleDataVector[LaneNo].LaneData.size() -1].EndTime_in_SimulationInterval;
//		int countArraySize = endTime - initialTime + 1;
//		float* downstreamLaneCounts = new float[countArraySize];
//		memset(downstreamLaneCounts, 0, sizeof(float) * countArraySize);
//
//		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
//		{
//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;
//			downstreamLaneCounts[end_time - initialTime]++;
//		}
//
//		for (int i=1; i < countArraySize; i++)
//		{
//			downstreamLaneCounts[i] += downstreamLaneCounts[i -1];
//		}
//
//		SmoothCumulativeCounts(downstreamLaneCounts, countArraySize);
//
//
//		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
//		{
//			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
//			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;
//			
//			float link_length_in_meter = m_Length * 1609.344f;
//			float position = 0.0f;
//			int upstreamCountNum = v;
//
//			DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];
//			pVehicle->m_PrevSpeed = 0.0f;
//			float prevPosition = 0.0f;
//			float prev_acceleration = -99999.0f; //acceleration at t-1
//			float prev_prev_acceleration = -99999.0f; //acceleration at t-2
//
//			for(int t = 1; t < end_time - start_time; t+= 1) 				
//			{
//				if (position < link_length_in_meter)
//				{
//					int bwtt = (int)((link_length_in_meter - position) / (this->m_BackwardWaveSpeed * 1609.34f / 3600.0f)) * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
//					int t_at_downstream = t - bwtt;
//					if (t_at_downstream < 0)
//					{
//						position = min(link_length_in_meter, position + this->m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);						
//					}
//					else
//					{
//						if (upstreamCountNum <= downstreamLaneCounts[t_at_downstream] + this->m_KJam * (link_length_in_meter - position) * 0.000621371f)
//						{
//							position = min(link_length_in_meter, position + this->m_SpeedLimit * 1609.34f / 3600.0f / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND);							
//						}
//					}
//				}
//				else
//				{
//					break;
//				}
//
//				if (t % NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND == 0)
//				{
//
//					float SpeedBySecond = position - prevPosition;					
//					float acceleration = SpeedBySecond - pVehicle->m_PrevSpeed;
//
//					float vsp = 0;
//					int OperatingMode = ComputeOperatingModeFromSpeed(vsp,SpeedBySecond, acceleration,0,pVehicle->m_VehicleType);
//
//					if (acceleration <= -2)
//					{
//						OperatingMode = 0;
//					}
//					else
//					{
//						if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
//						{
//							if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
//							{
//								OperatingMode = 0;
//							}
//						}
//					}
//
//					if ((SpeedBySecond / 0.44704f) >= -1.0f && (SpeedBySecond / 0.44704f) < 1.0f)
//					{
//						OperatingMode = 1;
//					}
//
//					pVehicle->m_OperatingModeCount[OperatingMode]++;
//					int integer_speed = SpeedBySecond / 0.44704f;  // convert meter per second to mile per hour
//					pVehicle->m_SpeedCount[integer_speed]++;
//
//
//					pVehicle->m_PrevSpeed  = SpeedBySecond;
//
//					int vehicle_type = pVehicle->m_VehicleType;
//
//					int age = pVehicle->m_Age ;
//
//					int time_in_min = t / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / 60;
//
//					if(EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
//					{
//						cout << "Emission rate data are not available for vehicle type = " <<  vehicle_type 
//							<< ", operating mode = " << OperatingMode 
//							<< " and age = " << age << endl;
//						g_ProgramStop();
//					}
//
//					float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy/3600.0f;
//					float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2/3600.0f;
//					float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX/3600.0f;
//					float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO/3600.0f;
//					float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC/3600.0f;
//
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].Energy+= Energy;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO2+= CO2;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].NOX+= NOX;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO+= CO;
//					m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].HC+= HC;
//
//					if (pVehicle->m_VehicleID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || 
//						(g_OutputSecondBySecondEmissionData && pVehicle->m_DepartureTime >= g_start_departure_time_in_min_for_output_second_by_second_emission_data 
//						&& pVehicle->m_DepartureTime <= g_end_departure_time_in_min_for_output_second_by_second_emission_data
//						&& pVehicle->m_bDetailedEmissionOutput))  // record data
//					{
//						VehicleSpeedProfileData element;
//						element.FromNodeNumber = this->m_FromNodeNumber ;
//						element.ToNodeNumber  = this->m_ToNodeNumber ;
//						element.TimeOnThisLinkInSecond = (t-start_time)/NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
//						element.Speed = SpeedBySecond / 0.44704f; // km/h to mph
//						element.Acceleration = acceleration * 3.28084; //meter/sec^2 to feet/sec^2
//						element.OperationMode = OperatingMode;
//						element.VSP  = vsp;
//						element.SpeedBinNo = GetSpeedBinNo(element.Speed);
//						element.VSPBinNo =  GetVSPBinNo(vsp, element.Speed);
//						element.Energy = Energy;
//						element.CO2 = CO2;
//						element.NOX = NOX;
//						element.CO = CO;
//						element.HC = HC;
//
//						pVehicle->m_SpeedProfile[t] = element;
//					}
//
//					prevPosition = position;
//					prev_prev_acceleration = prev_acceleration;
//					prev_acceleration = acceleration * 0.44704f;
//				}				
//			}
//		}
//
//
//		delete [] downstreamLaneCounts;
//	}
//}
//

void DTALink::ComputeVSP()  // VSP: vehicle specific power
{

	const int Max_Time_Step_In_A_Cycle = 100;
	const int Max_Simulation_Steps = 1800 * 60 * NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
	static float positionBuffer1[Max_Simulation_Steps];
	static float positionsBuffer2[Max_Simulation_Steps];	
	float *pLeaderPositions = positionBuffer1;
	float *pFollowerPositions = positionsBuffer2;

	//FILE* output = NULL;

	//if (this->m_FromNodeNumber == 2)
	//{
	//	fopen_s(&output,"trajectory.csv","w");
	//}

	#pragma omp parallel for 
	for(int LaneNo = 0; LaneNo < m_OutflowNumLanes; LaneNo++)
	{
		// step 1: sort vehicles arrival times
		sort(m_VehicleDataVector[LaneNo].LaneData.begin(), m_VehicleDataVector[LaneNo].LaneData.end(), vehicleCF_sort_function);

		// step 2: car following simulation
		float link_length_in_meter = m_Length * 1609.344f; //1609.344f: mile to meters

		int leaderInTime = -1;
		int leaderOutTime = -1;

		for(int v = 0; v < m_VehicleDataVector[LaneNo].LaneData.size(); v++)  // first do loop: every vehicle
		{
			int start_time =  m_VehicleDataVector[LaneNo].LaneData[v].StartTime_in_SimulationInterval;
			int end_time =  m_VehicleDataVector[LaneNo].LaneData[v].EndTime_in_SimulationInterval;

			// second do loop: start_time to end time
			for(int t = start_time; t <= end_time; t+=1) 				
			{
				//calculate free-flow position
				//xiF(t) = xi(t-τ) + vf(τ)
				pFollowerPositions[t - start_time] = 0.0f;

				if(t > start_time)
				{
					if (v == 0)
					{
						pFollowerPositions[t - start_time] = min(link_length_in_meter, pFollowerPositions[t - start_time - 1] +  m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval);
					}
					else
					{
						//calculate congested position if it is not the first vehicle
						//xiC(t) = xi-1(t-τ) - δ
						int time_t_minus_tau = t - m_VehicleDataVector[LaneNo].LaneData[v].TimeLag_in_SimulationInterval; // need to convert time in second to time in simulation time interval
						if (time_t_minus_tau < leaderInTime)
						{
							pFollowerPositions[t - start_time] = 0.0f;
						}
						else
						{
							if(time_t_minus_tau >= 0 && time_t_minus_tau <= leaderOutTime)  // the leader has not reached destination yet
							{
								// vehicle v-1: previous car
								//float CongestedDistance = VechileDistanceAry[v-1][time_t_minus_tau%Max_Time_Step_In_A_Cycle] - m_VehicleDataVector[LaneNo].LaneData[v].CriticalSpacing_in_meter;
								float CongestedDistance = max(0, pLeaderPositions[time_t_minus_tau - leaderInTime] - m_VehicleDataVector[LaneNo].LaneData[v].CriticalSpacing_in_meter);
								float potentialDistance = pFollowerPositions[t - start_time - 1] + m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval;
								// xi(t) = min(xAF(t), xAC(t))
								//if (FollowerPositions[t % Max_Simulation_Steps] > CongestedDistance /*&& CongestedDistance >= FollowerPositions[(t-1) % Max_Simulation_Steps]*/)
								//{
								//	FollowerPositions[t % Max_Simulation_Steps] = min(CongestedDistance, FollowerPositions[(t-1) % Max_Simulation_Steps]);
								//}

								pFollowerPositions[t - start_time] = min(CongestedDistance, potentialDistance);
								pFollowerPositions[t - start_time] = min(pFollowerPositions[t - start_time], link_length_in_meter);
							}
							else
							{
								pFollowerPositions[t - start_time] = min(link_length_in_meter, pFollowerPositions[t - start_time - 1] +  m_VehicleDataVector[LaneNo].LaneData[v].FreeflowDistance_per_SimulationInterval);
							}
						}

						if (t == end_time)
						{
							pFollowerPositions[t - start_time] = link_length_in_meter;
						}
					}
				}

				//if (v >= 0 && v <= 400 && LaneNo == 0)
				//{
				//	DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];

				//	if (output != NULL)
				//	{
				//		if (t != start_time)
				//		{
				//			fprintf(output,"%d, %d, %d, %d, %5.2f,%5.2f\n",v, pVehicle->m_VehicleID, LaneNo, t, pFollowerPositions[t - start_time], pFollowerPositions[t - start_time] - pFollowerPositions[t- start_time - 1]);
				//		}
				//		else
				//		{
				//			fprintf(output,"%d, %d, %d, %d,%5.2f,%5.2f\n",v, pVehicle->m_VehicleID, LaneNo, t, pFollowerPositions[t - start_time], 0.0f);
				//		}
				//	}
				//}
			}  // for each time step

			if (g_EmissionSmoothVehicleTrajectory)
			{
				MovingAverage(pFollowerPositions, end_time - start_time + 1);
			}

			float prev_acceleration = -99999.0f; //acceleration at t-1
			float prev_prev_acceleration = -99999.0f; //acceleration at t-2
			for (int t = start_time; t <= end_time; t += NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)
			{
				// output speed per second
				// for active vehicles (with positive speed or positive distance
				float SpeedBySecond = m_SpeedLimit * 0.44704f; // 1 mph = 0.44704 meters per second

				if (t == start_time) SpeedBySecond = 0.0f;

				if(t >= start_time + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)  // not the first second
				{
					//SpeedBySecond = (VechileDistanceAry[v][t%Max_Time_Step_In_A_Cycle] - VechileDistanceAry[v][max(0,t-NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND)%Max_Time_Step_In_A_Cycle]);						
					SpeedBySecond = pFollowerPositions[t - start_time] - pFollowerPositions[t - start_time - NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND];
				}

				if (t + NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND >= end_time)
				{
					SpeedBySecond = m_SpeedLimit * 0.44704f;
				}

				// different lanes have different vehicle numbers, so we should not have openMP conflicts here
				DTAVehicle* pVehicle  = g_VehicleMap[m_VehicleDataVector[LaneNo].LaneData[v].VehicleID];
				float acceleration = SpeedBySecond - pVehicle->m_PrevSpeed;

				float vsp = 0;
				int OperatingMode = ComputeOperatingModeFromSpeed(vsp,SpeedBySecond, acceleration,0,pVehicle->m_VehicleType);

				if (acceleration <= -2)
				{
					OperatingMode = 0;
				}
				else
				{
					if (prev_prev_acceleration < -1 && prev_acceleration < -1 && acceleration < -1)
					{
						if (prev_prev_acceleration != -99999.0f || prev_acceleration != -99999.0f)
						{
							OperatingMode = 0;
						}
					}
				}

				if ((SpeedBySecond / 0.44704f) >= -1.0f && (SpeedBySecond / 0.44704f) < 1.0f)
				{
					OperatingMode = 1;
				}

				pVehicle->m_OperatingModeCount[OperatingMode]++;
				int integer_speed = SpeedBySecond / 0.44704f;  // convert meter per second to mile per hour
				pVehicle->m_SpeedCount[integer_speed]++;


				pVehicle->m_PrevSpeed  = SpeedBySecond;

				int vehicle_type = pVehicle->m_VehicleType;

				int age = pVehicle->m_Age ;

				int time_in_min = start_time / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND / 60;

				if (EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
				{
					// try without data
					age = 5 * int(age*1.0 / 5 + 0.5);

				}


				if(EmissionRateData[vehicle_type][OperatingMode][age].bInitialized == false)
				{
					cout << "Emission rate data are not available for vehicle type = " <<  vehicle_type 
						<< ", operating mode = " << OperatingMode 
						<< " and age = " << age << endl;
					g_ProgramStop();
				}

				float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy / 3600.0f;
				float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2 / 3600.0f;
				float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX / 3600.0f;
				float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO / 3600.0f;
				float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC / 3600.0f;

					if(time_in_min ==430 && m_FromNodeNumber == 1 && m_ToNodeNumber == 3)
					{
					TRACE("");
					
					}
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].Energy += Energy;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO2 += CO2;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].NOX += NOX;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].CO += CO;
				m_VehicleDataVector[LaneNo].m_LaneEmissionVector [time_in_min].HC += HC;

				//if (pVehicle->m_VehicleID  ==  g_TargetVehicleID_OutputSecondBySecondEmissionData || 
				//	(g_OutputSecondBySecondEmissionData && pVehicle->m_DepartureTime >= g_start_departure_time_in_min_for_output_second_by_second_emission_data 
				//	&& pVehicle->m_DepartureTime <= g_end_departure_time_in_min_for_output_second_by_second_emission_data
				//	&& pVehicle->m_bDetailedEmissionOutput))  // record data
				if (pVehicle->m_VehicleID == g_TargetVehicleID_OutputSecondBySecondEmissionData )				{
					VehicleSpeedProfileData element;
					element.FromNodeNumber = this->m_FromNodeNumber ;
					element.ToNodeNumber  = this->m_ToNodeNumber ;
					element.TimeOnThisLinkInSecond = (t - start_time) / NUMBER_OF_CAR_FOLLOWING_SIMULATION_INTERVALS_PER_SECOND;
					element.Speed = SpeedBySecond / 0.44704f; // km/h to mph
					element.Acceleration = acceleration * 3.28084; //meter/sec^2 to feet/sec^2
					element.OperationMode = OperatingMode;
					element.VSP  = vsp;
					element.SpeedBinNo = GetSpeedBinNo(element.Speed);
					element.VSPBinNo =  GetVSPBinNo(vsp, element.Speed);
					element.Energy = Energy;
					element.CO2 = CO2;
					element.NOX = NOX;
					element.CO = CO;
					element.HC = HC;

					pVehicle->m_SpeedProfile[t] = element;
				}

				prev_prev_acceleration = prev_acceleration;
				prev_acceleration = acceleration * 0.44704f;				
			}

			float *tmp = pLeaderPositions;
			pLeaderPositions = pFollowerPositions;
			pFollowerPositions = tmp;
			leaderInTime = start_time;
			leaderOutTime = end_time;
		} // for each vehicle
	} //for each lane

	//if (output != NULL)
	//{
	//	fclose(output);
	//}

	//delete [] LeaderPositions;
	//delete [] FollowerPositions;

}
