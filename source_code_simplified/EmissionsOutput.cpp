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

#define _MAXIMUM_OPERATING_MODE_SIZE 41
#define _MAXIMUM_AGE_SIZE 31

int OperatingModeMap[MAX_SPEED_BIN][MAX_VSP_BIN] = { -1 };


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

		for (std::map<int, int>::iterator iterOP = OperatingModeCount.begin(); iterOP != OperatingModeCount.end(); iterOP++)
		{
			int OpModeID = iterOP->first; int count = iterOP->second;

			emissionResult.Energy += EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_TotalEnergy * count / 3600;
			emissionResult.CO2 += EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO2 * count / 3600;
			emissionResult.NOX += EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_NOX * count / 3600;
			emissionResult.CO += EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_CO * count / 3600;
			emissionResult.HC += EmissionRateData[vehicle_type][OpModeID][age].meanBaseRate_HC * count / 3600;
		}
	}
};

SPEED_BIN GetSpeedBinNo(float speed_mph)
{
	//the precision of speed output is 2
	//if the speed is 49.995, it would round up to 50
	//if the speed is 49.994, it would round to 49.99
	int speed = floor(speed_mph + 0.005);

	if (speed <= 25)
		return VSP_0_25mph;

	if (speed < 50)
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

	int line = 1;
	if (parser_emission.OpenCSVFile("input_vehicle_emission_rate.csv"))
	{

		while (parser_emission.ReadRecord())
		{
			line++;

			if (line == 256)
				TRACE("");


			int vehicle_type;
			int opModeID;

			if (parser_emission.GetValueByFieldName("vehicle_type", vehicle_type) == false)
				break;
			if (parser_emission.GetValueByFieldName("OpModeID", opModeID) == false)
				break;

			CEmissionRate element;
			if (parser_emission.GetValueByFieldName("meanBaseRate_TotalEnergy_(KJ/hr)", element.meanBaseRate_TotalEnergy) == false)
				break;
			if (parser_emission.GetValueByFieldName("meanBaseRate_CO2_(g/hr)", element.meanBaseRate_CO2) == false)
				break;
			if (parser_emission.GetValueByFieldName("meanBaseRate_NOX_(g/hr)", element.meanBaseRate_NOX) == false)
				break;
			if (parser_emission.GetValueByFieldName("meanBaseRate_CO_(g/hr)", element.meanBaseRate_CO) == false)
				break;
			if (parser_emission.GetValueByFieldName("meanBaseRate_HC_(g/hr)", element.meanBaseRate_HC) == false)
				break;


			if (parser_emission.GetValueByFieldName("age", element.Age) == false)
			{
				break;
			}


			if (element.Age < _MAXIMUM_AGE_SIZE && opModeID < _MAXIMUM_OPERATING_MODE_SIZE)
			{

				EmissionRateData[vehicle_type][opModeID][element.Age] = element;
				EmissionRateData[vehicle_type][opModeID][element.Age].bInitialized = true;
			}
			else
			{
				cout << "Reading error at line " << line << " at input_vehicle_emission_rate.csv" << endl;
				g_ProgramStop();

			}

		}
	}
	else
	{
		cout << "Error: File input_vehicle_emission_rate.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		g_ProgramStop();

	}

	cout << "Read " << line << " lines from file input_vehicle_emission_rate.csv." << endl;
}

enum Pollutants { EMISSION_CO2, EMISSION_NOX, EMISSION_CO, EMISSION_HC, MAX_POLLUTANT };


typedef struct
{
	int second;
	float speed;
	float position;
} SecondData;


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
	VSP_BIN VSPBinNo = GetVSPBinNo(vsp, s_mph);

	int OPBin = OperatingModeMap[SpeedBinNo][VSPBinNo];
	return OPBin;
}

int ComputeOperatingModeFromSpeed(float& vsp, float v /*meter per second*/, float a, float grade = 0, int vehicle_type = 1)
{
	int vehicle_type_no = vehicle_type - 1; // start from 0
	double TermA = g_VehicleTypeVector[vehicle_type_no].rollingTermA;
	double TermB = g_VehicleTypeVector[vehicle_type_no].rotatingTermB;
	double TermC = g_VehicleTypeVector[vehicle_type_no].dragTermC;
	double Mass = g_VehicleTypeVector[vehicle_type_no].sourceMass;
	vsp = (TermA * v + TermB * v * v + TermC * v * v * v + v * a * Mass) / Mass;
	//vsp = TermA/Mass*v + TermB/Mass*v*v+  TermC/Mass*v*v*v;
	float speed_mph = v * 2.23693629f;  //3600 seconds / 1606 meters per hour

	int OpMode = GetOperatingMode(vsp, speed_mph);

	return OpMode;
}


void g_CalculateEmissionMOE()
{

	

	int vehicleCounts = 0;


	float Energy = 0;
	float CO2 = 0;
	float NOX = 0;
	float CO = 0;
	float HC = 0;

	CVehicleEmissionResult emissionResult;
	std::map<int, DTAVehicle*>::iterator iterVM;
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



}
void OutputEmissionData()
{

	g_CalculateEmissionMOE();

	FILE* st2;
	fopen_s(&st2, "output_vehicle_operating_mode.csv", "w");

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

	if (st2 != NULL)
	{
		cout << "writing output_sampled_vehicle_operating_mode.csv..." << endl;
		std::map<int, DTAVehicle*>::iterator iterVM;
		//calculate the toll cost and emission cost
		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
		{
			DTAVehicle* pVehicle = iterVM->second;


			numOfSamples++;

			if (numOfSamples % 500 == 0)
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


			for (std::map<int, VehicleSpeedProfileData>::iterator iter_s = pVehicle->m_SpeedProfile.begin();
				iter_s != pVehicle->m_SpeedProfile.end();
				iter_s++, speedProfileCounter++)
			{
				VehicleSpeedProfileData element = iter_s->second;


				float acceleration = iter_s->second.Acceleration * 0.44704f;

				float vsp = 0;
				int OperatingMode = ComputeOperatingModeFromSpeed(vsp, iter_s->second.Speed * 0.44704f, acceleration, 0, pVehicle->m_VehicleType);

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
				float Energy = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_TotalEnergy / 3600;
				float CO2 = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO2 / 3600;
				float NOX = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_NOX / 3600;
				float CO = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_CO / 3600;
				float HC = EmissionRateData[vehicle_type][OperatingMode][age].meanBaseRate_HC / 3600;

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

				

				if (iter_s != pVehicle->m_SpeedProfile.begin())
				{
					totalCO2 += element.CO2;
					totalNOX += element.NOX;
					totalCO += element.CO;
					totalHC += element.HC;

					fprintf(fpSampledEmissionsFile, ",%5.2f,%f,%s,%s,%d,%f,%f,%f,%f,%f",
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

			}
		}

		//for (std::map<int, int>::iterator iterOP = pVehicle->m_OperatingModeCount.begin(); iterOP != pVehicle->m_OperatingModeCount.end(); iterOP++)
		//{
		//	fprintf(st2, "op=,%d,count=,%d\n", iterOP->first, iterOP->second);
		//}

		//for (std::map<int, int>::iterator iter_speed = pVehicle->m_SpeedCount.begin(); iter_speed != pVehicle->m_SpeedCount.end(); iter_speed++)
		//{
		//	fprintf(st2, "speed=,%d,count=,%d\n", iter_speed->first, iter_speed->second);
		//}

		fprintf(st2, "\n");
	}

	fclose(fpSampledEmissionsFile);
	fclose(fpSampledCycleEmissionRateFile);
	fclose(st2);
	cout << numOfSamples << " vehicles are sampled to output_sampled_vehicle_operating_mode.csv and output_sampled_vehicle_cycle_emission_rate.csv." << endl;

}

void ReadInputVehicleTypeFile()
{
	cout << "Step 1: Reading file input_vehicle_type.csv..." << endl;
	

	CCSVParser parser_vehicle_type;

	if (!parser_vehicle_type.OpenCSVFile("input_vehicle_type.csv"))
	{
		cout << "input_vehicle_type.csv cannot be opened.  Use default values. " << endl;

		ofstream VehicleTypeFile;
		VehicleTypeFile.open("input_vehicle_type.csv");
		if (VehicleTypeFile.is_open())
		{
			VehicleTypeFile << "vehicle_type,vehicle_type_name,percentage_of_age_0,percentage_of_age_5,percentage_of_age_10,percentage_of_age_15" << endl;
			VehicleTypeFile << "1,passenger car,25,25,25,25" << endl;
			VehicleTypeFile << "2,passenger truck,25,25,25,25" << endl;
			VehicleTypeFile << "3,light commercial truck,25,25,25,25" << endl;
			VehicleTypeFile << "4,single unit short-haul truck,25,25,25,25" << endl;
			VehicleTypeFile << "5,combination long-haul truck,25,25,25,25" << endl;
			VehicleTypeFile.close();
		}
	}

	if (parser_vehicle_type.inFile.is_open() || parser_vehicle_type.OpenCSVFile("input_vehicle_type.csv"))
	{
		g_VehicleTypeVector.clear();
		while (parser_vehicle_type.ReadRecord())
		{
			int vehicle_type = 0;
			if (parser_vehicle_type.GetValueByFieldName("vehicle_type", vehicle_type) == false)
				break;

			string vehicle_type_name;
			parser_vehicle_type.GetValueByFieldName("vehicle_type_name", vehicle_type_name);




			DTAVehicleType element;
			element.vehicle_type = vehicle_type;
			element.vehicle_type_name = vehicle_type_name;

			parser_vehicle_type.GetValueByFieldName("rolling_term_a", element.rollingTermA);
			parser_vehicle_type.GetValueByFieldName("rotating_term_b", element.rotatingTermB);
			parser_vehicle_type.GetValueByFieldName("drag_term_c", element.dragTermC);
			parser_vehicle_type.GetValueByFieldName("source_mass", element.sourceMass);


			float percentage_of_age = 0;

			int age = 0;

			// initialize age vector from 0 year to 30 year
			for (age = 0; age <= 30; age++)
			{
				element.percentage_age_vector.push_back(0);
			}

			for (age = 0; age <= 30; age++)
			{
				CString str_age;
				str_age.Format("percentage_of_age_%d", age);

				CT2CA pszConvertedAnsiString(str_age);
				// construct a std::string using the LPCSTR input
				std::string strStd(pszConvertedAnsiString);

				if (parser_vehicle_type.GetValueByFieldName(strStd, percentage_of_age) == true) // with data
				{
					element.percentage_age_vector[age] = percentage_of_age;

				}

			}


			g_VehicleTypeVector.push_back(element);

		}

	}
	else
	{
		cout << "input_vehicle_type.csv cannot be opened. " << endl;
		g_ProgramStop();

	}
	cout << "# of Vehicle Types = " << g_VehicleTypeVector.size() << endl;
}
std::vector<DTAVehicleType> g_VehicleTypeVector;
std::vector<DTAVehicle*> g_VehicleVector;
std::map<int, DTAVehicle*> g_VehicleMap;



int main()
{
	ReadInputVehicleTypeFile();
	ReadInputEmissionRateFile();

}


