//  Portions Copyright 2010 Xuesong Zhou

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
#include <stdlib.h>
#include <crtdbg.h>

#include "DTALite.h"


#include "Geometry.h"
#include "GlobalData.h"
#include "CSVParser.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

using namespace std;

/*************************************
How to build a simple DTA simulator

step 0: basic input functions


defintions of DTANode, DTALink and DTAVehicle
utility function g_read_integer() g_read_float()
ReadNetworkData() and ReadVehicleData()

step 1: dynamic memory management
FreeMemory() to deallocate dynamic memory for the whole simulation program
Allocate and deallocate dynamic arrays for DTANetworkForSP

step 2: network building for traffic assignment
DTANetworkForSP::BuildNetworkBasedOnZoneCentriod(int ZoneID)

step 3: convert time-dependent OD demand to vehicles data, sorted by departure time
CreateVehicles(int origin_zone, int destination_zone, float number_of_vehicles, int demand_type, float starting_time_in_min, float ending_time_in_min)


step 4: Scan eligiable list and shortest path algorithm
SEList functions in DTANetworkForSP
TDLabelCorrecting_DoubleQueue(int origin, int departure_time)

step 5: assign path to vehicles
GetLinkNoByNodeIndex(int usn_index, int dsn_index)
ZoneBasedPathAssignment(int zone,int departure_time_begin, int departure_time_end, int iteration)

step 6: integerate network building, shortest path algorithm and path assignment to dynamic traffic assignment

step 7: output vehicle trajectory file

step 8: NetworkLoading

step 9: VehicularSimulation

step 10: parallel computing: assign vehicles to arrays with different origins/departure times, to speed up the calculation

step 11: parallel computing for different zones
OpenMP

step 12: prepare dynamic arrays for within-simulation shortest path for pre-trp and enroute info vehicles
store the data for each destination node to global array as follows.
//int ***g_RTNodePredAry;  // vehicle type, destination, node available in real-time simulation
//unsigned char *** g_RTLabelCostAry; // vehicle type, destination, node available in real-time simulation


Step 13:
allow multiple centriods per zone in zone.csv, change BuildNetworkBasedOnZoneCentriod()

step 13: stochastic capacity with random numbers for different zones

step 14: UE, SO algorithm, day to day learning algorithm
updating, estimating variance and making prediction

make this as a convergency process

route choice -> compare threshold

Step 15: GetTimeDependentCapacityAtSignalizedIntersection
Add dynamical outflow capacity for signalized intersection

step 16: assign pre-trip info shortest path to vehicles

habitual paths vs. real time info
information provides


feature functions: link-based shortest path to consider turnning panalty

*************************************/

/**************************
menu -> project -> property -> configuraiton -> debugging -> setup working directory

***************************/


// The one and only application object

TCHAR g_SupportedDemandFormat[200] = _T("column, matrix, full_matrix, agent_csv, dsp_vehicle_dat, agent_bin, dynasmart, emme_matrix,trip_csv,transims_trip_file");


CWinApp theApp;

TCHAR g_DTASettingFileName[_MAX_PATH] = _T("./DTASettings.txt");

C_RealTimeSimulationSettings g_RealTimeSimulationSettings;

NetworkSimulationResult g_SimulationResult;
std::vector<DTANode> g_NodeVector;
std::map<int, int> g_NodeNametoIDMap;

HistoricalDemand g_HistDemand;
std::vector<DTALink*> g_LinkVector;
std::map<string, DTALink*> g_LinkMap;
std::map<int, DTALink*> g_LinkIDMap;



std::map<std::string, DTALink*> g_CountSensorIDMap;
std::map<std::string, DTALink*> g_LinkKeyMap;
std::map<std::string, DTALink*> g_SpeedSensorIDMap;


std::map<int, DTAZone> g_ZoneMap;
std::vector<int> g_ZoneNumber2NoVector;
std::vector<int> g_ZoneNo2NumberVector;

std::vector<DTAVehicleType> g_VehicleTypeVector;

std::vector<DTAVehicle*>		g_VehicleVector;

std::map<int, DTAVehicle*> g_VehicleMap;
std::map<int, DemandType> g_DemandTypeMap;
std::map<int, PricingType> g_PricingTypeMap;
std::map<int, DTALinkType> g_LinkTypeMap;
std::map<int, string> g_NodeControlTypeMap;

// time inteval settings in assignment and simulation
double g_DTASimulationInterval = 0.10000; // min

double g_UnitOfMileOrKM = 1.0000;
double g_gain_factor_link_travel_time_from_external_input = 0.5;

double g_CarFollowingSimulationInterval = 1.0/600; // 1/ 600 min
int g_number_of_intervals_per_min = 10; // round to nearest integer
int g_number_of_car_following_intervals_per_min = 600; // round to nearest integer
int g_AggregationTimetInterval = 15; // min
int g_TDSPTimetIntervalSizeForMin = 1;
int g_AggregationTimetIntervalSize = -1;  // no value
float g_DemandGlobalMultiplier = 1.0f;
int g_EmissionSmoothVehicleTrajectory = 0;
// maximal # of adjacent links of a node (including physical nodes and centriods( with connectors))
int g_AdjLinkSize = 30; // initial value of adjacent links

int g_ODZoneNumberSize = 0;
int g_ODZoneIDSize = 0;
int g_number_of_prohibited_movements = 0; 
int g_StartIterationsForOutputPath = 2;
int g_EndIterationsForOutputPath = 2;

// assignment and simulation settings
int g_NumberOfIterations = 1;
int g_ParallelComputingMode = 1;
int g_AgentBasedAssignmentFlag = 1;
int g_ProhibitUTurnOnFeewayLinkFlag = 1;
float g_ConvergencyRelativeGapThreshold_in_perc;
int g_NumberOfInnerIterations;

e_demand_loading_mode g_VehicleLoadingMode = demand_matrix_file_mode; // not load from vehicle file by default, 1: load vehicle file
int g_PlanningHorizon = 120;  // short horizon for saving memory

int g_SimululationReadyToEnd = 120;

// assignment
e_assignment_method g_UEAssignmentMethod = assignment_fixed_percentage; // 0: MSA, 1: day-to-day learning, 2: GAP-based switching rule for UE, 3: Gap-based switching rule + MSA step size for UE

float g_DepartureTimeChoiceEarlyDelayPenalty = 1;
float g_DepartureTimeChoiceLateDelayPenalty = 1;
float g_CurrentGapValue = 0.0; // total network gap value in the current iteration
float g_CurrentRelativeGapValue = 0.0;
float g_PrevRelativeGapValue = 0.0;
float g_PercentageCompleteTrips = 100.0;
int g_CurrentNumOfVehiclesSwitched = 0; // total number of vehicles switching paths in the current iteration; for MSA, g_UEAssignmentMethod = 0
int g_CurrentNumOfVehiclesForUEGapCalculation = 0;
int g_PrevNumOfVehiclesSwitched = 0; // // total number of vehicles switching paths in last iteration; for MSA, g_UEAssignmentMethod = 0
int g_ConvergenceThreshold_in_Num_Switch; // the convergence threshold in terms of number of vehicles switching paths; for MSA, g_UEAssignmentMethod = 0
int g_VehicleExperiencedTimeGap = 1; // 1: Vehicle experienced time gap; 0: Avg experienced path time gap
int g_NewPathWithSwitchedVehicles = 0; // number of new paths with vehicles switched to them

float g_TotalDemandDeviation = 0;
float g_UpdatedDemandPrintOutThreshold = 5;
float g_TotalMeasurementDeviation = 0; 

float g_UserClassPerceptionErrorRatio[MAX_SIZE_INFO_USERS] = {0};
int g_output_OD_path_MOE_file = 1;
int g_output_OD_TD_path_MOE_file = 1;
int g_output_OD_path_MOE_cutoff_volume = 1;
float g_OverallPerceptionErrorRatio = 0;
float g_VMSPerceptionErrorRatio;

int g_information_updating_interval_in_min;
bool g_bInformationUpdatingAndReroutingFlag = false;
bool g_bVehicleAttributeUpdatingFlag = false;

int g_information_updating_interval_of_VMS_in_min = 60;


int g_LearningPercentage = 15;
float g_TravelTimeDifferenceForSwitching = 1.0;  // min
float g_RelativeTravelTimePercentageDifferenceForSwitching = 15;  // min


int g_RandomizedCapacityMode = 0;
double g_CapacityLoadingFactor = 1.0;
int g_StochasticCapacityMode = 0;
int g_UseRandomCapacityMode = 0;
float g_MinimumInFlowRatio = 0.1f;
float g_RelaxInFlowConstraintAfterDemandLoadingTime = 60;
float g_MaxDensityRatioForVehicleLoading = 0.8f;
float g_DefaultSaturationFlowRate_in_vehphpl;

std::vector<VOTDistribution> g_VOTDistributionVector;
std::vector<TimeDependentDemandProfile> g_TimeDependentDemandProfileVector;
int g_DemandLoadingStartTimeInMin = 0;
int g_DemandLoadingEndTimeInMin = 0;

int g_ValidationDataStartTimeInMin =0;
int g_ValidationDataEndTimeInMin =0;

int g_number_of_warnings = 0;  // use a global count to avoid warning messages when running multiple scenarioss

double g_number_of_intra_zone_trips = 0;
ofstream g_scenario_short_description;

int g_Number_of_CompletedVehicles = 0;
int g_Number_of_CompletedVehiclesThatSwitch = 0;
int g_Number_of_GeneratedVehicles = 0;

int g_InfoTypeSize  = 1;  // for shortest path with generalized costs depending on LOV, HOV, trucks or other vehicle classes.
int g_start_iteration_for_MOEoutput = 0;

// for fast data acessing
int g_LastLoadedVehicleID = 0; // scan vehicles to be loaded in a simulation interval
int g_use_routing_policy_from_external_input = 0;
int g_output_routing_policy_file = 0;

int g_SystemOptimalStartingTimeinMin = 450;

VehicleArrayForOriginDepartrureTimeInterval** g_TDOVehicleArray =NULL; // TDO for time-dependent origin

std::vector<DTA_vhc_simple>   g_simple_vector_vehicles;	// vector of DSP_Vehicle, not pointer!;

FILE* g_ErrorFile = NULL;


ofstream g_LogFile;

CCSVWriter g_SummaryStatFile;
CCSVWriter g_MultiScenarioSummaryStatFile;

ofstream g_AssignmentLogFile;
ofstream g_EstimationLogFile;
ofstream g_WarningFile;

e_traffic_flow_model g_TrafficFlowModelFlag = tfm_BPR;
e_signal_representation_model g_SignalRepresentationFlag = signal_model_continuous_flow;
float g_LearningPercVector[1000] = {10};

int g_ShortestPathWithMovementDelayFlag = 0;
int g_UseDefaultLaneCapacityFlag = 1;
int g_UseFreevalRampMergeModelFlag = 0;
int g_OutputLinkCapacityFlag = 0;
int g_OutputLinkCapacityStarting_Time =0;
int g_OutputLinkCapacityEnding_Time =300;
int g_CalculateUEGapForAllAgents = 0;
int g_EmissionDataOutputFlag = 0;
int g_VehiclePathOutputFlag = 1;
int g_TimeDependentODMOEOutputFlag = 0;
int g_OutputSecondBySecondEmissionData =0;
float g_OutputSecondBySecondEmissionDataPercentage = 0.1f;
int g_start_departure_time_in_min_for_output_second_by_second_emission_data = 0;
int g_end_departure_time_in_min_for_output_second_by_second_emission_data = 0;
int g_OutputEmissionOperatingModeData = 0;
int g_TargetVehicleID_OutputSecondBySecondEmissionData = 0;

int g_TollingMethodFlag = 0;
float g_VMTTollingRate = 0;

int g_MergeNodeModelFlag=1;
int g_FIFOConditionAcrossDifferentMovementFlag = 0;

std::vector<NetworkMOE>  g_NetworkMOEAry;
std::vector<NetworkLoadingOutput>  g_AssignmentMOEVector;
std::map<int, RealTimeSimulationSettings>  g_RealTimeSimulationSettingsMap;

float g_ratio_mile_to_km = 1.60934;
DTASettings g_settings;  // global settings;


CTime g_AppStartTime;
CTime g_AppLastIterationStartTime;

unsigned int g_RandomSeed = 100;


using namespace std;


void ReadNodeControlTypeCSVFile()
{

	g_NodeControlTypeMap[0] = "unknown_control";
	g_NodeControlTypeMap[1] = "no_control";
	g_NodeControlTypeMap[2] = "yield_sign";
	g_NodeControlTypeMap[3] = "2way_stop_sign";
	g_NodeControlTypeMap[4] = "4way_stop_sign";
	g_NodeControlTypeMap[5] = "pretimed_signal";
	g_NodeControlTypeMap[6] = "actuated_signal";
	g_NodeControlTypeMap[7] = "roundabout";


	CCSVParser parser;
	if (parser.OpenCSVFile("input_node_control_type.csv"))
	{
		int control_type_code;
		int i=0;
		while(parser.ReadRecord())
		{
			control_type_code = 0;
			parser.GetValueByFieldName("unknown_control",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "unknown_control";

			control_type_code = 1;
			parser.GetValueByFieldName("no_control",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "no_control";

			control_type_code = 2;
			parser.GetValueByFieldName("yield_sign",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "yield_sign";

			control_type_code = 3;
			parser.GetValueByFieldName("2way_stop_sign",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "2way_stop_sign";

			control_type_code = 4;
			parser.GetValueByFieldName("4way_stop_sign",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "4way_stop_sign";

			control_type_code = 5;
			parser.GetValueByFieldName("pretimed_signal",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "pretimed_signal";

			control_type_code = 6;
			parser.GetValueByFieldName("actuated_signal",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "actuated_signal";

			control_type_code = 7;
			parser.GetValueByFieldName("roundabout",control_type_code);
			g_NodeControlTypeMap[control_type_code] = "roundabout";



			break;  // just one line
		}




	}

}

int FindNodeControlType(string control_type)
{
	for (std::map<int, string>::iterator iter_control_type = g_NodeControlTypeMap.begin(); iter_control_type != g_NodeControlTypeMap.end(); iter_control_type++)
	{
		if(iter_control_type->second.find(control_type) != string::npos)
		{
			return iter_control_type->first;
		}
	}

	return 0;
}

void g_ReadInputFiles()
{
	

	//*******************************
	// step 1: node input
	cout << "Step 1: Reading file input_node.csv..."<< endl;
	g_LogFile << "Step 1: Reading file input_node.csv..."<< endl;

	
	if(g_UEAssignmentMethod != assignment_accessibility_distanance)  //  node control type is not required when calculating node-to-node distance 
	{
		ReadNodeControlTypeCSVFile();
	}

	if(FindNodeControlType("pretimed_signal")>0)  // valid record
	{
		g_settings.pretimed_signal_control_type_code = FindNodeControlType("pretimed_signal");
	}

	if(FindNodeControlType("actuated_signal")>0)  //valid record
	{
		g_settings.actuated_signal_control_type_code = FindNodeControlType("actuated_signal");
	}

	if(FindNodeControlType("no_control")>0)  //valid record
	{
		g_settings.no_signal_control_type_code = FindNodeControlType("no_control");
	}

	int NodeControlTypeCount[10];

	for(int control_type_i = 0; control_type_i <10; control_type_i++)
	{
		NodeControlTypeCount[control_type_i]  = 0;
	}

	CCSVParser parser_node;
	if (parser_node.OpenCSVFile("input_node.csv"))
	{
		int i=0;
		while(parser_node.ReadRecord())
		{
			int node_id;
			DTANode* pNode = 0;

			if(parser_node.GetValueByFieldNameRequired("node_id",node_id) == false)
				break;

			int control_type = 0;


			DTANode Node;
			parser_node.GetValueByFieldName ("control_type",control_type);

			parser_node.GetValueByFieldName("x",Node.m_pt .x);
			parser_node.GetValueByFieldName("y",Node.m_pt .y);


			

			Node.m_NodeID = i;
			Node.m_ZoneID = 0;
			Node.m_NodeNumber = node_id;
			Node.m_ControlType  = control_type;

			NodeControlTypeCount[control_type] +=1;
			g_NodeVector.push_back(Node);
			g_NodeNametoIDMap[node_id] = i;
			i++;
		}
	}else
	{
		cout << "Error: File input_node.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();

	}

	g_SummaryStatFile.WriteParameterValue ("# of Nodes",g_NodeVector.size());
	g_SummaryStatFile.WriteParameterValue ("# of Prohibited Movements", g_number_of_prohibited_movements);

	g_SummaryStatFile.WriteParameterValue ("# of Pretimed Signals", NodeControlTypeCount[g_settings.pretimed_signal_control_type_code]);
	g_SummaryStatFile.WriteParameterValue ("# of Actuated Signals", NodeControlTypeCount[g_settings.actuated_signal_control_type_code]);


	//*******************************
	// step 2: link type input

	cout << "Step 2: Reading file input_link_type.csv..."<< endl;
	g_LogFile << "Step 2: Reading file input_link_type.csv.." << endl;

	CCSVParser parser_link_type;

		if (!parser_link_type.OpenCSVFile("input_link_type.csv"))
		{
			cout << "Error: File input_link_type.csv cannot be opened.\n Try to use default value."<< endl;
			g_ProgramStop();
		

		}
		if (parser_link_type.inFile.is_open () || parser_link_type.OpenCSVFile("input_link_type.csv"))
		{

			std::map<int, int> link_type_map;
			int line_no = 0;

			while(parser_link_type.ReadRecord())
			{
				DTALinkType element;

				if(parser_link_type.GetValueByFieldName("link_type",element.link_type ) == false)
				{
					if(line_no==0)
					{
						cout << "Error: Field link_type cannot be found in file input_link_type.csv."<< endl;
						g_ProgramStop();
					}else
					{  // read empty line
						break;
					}
				}

				if(link_type_map.find(element.link_type) != link_type_map.end())
				{
					cout << "Error: Field link_type " << element.link_type << " has been defined more than once in file input_link_type.csv."<< endl;
					g_ProgramStop();

					break;
				}

				link_type_map[element.link_type ] = 1;



				if(parser_link_type.GetValueByFieldName("link_type_name",element.link_type_name ) == false)
				{
					g_LogFile << "Warning: value link_type_name for link type " << element.link_type << " cannot be found in file input_link_type.csv." << endl;

				}
				if(parser_link_type.GetValueByFieldName("type_code",element.type_code  ) == false)
				{
					cout << "Error: Field type_code for link type " << element.link_type << "cannot be found in file input_link_type.csv." << endl;
					cout << "The corresponding links will not be added in the network." << endl;
					cout << "Please check and press any key to continue!" << endl;

					element.type_code = "n";
					getchar();
				}


				if(element.type_code.find_first_not_of("afrhwctnb")!=string::npos)
				{
					cout << "Error: invalid type_code for link type "<< element.link_type_name << " in input_link_type.csv = " << element.type_code << endl;
					g_ProgramStop();
				}

				parser_link_type.GetValueByFieldName("travel_time_bias_factor",element.link_type_bias_factor  );


				if(element.link_type_bias_factor  <=0.01 || element.link_type_bias_factor >=100)
				{
					cout << "Error: invalid link_type_bias_factor for link type "<< element.link_type_name << " in input_link_type.csv = " << element.link_type_bias_factor << endl;
					g_ProgramStop();
				
				}

				parser_link_type.GetValueByFieldName("capacity_adjustment_factor",element.capacity_adjustment_factor  );


				if(element.capacity_adjustment_factor  <=0.1 || element.capacity_adjustment_factor >=10)
				{
					cout << "Error: invalid capacity_adjustment_factor for link type "<< element.link_type_name << " in input_link_type.csv = " << element.capacity_adjustment_factor << endl;
					g_ProgramStop();
				
				}

				parser_link_type.GetValueByFieldName("approximate_cycle_length_in_second",element.approximate_cycle_length_in_second  );


				if(element.approximate_cycle_length_in_second  <=-2 || element.approximate_cycle_length_in_second >=1000)
				{
					cout << "Error: invalid approximate_cycle_length_in_second for link type "<< element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
					g_ProgramStop();
				
				}
				
				parser_link_type.GetValueByFieldName("saturation_flow_rate_in_vhc_per_hour_per_lane",element.saturation_flow_rate_in_vhc_per_hour_per_lane  );


				if(element.saturation_flow_rate_in_vhc_per_hour_per_lane  <=-2 || element.saturation_flow_rate_in_vhc_per_hour_per_lane >=3000)
				{
					cout << "Error: invalid saturation_flow_rate_in_vhc_per_hour_per_lane for link type "<< element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
					g_ProgramStop();
				
				}

				if(element.approximate_cycle_length_in_second >=1)
				{
				if(element.saturation_flow_rate_in_vhc_per_hour_per_lane  <=1)
				{
					cout << "Error: invalid saturation_flow_rate_in_vhc_per_hour_per_lane for link type with positive approximate_cycle_length_in_second"<< element.link_type_name << " in input_link_type.csv = " << element.approximate_cycle_length_in_second << endl;
					g_ProgramStop();
				
				}
				}


				if(g_UseDefaultLaneCapacityFlag==1)
				{
					if(parser_link_type.GetValueByFieldName("default_lane_capacity",element.default_lane_capacity  ) == false)
					{
						cout << "Error: Field default_lane_capacity cannot be found in file input_link_type.csv."<< endl;
						g_ProgramStop();
					}
				}


				g_LinkTypeMap[element.link_type  ] = element;

				line_no++;
			}
		}else
		{
			cout << "Error: File input_link_type.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;


		}

		g_SummaryStatFile.WriteParameterValue ("# of Link Types",g_LinkTypeMap.size());



	//*******************************
	// step 3: link data input

	char InputLinkFileName[_MAX_PATH];

	GetPrivateProfileString("input_file","link_data","input_link.csv",InputLinkFileName,_MAX_PATH,g_DTASettingFileName);


	int AllowExtremelyLowCapacityFlag = g_GetPrivateProfileInt("input_checking", "allow_extremely_low_capacity", 1, g_DTASettingFileName);	
	g_ProhibitUTurnOnFeewayLinkFlag = g_GetPrivateProfileInt("shortest_path", "prohibit_u_turn_on_freeway_link", 1, g_DTASettingFileName);	


	cout << "Step 3: Reading file input_link.csv..."<< endl;
	g_LogFile << "Step 3: Reading file input_link.csv..." << endl;

	int i = 0;

	int max_number_of_warnings_to_be_showed = 5;
	DTALink* pLink = 0;
	CCSVParser parser_link;
	int signal_reset_count = 0;
	int control_node_number = 0;
	int missing_node_error = 0;

	double total_link_length = 0;
	double total_link_coordinate_length = 0;
	if (parser_link.OpenCSVFile(InputLinkFileName))
	{
		bool bNodeNonExistError = false;
		while(parser_link.ReadRecord())
		{
			int from_node_name = 0;
			int to_node_name = 0;
			int direction = 1;
			double length_in_mile = 1;
			int number_of_lanes = 1;
			int speed_limit_in_mph = 0;
			float speed_at_capacity = 50;
			float KCritical = 10;
			double capacity = 0;
			int type;
			string name, mode_code;
			double K_jam,wave_speed_in_mph,AADT_conversion_factor;

			int org_link_id = 0;
			if(!parser_link.GetValueByFieldName("from_node_id",from_node_name)) 
			{
				if(i==0)
				{
					cout << "Field from_node_id has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
				else
				{ // i>=1;
					break;  //read empty line.

				}
			}

			if(!parser_link.GetValueByFieldName("to_node_id",to_node_name))
			{
				cout << "Field to_node_id has not been defined in file input_link.csv. Please check.";
				getchar();
				exit(0);

			}

			if (!parser_link.GetValueByFieldName("mode_code", mode_code))
				mode_code = "";

			if (mode_code.compare("w") == 0)   // do not model walking-only link in this version
				continue;

			if (mode_code.compare("t") == 0)   // do not model trainsit-only link in this version
				continue;

			if (mode_code.compare("b") == 0)   // do not model pedestran-only link in this version
				continue;

			if (mode_code.compare("n") == 0)   // do not model pedestran-only link in this version
				continue;

			if (!parser_link.GetValueByFieldName("link_type", type))
			{
				if (g_UEAssignmentMethod != assignment_accessibility_distanance)
				{

					cout << "Field link_type has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}

			if (g_LinkTypeMap[type].type_code.find_first_of('nb') != string::npos)   //type_code: no: not included in the network, b: bike
				continue;



			if (g_NodeNametoIDMap.find(from_node_name) == g_NodeNametoIDMap.end())
			{

				fprintf(g_ErrorFile, "from_node_id %d in input_link.csv has not be defined in input_node.csv.", from_node_name);

				if (missing_node_error <= 3)
				{
				cout << "from_node_id " << from_node_name << " in input_link.csv has not be defined in input_node.csv." << endl;
				cout << "associated link_type : " << type << endl;
				cout << "Please check error.log." << endl;
				getchar();
				}
			missing_node_error++;
				continue;

			}

			if(g_NodeNametoIDMap.find(to_node_name)== g_NodeNametoIDMap.end())
			{

				fprintf(g_ErrorFile, "to_node_id %d in input_link.csv has not be defined in input_node.csv.", to_node_name);
				if (missing_node_error <= 3)
				{
					cout << "to_node_id " << to_node_name << " in input_link.csv has not be defined in input_node.csv. " << endl;
					cout << "associated from_node_id: " << from_node_name << " ; link_type : " << type << endl;
					cout << "Please check error.log." << endl;
 					getchar();
				}
				missing_node_error++;

				continue;
			}

			parser_link.GetValueByFieldName("link_id",org_link_id);

			if(!parser_link.GetValueByFieldName("direction",direction))
				direction = 1;

		

			if(!parser_link.GetValueByFieldName("length",length_in_mile))
			{

				if(!parser_link.GetValueByFieldName("length_in_mile",length_in_mile))
				{

				cout<< "Field length has not been defined in file input_link.csv. Please check.";
				getchar();
				exit(0);
				}
			}

			if(length_in_mile>100)
			{
				cout << "Link: " << from_node_name << "->" << to_node_name << " in input_link.csv has " << "length_in_mile = " << length_in_mile << " , which might be too long. Please check.";
				//				sleep(5);
			}

			if(!parser_link.GetValueByFieldName("number_of_lanes",number_of_lanes))
			{
				if(g_UEAssignmentMethod != assignment_accessibility_distanance)
				{
					cout << "Field number_of_lanes has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}
			}

			if (g_LinkTypeMap[type].type_code.find('c') != string::npos && number_of_lanes == 0)
			{
				number_of_lanes = 7; // reset # of lanes for connectors to a positive value
			}

			if(!parser_link.GetValueByFieldName("speed_limit",speed_limit_in_mph))
			{

				if(!parser_link.GetValueByFieldName("speed_limit_in_mph",speed_limit_in_mph))
				{
				cout << "Field speed_limit_in_mph has not been defined in file input_link.csv. Please check.";
				getchar();
				exit(0);
				}
			}
			if (!parser_link.GetValueByFieldName("speed_at_capacity", speed_at_capacity))
			{
				speed_at_capacity = speed_limit_in_mph - 15;
			}


			if(!parser_link.GetValueByFieldName("lane_capacity_in_vhc_per_hour",capacity))
			{
				if(g_UEAssignmentMethod != assignment_accessibility_distanance)
				{
					cout << "Field lane_capacity_in_vhc_per_hour has not been defined in file input_link.csv. Please check.";
					getchar();
					exit(0);
				}

			}

			KCritical = capacity / max(10, speed_at_capacity); // e.g. 1500 vehicles/hour/lane / 30 mph = 50 vehicles /mile/lane


			if(g_CapacityLoadingFactor < 0.999 || g_CapacityLoadingFactor > 1.001)
				capacity*=g_CapacityLoadingFactor;

	
			int SaturationFlowRate;

			float BPR_Alpha = 0.15f;
			float BPR_Beta = 4.0f;

			parser_link.GetValueByFieldName("BPR_alpha_term",BPR_Alpha);
			parser_link.GetValueByFieldName("BPR_beta_term",BPR_Beta);



			int LeftTurnBayLengthInFeet  = 0;
			int LeftTurnCapacity  = 0;
			char link_direction;


			parser_link.GetValueByFieldName("from_approach",link_direction);





			if((g_UEAssignmentMethod != assignment_accessibility_distanance) && g_LinkTypeMap.find(type) == g_LinkTypeMap.end())
			{
				int round_down_type = int(type/10)*10;
				if(g_LinkTypeMap.find(round_down_type) != g_LinkTypeMap.end())  // round down type exists
				{
					g_LinkTypeMap[type].type_code = g_LinkTypeMap[round_down_type].type_code ;
				}else
				{
					cout << "Link: " << from_node_name << "->" << to_node_name << " in input_link.csv has " << "link_type = " << type << ", which has not been defined in file input_link.csv. Please check. Use default link type: arterial.";
					g_LinkTypeMap[type].type_code = 'a';
					getchar();
				}

				//				if(g_UseDefaultLaneCapacityFlag ==1)
				capacity = g_LinkTypeMap[type].default_lane_capacity;
			}


			if(!parser_link.GetValueByFieldName("AADT_conversion_factor",AADT_conversion_factor))
				AADT_conversion_factor = 0.1;

			if(!parser_link.GetValueByFieldName("jam_density",K_jam))
			{
				if(!parser_link.GetValueByFieldName("jam_density_in_vhc_pmpl",K_jam))
				if(g_settings.use_mile_or_km_as_length_unit == 1)
					K_jam = 180;
				else
					K_jam = 180/g_ratio_mile_to_km;
			}

			if(g_CapacityLoadingFactor < 0.999 || g_CapacityLoadingFactor > 1.001)
				K_jam*=g_CapacityLoadingFactor;

			if(!parser_link.GetValueByFieldName("wave_speed",wave_speed_in_mph))
			{
			if(!parser_link.GetValueByFieldName("wave_speed_in_mph",wave_speed_in_mph))
			{
				if(g_settings.use_mile_or_km_as_length_unit == 1)
					wave_speed_in_mph = 12;
				else
					wave_speed_in_mph = 12/g_ratio_mile_to_km;

			}
			}


			int ProhibitedU_Turn = 0;

			parser_link.GetValueByFieldName("prohibited_u-turn,",ProhibitedU_Turn);


			if(from_node_name == 58987 && to_node_name == 54430) 
			{
				TRACE(" ");
			}



			int link_code_start = 1;
			int link_code_end = 1;

			if (direction == -1) // reversed
			{
				link_code_start = 2; link_code_end = 2;
			}

			if (direction == 0) // two-directional link
			{
				link_code_start = 1; link_code_end = 2;
			}


			if(number_of_lanes == 0)  // skip this link 
			{
				g_WarningFile << "link with 0 lane, skip:" <<from_node_name << " -> " <<to_node_name << endl;
				continue;
			}

			if(capacity <1)  // skip this link 
			{
				g_WarningFile << "link with capacity " <<  capacity << ", skip: " <<from_node_name << " -> " <<to_node_name << endl;
				continue;
			}

			if(speed_limit_in_mph <1)  // skip this link 
			{
				g_WarningFile << "link with speed limit " <<  speed_limit_in_mph << ", skip: " <<from_node_name << " -> " <<to_node_name << endl;
				continue;
			}

			for(int link_code = link_code_start; link_code <=link_code_end; link_code++)
			{

				int FromID = from_node_name;
				int ToID = to_node_name;
				if(link_code == 1)  //AB link
				{
					FromID = from_node_name;
					ToID = to_node_name;
				}
				if(link_code == 2)  //BA link
				{
					FromID = to_node_name;
					ToID = from_node_name;
				}


				pLink = new DTALink(g_PlanningHorizon);
				if(pLink==NULL)
				{
					cout << "Allocating memory error at line "<< i+1  << endl;
					getchar();
					exit(0);

				}

				parser_link.GetValueByFieldName("name",pLink->m_Name,true);

				std::string link_key, count_sensor_id,speed_sensor_id;

				parser_link.GetValueByFieldName("link_key",link_key);
				parser_link.GetValueByFieldName("count_sensor_id",count_sensor_id);
				parser_link.GetValueByFieldName("speed_sensor_id",speed_sensor_id);
				
				if(link_key.size() > 0 ) 
						g_LinkKeyMap[link_key] = pLink;

				if(count_sensor_id.size() > 0 ) 
						g_CountSensorIDMap[count_sensor_id] = pLink;

				if(speed_sensor_id.size() > 0 ) 
						g_SpeedSensorIDMap[speed_sensor_id] = pLink;


				pLink->m_LinkNo = i;
				pLink->m_RandomSeed = pLink->m_LinkNo; // assign a link specific random seed

				pLink->m_OrgLinkID =  org_link_id;
				pLink->m_link_code  = link_code;
				pLink->m_FromNodeNumber = FromID;
				pLink->m_ToNodeNumber = ToID;
				pLink->m_FromNodeID = g_NodeNametoIDMap[pLink->m_FromNodeNumber ];
				pLink->m_ToNodeID= g_NodeNametoIDMap[pLink->m_ToNodeNumber];

				pLink->m_ProhibitedU_Turn = ProhibitedU_Turn;

				if(ProhibitedU_Turn ==1)
				{
					g_ShortestPathWithMovementDelayFlag = true; // with movement input
						string movement_id = GetMovementStringID(FromID, ToID , FromID);
						int middle_node_id = g_NodeNametoIDMap[ToID ];

						g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_from_node_id = FromID;
						g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_to_node_id = ToID ; 
						g_NodeVector[middle_node_id].m_MovementMap[movement_id].out_link_to_node_id = FromID;

						g_NodeVector[middle_node_id].m_MovementMap[movement_id].b_turning_prohibited = true;   // assign movement to individual node

						g_number_of_prohibited_movements++;
				}


				pLink->m_BPR_Alpha = BPR_Alpha;
				pLink->m_BPR_Beta = BPR_Beta;

				std::vector<CCoordinate> CoordinateVector;
				string geo_string;

				double link_coordinate_length = 0;
				if(parser_link.GetValueByFieldName("geometry",geo_string) == false)
				{
				// overwrite when the field "geometry" exists
				CGeometry geometry(geo_string);
				CoordinateVector = geometry.GetCoordinateList();
				for(int si = 0; si < CoordinateVector.size(); si++)
				{
					GDPoint	pt;
					pt.x = CoordinateVector[si].X;
					pt.y = CoordinateVector[si].Y;
					pLink->m_ShapePoints .push_back (pt);

					if(si>=1)
					{
						link_coordinate_length += pow( pow(CoordinateVector[si].X - CoordinateVector[si-1].X , 2) +  pow(CoordinateVector[si].Y - CoordinateVector[si-1].Y , 2),0.5);

					}
				}

				}
				

				parser_link.GetValueByFieldName("geometry",pLink->m_geometry_string);
				parser_link.GetValueByFieldName("geometry",pLink->m_original_geometry_string);


				total_link_length += length_in_mile;
				total_link_coordinate_length += link_coordinate_length;


				pLink->m_SpeedLimit= speed_limit_in_mph;

				pLink->m_SpeedAtCapacity = speed_at_capacity;
				pLink->m_KCritical = KCritical;
				
				float	m_KCritical;
				float length = max(0.00001,length_in_mile);  // minimum link length 0.0001

				pLink->m_OutflowNumLanes= number_of_lanes;
				pLink->m_InflowNumLanes = number_of_lanes;

				pLink->m_Orginal_OutflowNumLanes = number_of_lanes;
				pLink->m_Orginal_InflowNumLane = number_of_lanes;

				

				pLink->m_Direction = link_direction;

				if(g_AgentBasedAssignmentFlag != assignment_accessibility_distanance && g_TrafficFlowModelFlag != tfm_BPR)
					pLink->m_Length= max(length, pLink->m_SpeedLimit*0.1f/60.0f);  // we do not impose the minimum distance in this version
				else
					pLink->m_Length= length;



				if(AllowExtremelyLowCapacityFlag == 0 && capacity < 10 && g_number_of_warnings<max_number_of_warnings_to_be_showed)
				{
					cout << "In file input_link.csv, line "<< i+1 << " has capacity <10" << capacity <<", which might not be realistic. Please correct the error." << endl;
					getchar();
					g_number_of_warnings++;
				}

				

				pLink->m_BPRLaneCapacity  = pLink->m_LaneCapacity;
				pLink->m_link_type= type;



				if(g_LinkTypeMap.find(type) == g_LinkTypeMap.end())
				{
					int round_down_type = int(type/10)*10;
					if(g_LinkTypeMap.find(round_down_type) != g_LinkTypeMap.end())
					{
						g_LinkTypeMap[type].type_code = g_LinkTypeMap[round_down_type].type_code ;
					}else
					{
						cout << "In file input_link.csv, line "<< i+1 << " has link type "<< type <<", which has not been defined in input_link_type.csv. Please correct. Use default link type: arterial street." << endl;
						g_LinkTypeMap[type].type_code = 'a';
						getchar();
					}
				}


				pLink->m_LaneCapacity= capacity * g_LinkTypeMap[type].capacity_adjustment_factor ;


				pLink->m_LinkTypeName  = g_LinkTypeMap[type].link_type_name;


				pLink->m_bFreewayType = g_LinkTypeMap[type].IsFreeway ();
				pLink->m_bArterialType = g_LinkTypeMap[type].IsArterial();




				pLink->m_KJam = K_jam;
				pLink->m_AADTConversionFactor = AADT_conversion_factor;
				pLink->m_BackwardWaveSpeed = wave_speed_in_mph;

				if(g_LinkTypeMap[type].IsConnector () == true && g_NodeVector[pLink->m_ToNodeID]. m_ControlType!= g_settings.no_signal_control_type_code)
				{
					//"no_control" for the downstream node of a connector
					//							g_NodeVector[pLink->m_ToNodeID]. m_ControlType = no_signal_control_type_code;
					if(control_node_number==0)
						control_node_number= pLink->m_ToNodeNumber ;

					//							signal_reset_count++;
				}

				pLink->m_VehicleSpaceCapacity = max(1,pLink->m_Length * pLink->m_OutflowNumLanes *K_jam); 

				g_NodeVector[pLink->m_FromNodeID ].m_TotalCapacity += (pLink->m_LaneCapacity* pLink->m_OutflowNumLanes);
				g_NodeVector[pLink->m_ToNodeID ].m_IncomingLinkVector.push_back(i);
				g_NodeVector[pLink->m_FromNodeID ].m_OutgoingLinkVector .push_back(i);

				g_NodeVector[pLink->m_ToNodeID ].m_IncomingLinkDelay.push_back(0);

				// prevent U turns on freeway links
				if(g_ProhibitUTurnOnFeewayLinkFlag == 1 && g_LinkTypeMap[pLink->m_link_type].IsFreeway () == true)
				{
					g_ShortestPathWithMovementDelayFlag = true; // with movement input

						string movement_id = GetMovementStringID(pLink->m_FromNodeNumber, pLink->m_ToNodeNumber , pLink->m_FromNodeNumber);
						int middle_node_id = g_NodeNametoIDMap[pLink->m_ToNodeNumber ];

						g_NodeVector[middle_node_id].m_MovementMap[movement_id].turning_direction = "U-turn";
						g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_from_node_id = pLink->m_FromNodeNumber;
						g_NodeVector[middle_node_id].m_MovementMap[movement_id].in_link_to_node_id = pLink->m_ToNodeNumber ; 
						g_NodeVector[middle_node_id].m_MovementMap[movement_id].out_link_to_node_id = pLink->m_FromNodeNumber;

						g_NodeVector[middle_node_id].m_MovementMap[movement_id].b_turning_prohibited = true;   // assign movement to individual node


				
				}


				pLink->CalculateShapePointRatios();
				pLink->SetupMOE();




		// tally statics for each link type
		g_LinkTypeMap[pLink->m_link_type].number_of_links ++;
		g_LinkTypeMap[pLink->m_link_type].total_lane_capacity += pLink->m_LaneCapacity ;
		g_LinkTypeMap[pLink->m_link_type].total_speed_limit += pLink->m_SpeedLimit  ;
		g_LinkTypeMap[pLink->m_link_type].total_number_of_lanes += pLink->m_OutflowNumLanes   ;
		g_LinkTypeMap[pLink->m_link_type].total_k_jam += pLink->m_KJam    ;
		g_LinkTypeMap[pLink->m_link_type].total_length += pLink->m_Length   ;

			g_LinkVector.push_back(pLink);
			string link_string_id = GetLinkStringID(FromID, ToID);
			g_LinkMap[link_string_id] = pLink;
			g_LinkIDMap[org_link_id] = pLink;


			

				i++;



				if(i%1000==0)
				{
					cout << " loading " << i/1000 << "K links..." << endl;
				}

				if(i == MAX_LINK_NO && g_AgentBasedAssignmentFlag != assignment_accessibility_distanance) // g_AgentBasedAssignmentFlag == 2  -> no vehicle simulation
				{
					cout << "The network has more than "<< MAX_LINK_NO << " links."<< endl <<"Please contact the developers for a new 64 bit version for this large-scale network." << endl;
					getchar();
					exit(0);

				}

			}
		}

			g_UnitOfMileOrKM  =  	total_link_coordinate_length/max(0.1,total_link_length);



	}else
	{
		cout << "Error: File input_link.csv cannot be opened.\n It might be currently used and locked by EXCEL."<< endl;
		g_ProgramStop();
	}


	if(signal_reset_count>=1)
	{
		cout << signal_reset_count << " nodes' control type is reset to 'no control', as they are connected to connectors. " << endl << "For example, node " << control_node_number << endl;
		getchar();
	}

	cout << " total # of links loaded = " << g_LinkVector.size() << endl;

	g_SummaryStatFile.WriteParameterValue ("# of Links", g_LinkVector.size());



	//*******************************
	// step 5: vehicle type input

	cout << "Step 6: Reading file input_vehicle_type.csv..."<< endl;
	g_LogFile << "Step 6: Reading file input_vehicle_type.csv..."<< endl;

	CCSVParser parser_vehicle_type;

	if (!parser_vehicle_type.OpenCSVFile("input_vehicle_type.csv"))
	{
		cout << "input_vehicle_type.csv cannot be opened.  Use default values. "<< endl;

		ofstream VehicleTypeFile;
		VehicleTypeFile.open("input_vehicle_type.csv");
		if(VehicleTypeFile.is_open ())
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

	if (parser_vehicle_type.inFile .is_open () || parser_vehicle_type.OpenCSVFile("input_vehicle_type.csv"))
	{
		g_VehicleTypeVector.clear();
		while(parser_vehicle_type.ReadRecord())
		{
			int vehicle_type =0;
			if(parser_vehicle_type.GetValueByFieldName("vehicle_type",vehicle_type) == false)
				break;

			string vehicle_type_name;
			parser_vehicle_type.GetValueByFieldName("vehicle_type_name",vehicle_type_name);




			DTAVehicleType element;
			element.vehicle_type = vehicle_type;
			element.vehicle_type_name  = vehicle_type_name;

			parser_vehicle_type.GetValueByFieldName("rolling_term_a",element.rollingTermA);
			parser_vehicle_type.GetValueByFieldName("rotating_term_b",element.rotatingTermB);
			parser_vehicle_type.GetValueByFieldName("drag_term_c",element.dragTermC);
			parser_vehicle_type.GetValueByFieldName("source_mass",element.sourceMass);


			float percentage_of_age = 0;

			int age =0;

			// initialize age vector from 0 year to 30 year
			for(age = 0; age <= 30; age++)
			{
				element.percentage_age_vector .push_back (0);
			}

			for(age = 0; age <= 30; age++)
			{
				CString str_age;
				str_age.Format ("percentage_of_age_%d",age);

				CT2CA pszConvertedAnsiString (str_age);
				// construct a std::string using the LPCSTR input
				std::string strStd (pszConvertedAnsiString);

				if(parser_vehicle_type.GetValueByFieldName(strStd,percentage_of_age) == true) // with data
				{
					element.percentage_age_vector [age] = percentage_of_age;

				}

			}


			g_VehicleTypeVector.push_back(element);

		}

	}else
	{
		cout << "input_vehicle_type.csv cannot be opened. "<< endl;
		g_ProgramStop();

	}
	g_SummaryStatFile.WriteParameterValue ("# of Vehicle Types", g_VehicleTypeVector.size());


	g_SummaryStatFile.WriteTextLabel ("Demand Load Mode=,agent.bin file");

	////////////////////////////////////// VOT
	cout << "Step 10: Reading files based on user settings in meta database file..."<< endl;
	g_LogFile << "Step 10: Reading files  based on user settings in  meta database file..." << endl;


	g_ReadAgentBinFile("agent.bin", false);
	

	if (g_PlanningHorizon < g_DemandLoadingEndTimeInMin + 300)
	{
		//reset simulation horizon to make sure it is longer than the demand loading horizon
		g_PlanningHorizon = g_DemandLoadingEndTimeInMin + 300;

		for (unsigned link_index = 0; link_index< g_LinkVector.size(); link_index++)
		{
			DTALink* pLink = g_LinkVector[link_index];
			pLink->ResizeData(g_PlanningHorizon);
		}

	}



	cout << "Number of Vehicle Types = " << g_VehicleTypeVector.size() << endl;
	cout << "Number of Demand Types = " << g_DemandTypeMap.size() << endl;
	cout << "Number of VOT records = " << g_VOTDistributionVector.size() << endl;

	g_LogFile << "Number of Zones = " << g_ODZoneNumberSize << endl;
	g_LogFile << "Number of Nodes = " << g_NodeVector.size() << endl;
	g_LogFile << "Number of Links = " << g_LinkVector.size() << endl;
	g_LogFile << "Number of Vehicles to be Simulated = " << g_VehicleVector.size() << endl;
	g_LogFile << "Demand Loading Period = " << g_DemandLoadingStartTimeInMin << " min -> " << g_DemandLoadingEndTimeInMin << " min." << endl;
	g_LogFile << "Number of Vehicle Types = " << g_VehicleTypeVector.size() << endl;


	CString title_str;
	title_str.Format("\n--Link Type Statistics--");
	g_SummaryStatFile.WriteTextString(title_str);
	g_SummaryStatFile.Reset();

	g_SummaryStatFile.SetFieldName("link_type");
	g_SummaryStatFile.SetFieldName("link_type_name");
	g_SummaryStatFile.SetFieldName("link_type_code");
	g_SummaryStatFile.SetFieldName("number_of_links");
	g_SummaryStatFile.SetFieldName("avg_lane_capacity");
	g_SummaryStatFile.SetFieldName("avg_number_of_lanes");
	g_SummaryStatFile.SetFieldName("avg_speed_limit");
	g_SummaryStatFile.SetFieldName("avg_link_length");
	g_SummaryStatFile.SetFieldName("avg_K_jam");
	g_SummaryStatFile.SetFieldName("total_link_length");

	g_SummaryStatFile.WriteHeader(false, false);

	for (std::map<int, DTALinkType>::iterator itr = g_LinkTypeMap.begin(); itr != g_LinkTypeMap.end(); itr++)
	{
		std::replace(itr->second.link_type_name.begin(), itr->second.link_type_name.end(), ',', ' ');

		g_SummaryStatFile.SetValueByFieldName("link_type", itr->first);
		std::string link_type_name = itr->second.link_type_name.c_str();
		std::string type_code = itr->second.type_code.c_str();
		g_SummaryStatFile.SetValueByFieldName("link_type_name", link_type_name);
		g_SummaryStatFile.SetValueByFieldName("link_type_code", type_code);
		g_SummaryStatFile.SetValueByFieldName("number_of_links", itr->second.number_of_links);

		float avg_lane_capacity = itr->second.total_lane_capacity / max(1, itr->second.number_of_links);
		float avg_number_of_lanes = itr->second.total_number_of_lanes / max(1, itr->second.number_of_links);
		float avg_speed_limit = itr->second.total_speed_limit / max(1, itr->second.number_of_links);
		float avg_K_jam = itr->second.total_k_jam / max(1, itr->second.number_of_links);
		float avg_link_length = itr->second.total_length / max(1, itr->second.number_of_links);
		float total_link_length = avg_link_length * itr->second.number_of_links;

		g_SummaryStatFile.SetValueByFieldName("avg_lane_capacity", avg_lane_capacity);
		g_SummaryStatFile.SetValueByFieldName("avg_number_of_lanes", avg_number_of_lanes);
		g_SummaryStatFile.SetValueByFieldName("avg_speed_limit", avg_speed_limit);
		g_SummaryStatFile.SetValueByFieldName("avg_K_jam", avg_K_jam);
		g_SummaryStatFile.SetValueByFieldName("avg_link_length", avg_link_length);
		g_SummaryStatFile.SetValueByFieldName("total_link_length", total_link_length);

		g_SummaryStatFile.WriteRecord();
	}


	g_EmissionDataOutputFlag = g_GetPrivateProfileInt("emission", "output_emission_data", 1, g_DTASettingFileName);

	g_OutputEmissionOperatingModeData = g_GetPrivateProfileInt("emission", "output_opreating_mode_data", 0, g_DTASettingFileName);

	g_OutputSecondBySecondEmissionData = g_GetPrivateProfileInt("emission", "output_second_by_second_emission_data", 0, g_DTASettingFileName);
	g_OutputSecondBySecondEmissionDataPercentage = g_GetPrivateProfileFloat("emission", "sampling_percentange_for_outputting_second_by_second_emission_data", 1, g_DTASettingFileName);
//	g_EmissionSmoothVehicleTrajectory = g_GetPrivateProfileFloat("emission", "smooth_vehicle_trajectory", 1, g_DTASettingFileName);
	g_start_departure_time_in_min_for_output_second_by_second_emission_data = g_GetPrivateProfileInt("emission", "start_departure_time_in_min_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);
	g_end_departure_time_in_min_for_output_second_by_second_emission_data = g_GetPrivateProfileInt("emission", "end_departure_time_in_min_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);
	g_TargetVehicleID_OutputSecondBySecondEmissionData = g_GetPrivateProfileInt("emission", "target_vehicle_id_for_output_second_by_second_emission_data", 0, g_DTASettingFileName);


	ReadInputEmissionRateFile();
	ReadFractionOfOperatingModeForBaseCycle();
	SetupOperatingModeVector();
	OutputEmissionData();
}


void g_DetermineDemandLoadingPeriod()
{

	g_DemandLoadingStartTimeInMin = 1440;
	g_DemandLoadingEndTimeInMin = 0;

	CCSVParser parser0;
	if (parser0.OpenCSVFile("input_demand_meta_data.csv"))
	{
		int i = 0;

		while (parser0.ReadRecord())
		{

			int file_sequence_no = -1;
			string file_name;
			string format_type;
			int number_of_lines_to_be_skipped = 0;
			int subtotal_in_last_column = 0;
			int demand_type_in_3rd_column = 0;
			float loading_multiplier = 1;
			int start_time_in_min = -1;
			int end_time_in_min = -1;
			int number_of_demand_types = 0;
			float local_demand_loading_multiplier = 1;
			char demand_type_field_name[20];
			int demand_type_code[20] = { 0 };

			int demand_format_flag = 0;

			parser0.GetValueByFieldNameWithPrintOut("file_sequence_no", file_sequence_no);

			if (file_sequence_no <= -1)  // skip negative sequence no 
				break;

			parser0.GetValueByFieldNameWithPrintOut("file_name", file_name);
			if (file_name.length() == 0)  // no file name input
			{
				break;
			}

			parser0.GetValueByFieldNameWithPrintOut("start_time_in_min", start_time_in_min);
			parser0.GetValueByFieldNameWithPrintOut("end_time_in_min", end_time_in_min);

			if (start_time_in_min == -1)  // skip negative sequence no 
			{
				cout << "Please provide start_time_in_min in file input_demand_meta_data.csv" << endl;
				g_ProgramStop();
			}
			if (end_time_in_min == -1)  // skip negative sequence no 
			{
				cout << "Please provide end_time_in_min in file input_demand_meta_data.csv" << endl;
				g_ProgramStop();
			}

			if (end_time_in_min>1440)
			{
				cout << "end_time_in_min should be less than 1440 min in input_demand_meta_data.csv" << endl;
				g_ProgramStop();
			}

			if (start_time_in_min < 0)
			{
				cout << "start_time_in_min should be greater than 0 min in input_demand_meta_data.csv" << endl;
				g_ProgramStop();
			}

			// set g_DemandLoadingStartTimeInMin according the start time and end time of each record
			if (g_DemandLoadingStartTimeInMin > start_time_in_min)
				g_DemandLoadingStartTimeInMin = start_time_in_min;

			if (g_DemandLoadingEndTimeInMin < end_time_in_min)
				g_DemandLoadingEndTimeInMin = end_time_in_min;

		}

	}  //determine loading horizon

	if (g_DemandLoadingStartTimeInMin > g_DemandLoadingEndTimeInMin)
	{  // reset
		g_DemandLoadingStartTimeInMin = 0;
		g_DemandLoadingEndTimeInMin = 1400;
		g_PlanningHorizon = 1440;

	}

	cout << "demand loading starting time = " << g_DemandLoadingStartTimeInMin << " (min)" << endl;
	cout << "demand loading ending time = " << g_DemandLoadingEndTimeInMin << " (min)" << endl;

	if (g_PlanningHorizon < g_DemandLoadingEndTimeInMin + 300)
	{
		//reset simulation horizon to make sure it is longer than the demand loading horizon
		g_PlanningHorizon = g_DemandLoadingEndTimeInMin + 300;
	}
}
bool g_ReadAgentBinFile(string file_name, bool b_with_updated_demand_type_info)
{

	cout << "Reading Agent Bin File..." << endl;
	g_VehicleLoadingMode = vehicle_binary_file_mode;

	g_DetermineDemandLoadingPeriod();

	int path_node_sequence[MAX_NODE_SIZE_IN_A_PATH];

	g_AllocateDynamicArrayForVehicles();

	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "rb");
	if (st != NULL)
	{
		struct_VehicleInfo_Header header;

		int count = 0;
		while (!feof(st))
		{

			size_t result = fread(&header, sizeof(struct_VehicleInfo_Header), 1, st);

			if (header.vehicle_id < 0)
				break;

			if (header.vehicle_id == 28)
				TRACE("Vehicle ID = %d\n", header.vehicle_id);


			if (header.number_of_nodes != 12)
			{
				TRACE("");

			}

			if (result != 1)  // read end of file
				break;

			DTAVehicle* pVehicle = 0;
			//try
			//{
			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			//if(header.departure_time >= 420)
			//	break;

			////}
			////catch (std::bad_alloc& exc)
			////{
			////	cout << "Insufficient memory...";
			////	getchar();
			////	exit(0);

			////}

			pVehicle->m_VehicleID = header.vehicle_id;
			pVehicle->m_RandomSeed = pVehicle->m_VehicleID;

			pVehicle->m_OriginZoneID = header.from_zone_id;
			pVehicle->m_DestinationZoneID = header.to_zone_id;

			g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
			g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;


			pVehicle->m_DepartureTime = header.departure_time;

			if (g_DemandLoadingEndTimeInMin < pVehicle->m_DepartureTime)
				g_DemandLoadingEndTimeInMin = pVehicle->m_DepartureTime;

			if (g_DemandLoadingStartTimeInMin > pVehicle->m_DepartureTime)
				g_DemandLoadingStartTimeInMin = pVehicle->m_DepartureTime;

			pVehicle->m_PreferredDepartureTime = header.departure_time;
			pVehicle->m_ArrivalTime = header.arrival_time;

			pVehicle->m_TripTime = header.trip_time;

			pVehicle->m_DemandType = header.demand_type;
			pVehicle->m_PricingType = header.pricing_type;

			if (pVehicle->m_PricingType == 0) // unknown type
				pVehicle->m_PricingType = 1;

			pVehicle->m_VehicleType = header.vehicle_type;
			pVehicle->m_InformationClass = header.information_type;
			pVehicle->m_VOT = header.value_of_time;
			pVehicle->m_Age = header.age;


			//
			float vehicle_trip_multiplier_factor = 1.0f;

			if (b_with_updated_demand_type_info)
			{
				int demand_type = -1;
				double RandomPercentage = g_GetRandomRatio() * 100;

				double previous_cumulative_percentage = 0;

				for (std::map<int, DemandType>::iterator iterDemandType = g_DemandTypeMap.begin(); iterDemandType != g_DemandTypeMap.end(); iterDemandType++)
				{

					if (demand_type == -1)  // initialization 
					{
						demand_type = iterDemandType->first; // return pretrip as 2 or enoute as 3
					}

					double cumulative_percentage = iterDemandType->second.cumulative_demand_type_percentage;
					if (RandomPercentage >= previous_cumulative_percentage && RandomPercentage <= cumulative_percentage)
					{
						vehicle_trip_multiplier_factor = iterDemandType->second.vehicle_trip_multiplier_factor;
						demand_type = iterDemandType->first; // return pretrip as 2 or enoute as 3

						previous_cumulative_percentage = cumulative_percentage;

					}
				}

				//	TRACE("vehicle id = %d, demand type = %d\n ", header.vehicle_id, demand_type);

				if (demand_type == -1)
				{
					cout << "Error: demand_type = -1" << endl;
					g_ProgramStop();

				}

				pVehicle->m_DemandType = demand_type;



				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_PricingType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);


			}



			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;

			pVehicle->m_NodeSize = header.number_of_nodes;

			if (header.number_of_nodes >= 1999)

			{
				cout << "Error in reading agent file: header.number_of_node = " << header.number_of_nodes << endl;
				g_ProgramStop();
			}
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = header.complete_flag;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;
			pVehicle->m_NodeNumberSum = 0;

			pVehicle->m_ArrivalTime = header.arrival_time;

			int time_interval = pVehicle->m_DepartureTime / 15;



			if (pVehicle->m_NodeSize >= 1)  // in case reading error
			{
				pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];

				pVehicle->m_NodeNumberSum = 0;
				for (int i = 0; i < pVehicle->m_NodeSize; i++)
				{

					int node_id;
					float event_time_stamp, travel_time, emissions;

					struct_Vehicle_Node node_element;
					fread(&node_element, sizeof(node_element), 1, st);

					path_node_sequence[i] = node_element.NodeName;
					pVehicle->m_NodeNumberSum += path_node_sequence[i];

					if (i == 0)
					{  
						pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];
						pVehicle->m_DepartureTime = node_element.AbsArrivalTimeOnDSN;

						pVehicle->m_LeavingTimeFromLoadingBuffer = node_element.AbsArrivalTimeOnDSN;

					}
						

					if (i == pVehicle->m_NodeSize - 1)
						pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

					if (i >= 1)
					{
						DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
						if (pLink == NULL)
						{
							CString msg;
							msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], header.vehicle_id, file_name.c_str());
							cout << msg << endl;
							fclose(st);
							g_ProgramStop();

							return false;
						}

						if (pLink->GetNumberOfLanes() < 0.01)  // this is a blocked link by work zone
						{
							pVehicle->m_bForcedSwitchAtFirstIteration = true;

						}

						pVehicle->m_Distance += pLink->m_Length;

						pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
						pVehicle->m_LinkAry[i - 1].AbsArrivalTimeOnDSN = node_element.AbsArrivalTimeOnDSN;


					}
					

				}


				//if(g_DemandGlobalMultiplier<0.9999)
				//{
				//	double random_value = g_GetRandomRatio();
				//	if(random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				//	{

				//		delete pVehicle;
				//		continue;
				//	}
				//}


				if (vehicle_trip_multiplier_factor < 0.9999 && b_with_updated_demand_type_info == true)  // we have to run a random number to decide if the vehicles should be added into the simulation or not.
				{

					double RandomRatio = g_GetRandomRatio();
					if (RandomRatio < vehicle_trip_multiplier_factor)
					{

						delete pVehicle;
						pVehicle = NULL;
						continue;  // do not proceed to the remaining steps

					}

				}

				g_VehicleVector.push_back(pVehicle);
				g_VehicleMap[pVehicle->m_VehicleID] = pVehicle;


				count++;

				if (count % 10000 == 0)
					cout << "reading " << count / 1000 << "K agents from binary file " << file_name << endl;
			}
		}
			fclose(st);
		return true;

	}
	else
	{
		cout << "File agent.bin cannot be found. Please check." << endl;
		g_ProgramStop();



	}
	return false;
}


int g_FindAssignmentInterval(int departure_time_begin)
{
	int AssignmentInterval = int(departure_time_begin / g_AggregationTimetInterval);  // starting assignment interval

	if (AssignmentInterval < g_DemandLoadingStartTimeInMin / g_AggregationTimetInterval)
	{
		// reset assignment time interval 
		AssignmentInterval = g_DemandLoadingStartTimeInMin / g_AggregationTimetInterval;
	}
	if (AssignmentInterval >= g_AggregationTimetIntervalSize)
		AssignmentInterval = g_AggregationTimetIntervalSize - 1;


	return AssignmentInterval;

}

void g_AllocateDynamicArrayForVehicles()
{
	if (g_AggregationTimetIntervalSize <= 0)  // has not allocated memory yet
	{
		g_AggregationTimetIntervalSize = max(1, (g_DemandLoadingEndTimeInMin + 1) / g_AggregationTimetInterval + 1);
		g_TDOVehicleArray = AllocateDynamicArray<VehicleArrayForOriginDepartrureTimeInterval>(g_ZoneMap.size(), g_AggregationTimetIntervalSize);

	}
}


int _tmain(int argc, TCHAR* argv[], TCHAR* envp[])
{
	int nRetCode = 0;

	// initialize MFC and print and error on failure
	if (!AfxWinInit(::GetModuleHandle(NULL), NULL, ::GetCommandLine(), 0))
	{
		// TODO: change error code to suit your needs
		_tprintf(_T("Fatal Error: MFC initialization failed\n"));
		nRetCode = 1;

		return 0;
	}

	/**********************************************/
	//below is the main traffic assignment-simulation code



	// if  ./DTASettings.ini does not exist, then we should print out all the default settings for user to change
	//
	g_AppStartTime = CTime::GetCurrentTime();
	g_AppLastIterationStartTime = g_AppStartTime;


	g_ReadInputFiles();


	bool bStartWithEmptyFile = true;
	cout << "     outputing output_agent.csv... " << endl;
	int iteration = 0;
	OutputVehicleTrajectoryData("output_agent_emisssion.csv", "output_trip_emission.csv", iteration, true, false);
	
	//
	//ofstream output_ODMOE_file;

	//cout << "     outputing output_ODMOE.csv... " << endl;
	//output_ODMOE_file.open("output_ODMOE.csv");
	////	output_ODImpact_file.open ("output_ImpactedOD.csv");
	//if (output_ODMOE_file.is_open())
	//{
	//	OutputODMOEData(output_ODMOE_file, 1, 0);
	//	output_ODMOE_file.close();
	//}
	//else
	//{
	//	cout << "Error: File output_ODMOE.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
	//	g_ProgramStop();
	//}

	//ofstream output_MovementMOE_file;
	//cout << "     outputing output_MovementMOE.csv... " << endl;
	//output_MovementMOE_file.open("output_MovementMOE.csv", fstream::app);
	////	output_ODImpact_file.open ("output_ImpactedOD.csv");
	//if (output_MovementMOE_file.is_open())
	//{
	//	OutputMovementMOEData(output_MovementMOE_file);
	//}
	//else
	//{
	//	cout << "Error: File output_MovementMOE.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
	//	g_ProgramStop();
	//}


	//cout << "     outputing output_LinkTDMOE.csv... " << endl;
	//OutputLinkMOEData("output_LinkTDMOE.csv", iteration, bStartWithEmptyFile);


	return nRetCode;
}

char g_get_vehicle_complete_flag(bool flag)
{
	if (flag == true)
		return 'c';
	else
		return 'i';

}


void OutputVehicleTrajectoryData(char fname_agent[_MAX_PATH], char fname_trip[_MAX_PATH], int Iteration, bool bStartWithEmpty, bool bIncremental)
{

	FILE* st_agent = NULL;
	FILE* st_agent_compact = NULL;

	FILE* st_struct = NULL;
	FILE* st_path_link_sequence = NULL;
	FILE* st_path = NULL;

	fopen_s(&st_agent, fname_agent, "w");
	fopen_s(&st_agent_compact, fname_trip, "w");

	if (st_agent_compact == NULL)
	{
		cout << "File " << fname_trip << " cannot be opened." << endl;
		g_ProgramStop();

	}

	int err = 0;
	if (g_VehicleLoadingMode == vehicle_binary_file_mode)
	{
		err = fopen_s(&st_struct, "agent_scenario.bin", "wb");
		if (err > 0)
		{
			cout << "The file 'agent_scenario.bin' was not opened\n";
			g_ProgramStop();
		}
	}
	else
	{
		err = fopen_s(&st_struct, "agent.bin", "wb");

		if (err > 0)
		{
			cout << "The file 'agent.bin' was not opened\n";

			g_ProgramStop();
		}

		DeleteFile("agent_scenario.bin");  // remove agent_scenario.bin as the final result is agent.bin

	}

	//	fopen_s(&st_path_link_sequence,"output_path_link_sequence.csv","w");
	fopen_s(&st_path, "output_path.csv", "w");


	if (st_agent != NULL)
	{
		std::map<int, DTAVehicle*>::iterator iterVM;
		int VehicleCount_withPhysicalPath = 0;
		// output statistics
		fprintf(st_agent, "agent_id,trip_id,from_zone_id,to_zone_id,from_origin__node_id,to_destination_node_id,departure_time,arrival_time,buffer_waiting_time,complete_flag,trip_time,demand_type,pricing_type,vehicle_type,information_type,value_of_time,toll_cost_in_dollar,emissions,distance_in_mile,TotalEnergy_(KJ),CO2_(g),NOX_(g),CO_(g),HC_(g),age,number_of_nodes,path_node_sequence,path_time_sequence,link_travel_time_sequence,path_sequence\n");
		fprintf(st_agent_compact, "agent_id,trip_id,from_zone_id,to_zone_id,from_origin__node_id,to_destination_node_id,start_time_in_min,end_time_in_min,travel_time_in_min,demand_type,pricing_type,information_type,value_of_time,vehicle_type,vehicle_age,distance\n");

		if (st_path_link_sequence != NULL)
			fprintf(st_path_link_sequence, "vehicle_id,from_zone_id,to_zone_id,link_sequence,from_node_id->to_node_id,link_length_in_miles,speed_limit,link_type,with_signal_flag,from_node_time_stamp_in_min,free_flow_travel_time_in_min,travel_time_in_min,delay_in_min\n");

		if (st_path != NULL)
			fprintf(st_path, "vehicle_id,from_zone_id,to_zone_id,departure_time,pricing_type,number_of_nodes,path_sequence\n");

		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
		{

			DTAVehicle* pVehicle = iterVM->second;

			if (pVehicle->m_VehicleID == 2)
				TRACE("");


			if (pVehicle->m_NodeSize >= 2 && pVehicle->m_PricingType != 4)  // with physical path in the network
			{


				if (bIncremental && pVehicle->m_bComplete && pVehicle->b_already_output_flag)  //skip output this vehicle, only under incremental mode, and this vehicle has completed its trip and being output previously in a file
					continue;


				int UpstreamNodeID = 0;
				int DownstreamNodeID = 0;

				int LinkID_0 = pVehicle->m_LinkAry[0].LinkNo;
				UpstreamNodeID = g_LinkVector[LinkID_0]->m_FromNodeID;
				DownstreamNodeID = g_LinkVector[LinkID_0]->m_ToNodeID;

				float TripTime = 0;

				if (pVehicle->m_bComplete)
					TripTime = pVehicle->m_ArrivalTime - pVehicle->m_DepartureTime;


				float m_gap = 0;
				fprintf(st_agent, "%d,%d,%d,%d,%d,%d,%4.2f,%4.2f,%4.2f,%c,%4.2f,%d,%d,%d,%d,%4.2f,%4.3f,%4.3f,%4.3f,%f,%f,%f,%f,%f,%d,%d,",
					pVehicle->m_VehicleID,
					pVehicle->m_ExternalTripID,
					pVehicle->m_OriginZoneID,
					pVehicle->m_DestinationZoneID,
					g_NodeVector[pVehicle->m_OriginNodeID].m_NodeNumber,
					g_NodeVector[pVehicle->m_DestinationNodeID].m_NodeNumber,
					pVehicle->m_DepartureTime,
					pVehicle->m_ArrivalTime,
					0, //buffer_waiting_time
					g_get_vehicle_complete_flag(pVehicle->m_bComplete),
					TripTime,
					pVehicle->m_DemandType,
					pVehicle->m_PricingType,
					pVehicle->m_VehicleType,
					pVehicle->m_InformationClass,
					pVehicle->m_VOT,
					pVehicle->m_TollDollarCost,
					pVehicle->m_Emissions,
					pVehicle->m_Distance,
					pVehicle->Energy,
					pVehicle->CO2, 
					pVehicle->NOX, 
					pVehicle->CO,
					pVehicle->HC, 
					pVehicle->m_Age,
					pVehicle->m_NodeSize);

				fprintf(st_agent_compact, "%d,%d,%d,%d,%d,%d,%4.2f,%4.2f,%4.2f,%d,%d,%d,%4.2f,%d,%d,%4.2f,",
					pVehicle->m_VehicleID, pVehicle->m_ExternalTripID, pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID,
					g_NodeVector[pVehicle->m_OriginNodeID].m_NodeNumber,
					g_NodeVector[pVehicle->m_DestinationNodeID].m_NodeNumber,
					pVehicle->m_DepartureTime,

					pVehicle->m_ArrivalTime,
					pVehicle->m_ArrivalTime - pVehicle->m_DepartureTime,
					pVehicle->m_DemandType, pVehicle->m_PricingType,
					pVehicle->m_InformationClass,
					pVehicle->m_VOT, pVehicle->m_VehicleType, pVehicle->m_Age,
					pVehicle->m_Distance);


				//#ifdef  _large_memory_usage_lr
				//				// vms response 
				//				for(int vl = 0; vl < pVehicle->m_VMSResponseVector.size() ; vl++)
				//				{
				//					int linkid = pVehicle->m_VMSResponseVector[vl].LinkNo ;
				//				fprintf(st_agent,"%d->%d@%d(%d)", g_LinkVector[linkid]->m_FromNodeNumber ,g_LinkVector[linkid]->m_ToNodeNumber, pVehicle->m_VMSResponseVector [vl].ResponseTime ,  pVehicle->m_VMSResponseVector [vl].SwitchFlag );
				//				
				//				}
				//	fprintf(st_agent,",");
				//
				//
				//				// evacuation response 
				//				for(int vl = 0; vl < pVehicle->m_EvacuationResponseVector.size() ; vl++)
				//				{
				//					int linkid = pVehicle->m_EvacuationResponseVector[vl].LinkNo ;
				//				fprintf(st_agent,"%d->%d@%d", g_LinkVector[linkid]->m_FromNodeNumber ,
				//					g_LinkVector[linkid]->m_ToNodeNumber, 
				//					pVehicle->m_EvacuationResponseVector [vl].ResponseTime);
				//				
				//				}
				//#else
				//				fprintf(st_agent,",,");
				//#endif


				if (st_path != NULL)
				{
					fprintf(st_path, "%d,%d,%d,%.1f,%d,%d,", pVehicle->m_VehicleID, pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, pVehicle->m_DepartureTime, pVehicle->m_PricingType, pVehicle->m_NodeSize);
				}
				struct_VehicleInfo_Header header;
				header.day_no = Iteration; // Iteration starts from 0
				header.vehicle_id = pVehicle->m_VehicleID;
				header.from_zone_id = pVehicle->m_OriginZoneID;
				header.to_zone_id = pVehicle->m_DestinationZoneID;
				header.departure_time = pVehicle->m_DepartureTime;
				header.arrival_time = pVehicle->m_ArrivalTime;
				header.complete_flag = pVehicle->m_bComplete;
				header.trip_time = TripTime;
				header.demand_type = pVehicle->m_DemandType;
				header.pricing_type = pVehicle->m_PricingType;
				header.vehicle_type = pVehicle->m_VehicleType;
				header.information_type = pVehicle->m_InformationClass;
				header.value_of_time = pVehicle->m_VOT;
				header.toll_cost_in_dollar = pVehicle->m_TollDollarCost;
				header.emissions = pVehicle->m_Emissions;
				header.distance_in_mile = pVehicle->m_Distance;
				header.Energy = pVehicle->Energy;
				header.CO2 = pVehicle->CO2;
				header.NOX = pVehicle->NOX;
				header.CO = pVehicle->CO;
				header.HC = pVehicle->HC;
				header.number_of_nodes = pVehicle->m_NodeSize;
				header.age = pVehicle->m_Age;
				header.number_of_VMS_response_links = 0;
				header.version_no = 1;
				header.reserverd_field2 = 0;

				struct_VehicleInfo_Header InfoHeaderAsVehicleInput;
				InfoHeaderAsVehicleInput.vehicle_id = pVehicle->m_VehicleID;
				InfoHeaderAsVehicleInput.from_zone_id = pVehicle->m_OriginZoneID;
				InfoHeaderAsVehicleInput.to_zone_id = pVehicle->m_DestinationZoneID;
				InfoHeaderAsVehicleInput.departure_time = pVehicle->m_DepartureTime;
				InfoHeaderAsVehicleInput.demand_type = pVehicle->m_DemandType;
				InfoHeaderAsVehicleInput.pricing_type = pVehicle->m_PricingType;
				InfoHeaderAsVehicleInput.vehicle_type = pVehicle->m_VehicleType;
				InfoHeaderAsVehicleInput.information_type = pVehicle->m_InformationClass;
				InfoHeaderAsVehicleInput.value_of_time = pVehicle->m_VOT;


				fwrite(&header, sizeof(struct_VehicleInfo_Header), 1, st_struct);

				//

				// path_node_sequence 
				int j = 0;

				if (g_LinkVector[pVehicle->m_LinkAry[0].LinkNo] == NULL)
				{

					cout << "Error: vehicle" << pVehicle->m_VehicleID << "at LinkID" << pVehicle->m_LinkAry[0].LinkNo << endl;
					cin.get();  // pause

				}

				int NodeID = g_LinkVector[pVehicle->m_LinkAry[0].LinkNo]->m_FromNodeID;  // first node
				int NodeName = g_NodeVector[NodeID].m_NodeNumber;


				fprintf(st_agent, "%d;", NodeName);
				for (j = 0; j< pVehicle->m_NodeSize - 1; j++)  // for all nodes
				{
					int LinkID = pVehicle->m_LinkAry[j].LinkNo;
					int NodeID = g_LinkVector[LinkID]->m_ToNodeID;
					int NodeName = g_NodeVector[NodeID].m_NodeNumber;
					fprintf(st_agent, "%d;", NodeName);
				}

				fprintf(st_agent, ",");

				// path timestamp sequence

				fprintf(st_agent, "%4.1f;", pVehicle->m_LeavingTimeFromLoadingBuffer);

				for (j = 0; j< pVehicle->m_NodeSize - 1; j++)  // for all nodes
				{
					fprintf(st_agent, "%4.1f;", pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN);
				}
				fprintf(st_agent, ",");

				// time and node sequence

				// link travel time sequence


				float last_timestamp = pVehicle->m_LeavingTimeFromLoadingBuffer;

				for (j = 0; j< pVehicle->m_NodeSize - 1; j++)  // for all nodes
				{
					fprintf(st_agent, "%4.2f;", pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN - last_timestamp);
					last_timestamp = pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN;
				}
				fprintf(st_agent, ",");

				// time and node sequence

				fprintf(st_agent, "\"");

				j = 0;
				if (g_LinkVector[pVehicle->m_LinkAry[0].LinkNo] == NULL)
				{

					cout << "Error: vehicle" << pVehicle->m_VehicleID << "at LinkID" << pVehicle->m_LinkAry[0].LinkNo << endl;
					cin.get();  // pause

				}

				NodeID = g_LinkVector[pVehicle->m_LinkAry[0].LinkNo]->m_FromNodeID;  // first node
				NodeName = g_NodeVector[NodeID].m_NodeNumber;


				//				float link_entering_time = pVehicle->m_DepartureTime;
				float link_entering_time = pVehicle->m_LeavingTimeFromLoadingBuffer;

				fprintf(st_agent, "<%d;%4.2f;0;0>",
					NodeName, link_entering_time);

				if (st_path != NULL)
				{
					fprintf(st_path, "%d,", NodeName);
				}


				struct_Vehicle_Node node_element;
				node_element.NodeName = NodeName;
				node_element.AbsArrivalTimeOnDSN = pVehicle->m_DepartureTime;
				fwrite(&node_element, sizeof(node_element), 1, st_struct);

				float LinkWaitingTime = 0;
				for (j = 0; j< pVehicle->m_NodeSize - 1; j++)  // for all nodes
				{
					int LinkID = pVehicle->m_LinkAry[j].LinkNo;

					if (g_LinkVector[LinkID]->CapacityReductionVector.size() >= 1)
						pVehicle->m_bImpacted = true;

					if (pVehicle->m_bImpacted)
					{
						g_LinkVector[LinkID]->CFlowImpactedCount++;  // count vehicles being impacted
					}





					int NodeID = g_LinkVector[LinkID]->m_ToNodeID;
					int NodeName = g_NodeVector[NodeID].m_NodeNumber;
					float LinkTravelTime = 0;
					float Emissions = 0;

					if (NodeName == 7340)
						TRACE("");

					if (j == 0) // origin node
					{
						//						link_entering_time =  pVehicle->m_DepartureTime;
						link_entering_time = pVehicle->m_LeavingTimeFromLoadingBuffer;
					}
					else
					{
						link_entering_time = pVehicle->m_LinkAry[j - 1].AbsArrivalTimeOnDSN;
					}

					LinkTravelTime = (pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN) - link_entering_time;
					LinkWaitingTime = pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN - link_entering_time - g_LinkVector[LinkID]->m_FreeFlowTravelTime;

					int FromNodeID = g_LinkVector[LinkID]->m_FromNodeID;
					int FromNodeNumber = g_NodeVector[FromNodeID].m_NodeNumber;

					int ToNodeID = g_LinkVector[LinkID]->m_ToNodeID;
					int ToNodeNumber = g_NodeVector[ToNodeID].m_NodeNumber;

					if (j < pVehicle->m_NodeSize - 2 && pVehicle->m_bComplete) // we have not reached the destination yest
					{

						int NextLinkID = pVehicle->m_LinkAry[j + 1].LinkNo;
						int NextNodeID = g_LinkVector[NextLinkID]->m_ToNodeID;
						int DestNodeNumber = g_NodeVector[NextNodeID].m_NodeNumber;

						// construct movement id
						string movement_id = GetMovementStringID(FromNodeNumber, NodeName, DestNodeNumber);
						if (g_NodeVector[ToNodeID].m_MovementMap.find(movement_id) != g_NodeVector[ToNodeID].m_MovementMap.end()) // the capacity for this movement has been defined
						{

							g_NodeVector[ToNodeID].m_MovementMap[movement_id].total_vehicle_count++;
							g_NodeVector[ToNodeID].m_MovementMap[movement_id].total_vehicle_delay += LinkWaitingTime;

							TRACE("movement: %d, %f, link travel time %f\n", g_NodeVector[ToNodeID].m_MovementMap[movement_id].total_vehicle_count, g_NodeVector[ToNodeID].m_MovementMap[movement_id].total_vehicle_delay, LinkWaitingTime);

						}


					}

					if (st_path_link_sequence != NULL)
					{
						//link_length_in_miles,speed_limit,link_type,with_signal_flag,

						if (pVehicle->m_bComplete)
							fprintf(st_path_link_sequence, "v%d,", pVehicle->m_VehicleID);
						else
							fprintf(st_path_link_sequence, "v_incomplete_%d,", pVehicle->m_VehicleID);

						fprintf(st_path_link_sequence, "%d,%d,%d,%d->%d,%.2f,%.0f,%s,%d,%.2f,%.2f,%.2f,%.2f\n",
							pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, j, FromNodeNumber, ToNodeNumber, g_LinkVector[LinkID]->m_Length, g_LinkVector[LinkID]->m_SpeedLimit, g_LinkTypeMap[g_LinkVector[LinkID]->m_link_type].link_type_name.c_str(), g_LinkVector[LinkID]->m_bSignalizedArterialType,
							pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN,
							g_LinkVector[LinkID]->m_FreeFlowTravelTime, LinkTravelTime, LinkWaitingTime);
					}


					//						fprintf(st_agent, ",,,,,,,,,,,,,,%d,%d%,%6.2f,%6.2f,%6.2f\n", j+2,NodeName,pVehicle->m_LinkAry [j].AbsArrivalTimeOnDSN,LinkWaitingTime, g_LinkVector[LinkID]->m_LinkMOEAry [link_entering_time].TravelTime ) ;
					fprintf(st_agent, "<%d; %4.2f;%4.2f;%4.2f>", NodeName, pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN, LinkTravelTime, Emissions);
					if (st_path != NULL)
					{
						fprintf(st_path, "%d,", NodeName);
					}

					node_element.NodeName = NodeName;
					node_element.AbsArrivalTimeOnDSN = pVehicle->m_LinkAry[j].AbsArrivalTimeOnDSN;
					fwrite(&node_element, sizeof(node_element), 1, st_struct);


				} //for all nodes in path
				fprintf(st_agent, "\"\n");
				fprintf(st_path, "\n");
				fprintf(st_agent_compact, "\n");


				// mark this vehicle has output its trip data
				pVehicle->b_already_output_flag = true;



			}
			else
			{// without physical path in the network
				float TripTime = 0;

				fprintf(st_agent, "%d,%d,%d,%d,%d,%d,%4.2f,%4.2f,%d,%4.2f,%d,%d,%d,%d,%4.2f,%4.3f,%4.3f,%4.3f,%f,%f,%f,%f,%f,%d\n",
					pVehicle->m_VehicleID, pVehicle->m_ExternalTripID, pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID,

					g_NodeVector[pVehicle->m_OriginNodeID].m_NodeNumber,
					g_NodeVector[pVehicle->m_DestinationNodeID].m_NodeNumber,

					pVehicle->m_DepartureTime, pVehicle->m_ArrivalTime, pVehicle->m_bComplete, TripTime,
					pVehicle->m_DemandType, pVehicle->m_PricingType, pVehicle->m_VehicleType,
					pVehicle->m_InformationClass,
					pVehicle->m_VOT, pVehicle->m_TollDollarCost, pVehicle->m_Emissions, pVehicle->m_Distance,
					pVehicle->Energy, pVehicle->CO2, pVehicle->NOX, pVehicle->CO, pVehicle->HC,
					pVehicle->m_NodeSize);

				// write everything out, without path sequence
			}
			//if(pVehicle->m_bLoaded == false) 
			//{
			//	cout << "Warning: Not loaded vehicle " << pVehicle->m_VehicleID << " from zone " << 
			//		pVehicle->m_OriginZoneID << " to zone " << pVehicle->m_DestinationZoneID << " departing at"
			//		<< pVehicle->m_DepartureTime << " demand type = " << (int)(pVehicle->m_DemandType) << " Node Size in path = " <<  pVehicle->m_NodeSize << endl;
			//}
		} // for all paths

		// not loaded in simulation


		fclose(st_agent);
		fclose(st_agent_compact);

		fclose(st_struct);


		if (st_path_link_sequence != NULL)
			fclose(st_path_link_sequence);

		if (st_path != NULL)
			fclose(st_path);

	}
	else
	{
		fprintf(g_ErrorFile, "File output_agent_emission.csv cannot be opened. It might be currently used and locked by EXCEL.");
		cout << "Error: File output_agent_emission.csv cannot be opened.\n It might be currently used and locked by EXCEL." << endl;
		getchar();
	}
}
