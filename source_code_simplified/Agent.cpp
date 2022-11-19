#include "stdafx.h"
#include <stdlib.h>
#include <crtdbg.h>

#include "DTALite.h"


#include "Geometry.h"
#include "GlobalData.h"
#include "CSVParser.h"
#include "SafetyPlanning.h"
#include <iostream>
#include <fstream>
#include <omp.h>
#include <algorithm>

using namespace std;
void g_AllocateDynamicArrayForVehicles()
{
	if (g_AggregationTimetIntervalSize <= 0)  // has not allocated memory yet
	{
		g_AggregationTimetIntervalSize = max(1, (g_DemandLoadingEndTimeInMin + 1) / g_AggregationTimetInterval + 1);
		g_TDOVehicleArray = AllocateDynamicArray<VehicleArrayForOriginDepartrureTimeInterval>(g_ZoneMap.size(), g_AggregationTimetIntervalSize);

	}
}
vector<int> ParseLineToIntegers(string line)
{
	vector<int> SeperatedIntegers;
	string subStr;
	istringstream ss(line);


	char Delimiter = ';';


	while (std::getline(ss, subStr, Delimiter))
	{
		int integer = atoi(subStr.c_str());
		SeperatedIntegers.push_back(integer);
	}
	return SeperatedIntegers;
}


void g_ReadDSPVehicleFile(string file_name)
{
	if (g_TrafficFlowModelFlag == tfm_BPR)  //BRP  // static assignment parameters
	{
		g_AggregationTimetInterval = 60;
	}

	g_AllocateDynamicArrayForVehicles();
	/*     5311320           1    # of vehicles in the file, Max # of stops
	#   usec   dsec   stime vehcls vehtype ioc #ONode #IntDe info ribf    comp   izone Evac InitPos    VoT  tFlag pArrTime TP IniGas
	1  11261   3248    0.00     3     1     1     1     1     0  0.0000  1.0000 1110    0  0.26324894    1.12    2   21.0    7  0.0
	294  20.59

	*/

	FILE* st = NULL;

	fopen_s(&st, file_name.c_str(), "r"); /// 
	if (st != NULL)
	{
		cout << "Reading file " << file_name << " ..." << endl;
		g_LogFile << "Reading file " << file_name << endl;
		int count = 0;

		//# of vehicles in the file 
		g_read_integer(st);
		// Max # of stops 
		g_read_integer(st);
		float total_number_of_vehicles_to_be_generated = 0;

		int i = 0;
		int line_no = 1;
		while (true)
		{

			line_no += 2;

			if (line_no % 1000 == 0)
				cout << "loading " << line_no / 1000 << " k lines" << endl;

			// #
			int agent_id = g_read_integer(st);

			if (agent_id < 0)
				break;

			DTAVehicle* pVehicle = 0;

			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			pVehicle->m_VehicleID = i;
			pVehicle->m_RandomSeed = pVehicle->m_VehicleID;

			// 	usec 

			int origin_node_number = g_read_integer(st);
			pVehicle->m_OriginNodeID = g_NodeNametoIDMap[origin_node_number];

			//dsec  

			g_read_integer(st);

			//stime 

			pVehicle->m_DepartureTime = g_read_float(st);


			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: agent " << agent_id << " in file " << file_name << " has a departure time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please check!";
				g_ProgramStop();
			}



			//vehicle class
			pVehicle->m_PricingType = g_read_integer(st);

			pVehicle->m_DemandType = pVehicle->m_PricingType;
			//vehicle type 

			pVehicle->m_VehicleType = g_read_integer(st);
			//information class  

			pVehicle->m_InformationClass = g_read_integer(st);


			//#ONode 
			g_read_integer(st);
			// #IntDe 
			g_read_integer(st);
			//info 
			g_read_integer(st);

			//ribf  
			g_read_float(st);
			//comp 
			g_read_float(st);
			//izone 

			pVehicle->m_OriginZoneID = g_read_integer(st);

			//Evac
			float evac_value = g_read_float(st);
			//InitPos 
			g_read_float(st);
			//VoT 
			pVehicle->m_VOT = g_read_float(st);
			//tFlag 
			g_read_float(st);
			//pArrTime 
			float PATvalue = g_read_float(st);
			//TP 
			float TP_value = g_read_float(st);
			//IniGas
			float value = g_read_float(st);

			pVehicle->m_DestinationZoneID = g_read_integer(st);




			// stop time?
			float travel_time_value = g_read_float(st);


			pVehicle->m_DestinationNodeID = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_VehicleID % 100) / 100.0f);;

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}


			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;

			pVehicle->m_NodeSize = 0;

			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;

			if (pVehicle->m_OriginZoneID == pVehicle->m_DestinationZoneID)
			{  // do not simulate intra zone traffic
				continue;
			}
			if (g_DemandGlobalMultiplier<0.9999)
			{
				double random_value = g_GetRandomRatio();
				if (random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				{

					delete pVehicle;
					continue;
				}
			}

			g_VehicleVector.push_back(pVehicle);
			g_VehicleMap[i] = pVehicle;

			int AssignmentInterval = g_FindAssignmentInterval(pVehicle->m_DepartureTime);

			g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_VehicleID);


			i++;

		}

	}
	else
	{
		cout << "File " << file_name << " cannot be opened. Please check." << endl;
		g_ProgramStop();

	}

}

bool AddPathToVehicle(DTAVehicle * pVehicle, std::vector<int> path_node_sequence, CString FileName)
{

	if (pVehicle->m_NodeSize >= 1 && pVehicle->m_LinkAry != NULL)
	{
		delete pVehicle->m_LinkAry;
	}

	pVehicle->m_NodeSize = path_node_sequence.size();

	if (pVehicle->m_NodeSize >= 1)  // in case reading error
	{
		pVehicle->m_LinkAry = new SVehicleLink[pVehicle->m_NodeSize];
		pVehicle->m_NodeNumberSum = 0;
	
		pVehicle->m_Distance = 0;  // reset distanace when there are new paths assigned. 
		for (int i = 0; i < pVehicle->m_NodeSize; i++)
		{

			int node_id;
			float event_time_stamp, travel_time, emissions;

			pVehicle->m_NodeNumberSum += path_node_sequence[i];

			if (i == 0)
				pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

			if (i == pVehicle->m_NodeSize - 1)
				pVehicle->m_DestinationNodeID = g_NodeNametoIDMap[path_node_sequence[pVehicle->m_NodeSize - 1]];

			if (i >= 1)
			{
				DTALink* pLink = g_LinkMap[GetLinkStringID(path_node_sequence[i - 1], path_node_sequence[i])];
				if (pLink == NULL && FileName.GetLength() > 0)
				{
					CString msg;
					msg.Format("Error in reading link %d->%d for vehicle id %d  in file %s.", path_node_sequence[i - 1], path_node_sequence[i], pVehicle->m_VehicleID, FileName);
					cout << msg << endl;

					return false;
				}

				pVehicle->m_Distance += pLink->m_Length;

				pVehicle->m_LinkAry[i - 1].LinkNo = pLink->m_LinkNo; // start from 0
			}


		}

	}
	return true;
}


void g_ReadDTALiteAgentCSVFile(string file_name)
{
	bool ecaculation_modeling_flag = false;

	if (file_name.find("evacuation") != string::npos)
		ecaculation_modeling_flag = true;

	bool agent_group_flag = false;

	if (file_name.find("group") != string::npos)
		agent_group_flag = true;

	if (g_TrafficFlowModelFlag == tfm_BPR)  //BRP  // static assignment parameters
	{
		g_AggregationTimetInterval = 60;
	}

	g_AllocateDynamicArrayForVehicles();
	CCSVParser parser_agent;

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name))
	{

		cout << "reading file " << file_name << endl;

		int line_no = 1;

		int i = 0;
		while (parser_agent.ReadRecord())
		{

			int agent_id = 0;

			parser_agent.GetValueByFieldNameRequired("agent_id", agent_id);
			DTAVehicle* pVehicle = 0;


			if (g_DemandGlobalMultiplier<0.9999)
			{
				double random_value = g_GetRandomRatio();
				if (random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				{

					continue;
				}
			}
			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			pVehicle->m_VehicleID = i;
			pVehicle->m_RandomSeed = pVehicle->m_VehicleID;

			parser_agent.GetValueByFieldNameRequired("from_zone_id", pVehicle->m_OriginZoneID);
			parser_agent.GetValueByFieldNameRequired("to_zone_id", pVehicle->m_DestinationZoneID);

			int origin_node_id = -1;
			int origin_node_number = -1;

			parser_agent.GetValueByFieldName("origin_node_id", origin_node_number);

			if (g_NodeNametoIDMap.find(origin_node_number) != g_NodeNametoIDMap.end())  // convert node number to internal node id
			{
				origin_node_id = g_NodeNametoIDMap[origin_node_number];
			}

			int destination_node_id = -1;
			int destination_node_number = -1;
			parser_agent.GetValueByFieldName("destination_node_id", destination_node_number);

			if (g_NodeNametoIDMap.find(destination_node_number) != g_NodeNametoIDMap.end()) // convert node number to internal node id
			{
				destination_node_id = g_NodeNametoIDMap[destination_node_number];
			}

			if (origin_node_id == -1)  // no default origin node value, re-generate origin node
				origin_node_id = g_ZoneMap[pVehicle->m_OriginZoneID].GetRandomOriginNodeIDInZone((pVehicle->m_VehicleID % 100) / 100.0f);  // use pVehicle->m_VehicleID/100.0f as random number between 0 and 1, so we can reproduce the results easily

			if (destination_node_id == -1)// no default destination node value, re-destination origin node
				destination_node_id = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_VehicleID % 100) / 100.0f);

			pVehicle->m_OriginNodeID = origin_node_id;
			pVehicle->m_DestinationNodeID = destination_node_id;


			if (origin_node_id == destination_node_id)
			{  // do not simulate intra zone traffic
				continue;
			}

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}

			float departure_time = 0;
			parser_agent.GetValueByFieldNameRequired("departure_time", departure_time);

			pVehicle->m_DepartureTime = departure_time;
			int beginning_departure_time = departure_time;


			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: agent " << agent_id << " in file " << file_name << " has a departure time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt";
				g_ProgramStop();
			}

			parser_agent.GetValueByFieldNameRequired("demand_type", pVehicle->m_DemandType);
			parser_agent.GetValueByFieldNameRequired("pricing_type", pVehicle->m_PricingType);
			parser_agent.GetValueByFieldNameRequired("vehicle_type", pVehicle->m_VehicleType);
			parser_agent.GetValueByFieldNameRequired("information_type", pVehicle->m_InformationClass);
			parser_agent.GetValueByFieldNameRequired("value_of_time", pVehicle->m_VOT);
			parser_agent.GetValueByFieldNameRequired("vehicle_age", pVehicle->m_Age);


			pVehicle->m_attribute_update_time_in_min = -1;
			parser_agent.GetValueByFieldName("time_for_attribute_updating", pVehicle->m_attribute_update_time_in_min);
			parser_agent.GetValueByFieldName("updated_to_zone_id", pVehicle->m_DestinationZoneID_Updated);

			if (pVehicle->m_attribute_update_time_in_min >= 0)  // there are vehicles' attributes need to be updated
			{

				if (g_ZoneMap.find(pVehicle->m_DestinationZoneID_Updated) == g_ZoneMap.end())
				{


					cout << "updated_to_zone_id " << pVehicle->m_DestinationZoneID_Updated << " does not exist for vehicle " << pVehicle->m_VehicleID << endl;
					g_ProgramStop();


				}
				g_bVehicleAttributeUpdatingFlag = true;
				g_bInformationUpdatingAndReroutingFlag = true;

			}


			int number_of_nodes = 0;
			parser_agent.GetValueByFieldNameRequired("number_of_nodes", number_of_nodes);

			std::vector<int> path_node_sequence;
			if (number_of_nodes >= 2)
			{
				string path_node_sequence_str;
				parser_agent.GetValueByFieldNameRequired("path_node_sequence", path_node_sequence_str);

				path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

				AddPathToVehicle(pVehicle, path_node_sequence, file_name.c_str());
			}

			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;

			pVehicle->m_NodeSize = 0;

			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;

			int number_of_agents = 1;

			float ending_departure_time = 0;
			if (agent_group_flag == true)
			{
				parser_agent.GetValueByFieldName("number_of_agents", number_of_agents);

				ending_departure_time = departure_time;

				parser_agent.GetValueByFieldName("ending_departure_time", ending_departure_time);
			}


			for (int agent_i = 0; agent_i < number_of_agents; agent_i++)
			{

				if (agent_i >= 1 && agent_group_flag)
				{

					DTAVehicle* pNewVehicle = new (std::nothrow) DTAVehicle;
					if (pNewVehicle == NULL)
					{
						cout << "Insufficient memory...";
						getchar();
						exit(0);
					}

					pNewVehicle = pVehicle;  // copy all attributes;

					// use new departure time 
					float departure_time = beginning_departure_time + (ending_departure_time - beginning_departure_time)*agent_i / number_of_agents + 1;
					pNewVehicle->m_DepartureTime = departure_time;

					// add node sequence 

					//if(number_of_nodes>=2)
					//{
					//AddPathToVehicle(pVehicle, path_node_sequence,file_name.c_str ());
					//}


					g_VehicleVector.push_back(pNewVehicle);
					g_VehicleMap[i] = pNewVehicle;



				}
				else
				{
					g_VehicleVector.push_back(pVehicle);
					g_VehicleMap[pVehicle->m_VehicleID] = pVehicle;

				}





				int AssignmentInterval = g_FindAssignmentInterval(pVehicle->m_DepartureTime);

				ASSERT(pVehicle->m_OriginZoneID <= g_ODZoneNumberSize);

				g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_VehicleID);

				i++;
			}


			line_no++;
		}



		cout << i << " agents have been read from file " << file_name << endl;

	}
	else
	{
		cout << "File " << file_name << " cannot be opened. Please check." << endl;
		g_ProgramStop();

	}

}

void g_UseExternalPath(DTAVehicle* pVehicle)
{

	if (g_ODPathSetVector == NULL)
		return;

	int OrgZoneSequentialNo = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
	int DestZoneSequentialNo = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

	float random_value = pVehicle->GetRandomRatio();

	// loop through the path set
	if (g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet.size() >= 1)
	{
		int i = 0;
		for (i = 0; i < g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet.size(); i++)
		{

			if (random_value <= g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet[i].CumulativeRatio)
			{
				i = i - 1;
				break;
			}

		}

		if (i < 0)
			i = 0;

		if (i == g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet.size())
			i = g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet.size() - 1;

		AddPathToVehicle(pVehicle, g_ODPathSetVector[OrgZoneSequentialNo][DestZoneSequentialNo].PathSet[i].m_NodeNumberArray, NULL);

	}

}
bool g_ReadTripCSVFile(string file_name, bool bOutputLogFlag, int &LineCount)
{
	LineCount = 0;

	g_AllocateDynamicArrayForVehicles();
	float start_time_value = -100;

	CCSVParser parser_agent;

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name, false))
	{

		if (bOutputLogFlag)
		{
			cout << "reading file " << file_name << endl;
		}
		int line_no = 1;

		int i = 0;

		int count = 0;

		int count_for_sameOD = 0;
		int count_for_not_defined_zones = 0;

		while (parser_agent.ReadRecord())
		{

			if ((count + 1) % 1000 == 0 && bOutputLogFlag)
			{
				cout << "reading " << count + 1 << " records..." << endl;

			}
			count++;

			int trip_id = 0;

			parser_agent.GetValueByFieldNameRequired("trip_id", trip_id);
			DTAVehicle* pVehicle = 0;

			pVehicle = new (std::nothrow) DTAVehicle;
			if (pVehicle == NULL)
			{
				cout << "Insufficient memory...";
				getchar();
				exit(0);

			}

			pVehicle->m_VehicleID = g_VehicleVector.size();

			pVehicle->m_ExternalTripID = trip_id;


			pVehicle->m_RandomSeed = pVehicle->m_VehicleID;

			parser_agent.GetValueByFieldNameRequired("from_zone_id", pVehicle->m_OriginZoneID);
			parser_agent.GetValueByFieldNameRequired("to_zone_id", pVehicle->m_DestinationZoneID);

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) == g_ZoneMap.end() || g_ZoneMap.find(pVehicle->m_DestinationZoneID) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;

				continue;
			}



			int origin_node_id = -1;
			int origin_node_number = -1;

			parser_agent.GetValueByFieldName("origin_node_id", origin_node_number);

			if (g_NodeNametoIDMap.find(origin_node_number) != g_NodeNametoIDMap.end())  // convert node number to internal node id
			{
				origin_node_id = g_NodeNametoIDMap[origin_node_number];
			}

			int destination_node_id = -1;
			int destination_node_number = -1;
			parser_agent.GetValueByFieldName("destination_node_id", destination_node_number);

			if (g_NodeNametoIDMap.find(destination_node_number) != g_NodeNametoIDMap.end()) // convert node number to internal node id
			{
				destination_node_id = g_NodeNametoIDMap[destination_node_number];
			}

			if (origin_node_id == -1)  // no default origin node value, re-generate origin node
				origin_node_id = g_ZoneMap[pVehicle->m_OriginZoneID].GetRandomOriginNodeIDInZone((pVehicle->m_VehicleID % 100) / 100.0f);  // use pVehicle->m_VehicleID/100.0f as random number between 0 and 1, so we can reproduce the results easily

			if (destination_node_id == -1)// no default destination node value, re-destination origin node
				destination_node_id = g_ZoneMap[pVehicle->m_DestinationZoneID].GetRandomDestinationIDInZone((pVehicle->m_VehicleID % 100) / 100.0f);

			pVehicle->m_OriginNodeID = origin_node_id;
			pVehicle->m_DestinationNodeID = destination_node_id;


			if (origin_node_id == destination_node_id)
			{  // do not simulate intra zone traffic

				count_for_sameOD++;
				continue;
			}

			if (g_ZoneMap.find(pVehicle->m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[pVehicle->m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[pVehicle->m_OriginZoneID].m_OriginVehicleSize += 1;

			}

			float departure_time = 0;
			parser_agent.GetValueByFieldNameRequired("start_time_in_min", departure_time);


			if (start_time_value < 0)  // set first value
				start_time_value = departure_time;
			else if (start_time_value > departure_time + 0.00001)  // check if the departure times are sequential
			{
				departure_time = start_time_value; // use a larger value 
				start_time_value = departure_time;
			}

			pVehicle->m_DepartureTime = departure_time;
			int beginning_departure_time = departure_time;


			if (pVehicle->m_DepartureTime < g_DemandLoadingStartTimeInMin || pVehicle->m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				cout << "Error: trip_id " << trip_id << " in file " << file_name << " has a start time of " << pVehicle->m_DepartureTime << ", which is out of the demand loading range: " <<
					g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt";
				g_ProgramStop();
			}

			if (parser_agent.GetValueByFieldName("demand_type", pVehicle->m_DemandType) == true)
			{

				g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_PricingType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);

				// if there are values in the file, then update the related attributes; 
				int VOT = 0;
				int PricingType = 0;
				int VehicleType = 0;

				parser_agent.GetValueByFieldName("pricing_type", PricingType);
				
				if (PricingType >= 1)
					pVehicle->m_PricingType = PricingType;

				
				parser_agent.GetValueByFieldName("vehicle_type", VehicleType);

				if (VehicleType >=1)
				{
				
					pVehicle->m_VehicleType = VehicleType;
				}


				parser_agent.GetValueByFieldName("information_type", pVehicle->m_InformationClass); //default is 0;
				parser_agent.GetValueByFieldName("value_of_time", VOT);

				if (VOT >= 1)  // only with valid value
					pVehicle->m_VOT = VOT;

				parser_agent.GetValueByFieldName("vehicle_age", pVehicle->m_Age);
			}
			else
			{


			}

			int number_of_nodes = 0;
			parser_agent.GetValueByFieldName("number_of_nodes", number_of_nodes);

			std::vector<int> path_node_sequence;
			if (number_of_nodes >= 2)  // condition 1: user external input in the min by min trip file
			{
				string path_node_sequence_str;
				parser_agent.GetValueByFieldName("path_node_sequence", path_node_sequence_str);

				path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

				AddPathToVehicle(pVehicle, path_node_sequence, file_name.c_str());
			}
			else if (g_use_routing_policy_from_external_input) // condition 2: use routing policy from external input (saved from pervious iterations)
			{
				
						g_UseExternalPath(pVehicle);
			}   // condition 3: no path is available from external input, before the simulation, DTALite will calculate the path into 
			

			pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;

			pVehicle->m_NodeSize = 0;

			pVehicle->m_NodeNumberSum = 0;
			pVehicle->m_Distance = 0;

			int number_of_agents = 1;

			float ending_departure_time = 0;

			g_VehicleVector.push_back(pVehicle);
			g_VehicleMap[pVehicle->m_VehicleID] = pVehicle;

			int AssignmentInterval = g_FindAssignmentInterval(pVehicle->m_DepartureTime);

			ASSERT(pVehicle->m_OriginZoneID <= g_ODZoneNumberSize);

			g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_VehicleID);


			i++;
		}


		line_no++;



		if (bOutputLogFlag)
		{

			cout << count << " records have been read from file " << file_name << endl;

			cout << i << " agents have been read from file " << file_name << endl;

			if (count_for_sameOD >= 1)
				cout << "there are " << count_for_sameOD << " agents with the same from_zone_id and to_zone_id, which will not be simulated. " << endl;


			if (count_for_not_defined_zones >= 1)
				cout << "there are " << count_for_not_defined_zones << " agents with zones not being defined in input_zone.csv file, which will not be simulated. " << endl;

		}
		LineCount = count;
	}
	else
	{
		cout << "Waiting for file " << file_name << "... " << endl;

		return false;
	}

	return true;

}

bool g_ReadTRANSIMSTripFile(string file_name, bool bOutputLogFlag)
{
	g_AllocateDynamicArrayForVehicles();

	CCSVParser parser_agent;


	parser_agent.Delimiter = '\t';

	float total_number_of_vehicles_to_be_generated = 0;

	if (parser_agent.OpenCSVFile(file_name, false))
	{

		if (bOutputLogFlag)
		{
			cout << "reading file " << file_name << endl;
		}
		int line_no = 1;

		int i = 0;

		int count = 0;

		int count_for_sameOD = 0;
		int count_for_not_defined_zones = 0;

		while (parser_agent.ReadRecord())
		{

			if ((count + 1) % 1000 == 0 && bOutputLogFlag)
			{
				cout << "reading " << count + 1 << " records..." << endl;

			}
			count++;

			if (g_DemandGlobalMultiplier<0.9999)
			{
				double random_value = g_GetRandomRatio();
				if (random_value>g_DemandGlobalMultiplier) // if random value is less than demand multiplier, then skip, not generate vehicles
				{

					continue;
				}
			}

			int trip_id = 0;

			DTA_vhc_simple vhc;

			parser_agent.GetValueByFieldNameRequired("ORIGIN", vhc.m_OriginZoneID);
			parser_agent.GetValueByFieldNameRequired("DESTINATION", vhc.m_DestinationZoneID);

			if (g_ZoneMap.find(vhc.m_OriginZoneID) == g_ZoneMap.end() || g_ZoneMap.find(vhc.m_DestinationZoneID) == g_ZoneMap.end())
			{
				count_for_not_defined_zones++;

				continue;
			}



			if (vhc.m_OriginZoneID == vhc.m_DestinationZoneID)
			{  // do not simulate intra zone traffic

				count_for_sameOD++;
				continue;
			}

			if (g_ZoneMap.find(vhc.m_OriginZoneID) != g_ZoneMap.end())
			{
				g_ZoneMap[vhc.m_OriginZoneID].m_Demand += 1;
				g_ZoneMap[vhc.m_OriginZoneID].m_OriginVehicleSize += 1;

			}


			std::string START_TIME;
			parser_agent.GetValueByFieldNameRequired("START", START_TIME);



			if (START_TIME.find(":") != std::string::npos)
			{
				int hour, min, second;
				sscanf(START_TIME.c_str(), "%d:%d:%d", &hour, &min, &second);
				vhc.m_DepartureTime = hour * 60 + min + second / 60.0;
			}
			else
			{
				float min;

				sscanf(START_TIME.c_str(), "%f", &min);

				vhc.m_DepartureTime = min;

			}


			if (vhc.m_DepartureTime < g_DemandLoadingStartTimeInMin || vhc.m_DepartureTime > g_DemandLoadingEndTimeInMin)
			{

				//cout << "Error: trip_id " <<  trip_id << " in file " << file_name << " has a start time of " << vhc.m_DepartureTimeIndex  << ", which is out of the demand loading range: " << 
				//	g_DemandLoadingStartTimeInMin << "->" << g_DemandLoadingEndTimeInMin << " (min)." << endl << "Please change the setting in section agent_input, demand_loading_end_time_in_min in file DTASettings.txt" ;

				continue;
			}

			//parser_agent.GetValueByFieldName("demand_type",pVehicle->m_DemandType);
			//parser_agent.GetValueByFieldName("pricing_type",pVehicle->m_PricingType);
			//parser_agent.GetValueByFieldName("vehicle_type",pVehicle->m_VehicleType);
			//parser_agent.GetValueByFieldName("information_type",pVehicle->m_InformationClass);
			//parser_agent.GetValueByFieldName("value_of_time",pVehicle->m_VOT);
			//parser_agent.GetValueByFieldName("vehicle_age",pVehicle->m_Age );


			/*		int number_of_nodes = 0;
			parser_agent.GetValueByFieldName("number_of_nodes",number_of_nodes );

			std::vector<int> path_node_sequence;
			if(number_of_nodes >=2)
			{
			string path_node_sequence_str;
			parser_agent.GetValueByFieldName("path_node_sequence",path_node_sequence_str);

			path_node_sequence = ParseLineToIntegers(path_node_sequence_str);

			AddPathToVehicle(pVehicle, path_node_sequence,file_name.c_str ());
			}*/

			//pVehicle->m_TimeToRetrieveInfo = pVehicle->m_DepartureTime;
			//pVehicle->m_ArrivalTime  = 0;
			//pVehicle->m_bComplete = false;
			//pVehicle->m_bLoaded  = false;
			//pVehicle->m_TollDollarCost = 0;
			//pVehicle->m_Emissions  = 0;
			//pVehicle->m_Distance = 0;

			//pVehicle->m_NodeSize = 0;

			//pVehicle->m_NodeNumberSum =0;
			//pVehicle->m_Distance =0;

			line_no++;

			//

			int demand_type = 1;
			vhc.m_DemandType = demand_type;


			g_GetVehicleAttributes(vhc.m_DemandType, vhc.m_VehicleType, vhc.m_PricingType, vhc.m_InformationClass, vhc.m_VOT, vhc.m_Age);

			g_simple_vector_vehicles.push_back(vhc);

			//// debug info
			//if(g_simple_vector_vehicles.size() == 100000)
			//	break;



		}
		if (bOutputLogFlag)
		{

			cout << count << " records have been read from file " << file_name << endl;

			cout << i << " agents have been read from file " << file_name << endl;

			if (count_for_sameOD >= 1)
				cout << "there are " << count_for_sameOD << " agents with the same from_zone_id and to_zone_id, which will not be simulated. " << endl;


			if (count_for_not_defined_zones >= 1)
				cout << "there are " << count_for_not_defined_zones << " agents with zones not being defined in input_zone.csv file, which will not be simulated. " << endl;

		}

	}
	else
	{
		cout << "Waiting for file " << file_name << "... " << endl;

		return false;
	}

	return true;

}


void g_ReadScenarioFilesUnderAgentBinaryMode()
{
	CCSVParser parser_demand_type;

	if (parser_demand_type.OpenCSVFile("Scenario_Demand_Type.csv"))
	{
		g_DemandTypeMap.clear();
		float cumulative_demand_type_percentage = 0;

		while (parser_demand_type.ReadRecord())
		{


			DemandType element;
			int demand_type = 1;
			int pricing_type = 0;
			float average_VOT = 10;

			if (parser_demand_type.GetValueByFieldName("demand_type", demand_type) == false)
				break;

			float demand_type_percentage = 0;
			if (parser_demand_type.GetValueByFieldName("demand_type_percentage", element.demand_type_percentage) == false)
			{
				cout << "Field demand_type_percentage is missing in Scenario_Demand_Type.csv. Please check." << endl;
				g_ProgramStop();
				break;

			}


			if (parser_demand_type.GetValueByFieldName("vehicle_trip_multiplier_factor", element.vehicle_trip_multiplier_factor) == false)
			{
				cout << "Field vehicle_trip_multiplier_factor is missing in Scenario_Demand_Type.csv. Please check." << endl;
				g_ProgramStop();
				break;

			}




			cumulative_demand_type_percentage += element.demand_type_percentage;

			element.cumulative_demand_type_percentage = cumulative_demand_type_percentage;

			float ratio_pretrip = 0;
			float ratio_enroute = 0;
			float ratio_personalized_info = 0;
			float ratio_eco_so_info = 0;

			if (parser_demand_type.GetValueByFieldName("pricing_type", pricing_type) == false)
				break;

			if (pricing_type < 0)
			{
				cout << "Error: Field pricing_type " << pricing_type << " should be >=1 in the Secnario_Demand_Type.csv file.";
				g_ProgramStop();
			}

			if (pricing_type == 4 && element.demand_type_percentage >=1)
			{
				cout << "Error: Transit type is not supported in the agent file reshuffling. Please change demand_type_percentage = 0 for pricing_type = 4 (transit) in the Secnario_Demand_Type.csv file.";
				g_ProgramStop();
			}

			if (pricing_type != 4 && element.vehicle_trip_multiplier_factor >= 1.001)
			{
				cout << "Field vehicle_trip_multiplier_factor should be <=1 in Scenario_Demand_Type.csv. Please check." << endl;
				g_ProgramStop();
			}
			parser_demand_type.GetValueByFieldName("percentage_of_pretrip_info", ratio_pretrip);
			parser_demand_type.GetValueByFieldName("percentage_of_enroute_info", ratio_enroute);
			parser_demand_type.GetValueByFieldName("percentage_of_personalized_info", ratio_personalized_info);
			parser_demand_type.GetValueByFieldName("percentage_of_eco_so_info", ratio_eco_so_info);

			if (ratio_eco_so_info >= 1)
			{
				g_EmissionDataOutputFlag = 2;  //enable emission output at each iteration
			
			}

			element.demand_type = demand_type;

			parser_demand_type.GetValueByFieldName("demand_type_name", element.demand_type_name);

			element.pricing_type = pricing_type;

			element.info_class_percentage[1] = 0;  //learning 
			element.info_class_percentage[2] = ratio_pretrip;
			element.info_class_percentage[3] = ratio_enroute;
			element.info_class_percentage[4] = ratio_personalized_info;
			element.info_class_percentage[5] = ratio_eco_so_info;

			

			 
			element.info_class_percentage[0] = 100 - ratio_enroute - ratio_pretrip - ratio_personalized_info - ratio_eco_so_info;

			for (int ic = 0; ic < MAX_INFO_CLASS_SIZE; ic++)
			{
				element.cumulative_info_class_percentage[ic] = element.cumulative_info_class_percentage[ic - 1] + element.info_class_percentage[ic];
			}
			for (int i = 0; i < g_VehicleTypeVector.size(); i++)
			{
				std::ostringstream  str_percentage_of_vehicle_type;
				str_percentage_of_vehicle_type << "percentage_of_vehicle_type" << i + 1;

				float percentage_vehicle_type = 0;
				if (parser_demand_type.GetValueByFieldName(str_percentage_of_vehicle_type.str(), percentage_vehicle_type) == false)
				{
					cout << "Error: Field percentage_of_vehicle_type " << i + 1 << " cannot be found in the Scenario_Demand_Type.csv file.";
					cout << "In file Scenario_Vehicle_Type.csv, " << g_VehicleTypeVector.size() << " have been defined, so Scenario_Demand_Type.csv should percentage_of_vehicle_type for all vehicle types.";


					g_ProgramStop();
					return;
				}
				else
				{
					element.vehicle_type_percentage[i + 1] = percentage_vehicle_type;

					element.cumulative_type_percentage[i + 1] = element.cumulative_type_percentage[i] + percentage_vehicle_type;

				}
			}


			g_DemandTypeMap[demand_type] = element;

		}

if (cumulative_demand_type_percentage >= 99 && cumulative_demand_type_percentage <= 101)
cumulative_demand_type_percentage = 100;
else
{
	cout << "Error: Sum of demand_type_percentage =  " << cumulative_demand_type_percentage  << " which should be 100 in the Scenario_Demand_Type.csv file.";

}

	}
	else
	{

		cout << "Error: File Scenario_Demand_Type.csv cannot be opened.\nThis file is required when setting format_type = agent_bin_with_updated_demand_vehicle_type_info in file input_demand_meta_data.csv." << endl;
		g_ProgramStop();
	}


	// reading updated vehicle type 

	CCSVParser parser_vehicle_type;

	if (parser_vehicle_type.OpenCSVFile("Scenario_Vehicle_Type.csv"))
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
		cout << "Sceanrio_Vehicle_Type.csv cannot be opened.\nThis file is required when setting format_type = agent_bin_with_updated_demand_vehicle_type_info in file input_demand_meta_data.csv." << endl;
		g_ProgramStop();

	}

	// reading VOT

	CCSVParser parser_VOT;

	double cumulative_percentage = 0;

	if (parser_VOT.OpenCSVFile("Scenario_VOT.csv"))
	{
		int i = 0;
		int old_demand_type = 0;
		while (parser_VOT.ReadRecord())
		{
			int demand_type = 0;

			if (parser_VOT.GetValueByFieldName("demand_type", demand_type) == false)
				break;

			if (demand_type != old_demand_type)
				cumulative_percentage = 0;   //switch vehicle type, reset cumulative percentage


			int VOT;
			if (parser_VOT.GetValueByFieldName("VOT_dollar_per_hour", VOT) == false)
				break;

			float percentage;
			if (parser_VOT.GetValueByFieldName("percentage", percentage) == false)
				break;

			old_demand_type = demand_type;
			VOTDistribution element;
			element.demand_type = demand_type;
			element.percentage = percentage;
			element.VOT = VOT;
			element.cumulative_percentage_LB = cumulative_percentage;
			cumulative_percentage += percentage;
			element.cumulative_percentage_UB = cumulative_percentage;
			//			TRACE("Pricing type = %d, [%f,%f]\n",pricing_type,element.cumulative_percentage_LB,element.cumulative_percentage_UB);
			g_VOTDistributionVector.push_back(element);

		}

	}
	else
	{
		cout << "Scenario_VOT.csv cannot be opened." << endl;
		g_ProgramStop();
	}

}
bool g_ReadAgentBinFile(string file_name, bool b_with_updated_demand_type_info)
{

	cout << "Reading Agent Bin File..." << endl;
	g_VehicleLoadingMode = vehicle_binary_file_mode;

	g_DetermineDemandLoadingPeriod();

	if (b_with_updated_demand_type_info)
	{
		g_ReadScenarioFilesUnderAgentBinaryMode();
	}


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
				cout <<  "Error in reading agent file: header.number_of_node = " << header.number_of_nodes <<endl;
				g_ProgramStop();
			}
			pVehicle->m_ArrivalTime = 0;
			pVehicle->m_bComplete = false;
			pVehicle->m_bLoaded = false;
			pVehicle->m_TollDollarCost = 0;
			pVehicle->m_Emissions = 0;
			pVehicle->m_Distance = 0;
			pVehicle->m_NodeNumberSum = 0;

			int time_interval = pVehicle->m_DepartureTime / 15;

			if (g_ODEstimationFlag == 1) // having hist od only unde ODME mode
			{
				g_HistDemand.AddValue(pVehicle->m_OriginZoneID, pVehicle->m_DestinationZoneID, time_interval, 1); // to store the initial table as hist database
			}

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
						pVehicle->m_OriginNodeID = g_NodeNametoIDMap[path_node_sequence[0]];

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

				int AssignmentInterval = g_FindAssignmentInterval(pVehicle->m_DepartureTime);

				g_TDOVehicleArray[g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo][AssignmentInterval].VehicleArray.push_back(pVehicle->m_VehicleID);

				count++;

				if (count % 10000 == 0)
					cout << "reading " << count / 1000 << "K agents from binary file " << file_name << endl;
			}
		}
		g_ResetVehicleAttributeUsingDemandType();

		if (g_use_global_path_set_flag == 1)
			g_BuildGlobalPathSet();
		
		
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

void g_ResetVehicleType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;
		pVehicle->m_InformationClass = info_hist_based_on_routing_policy;
		double RandomPercentage = g_GetRandomRatio() * 100;
		for (int in = 0; in < MAX_INFO_CLASS_SIZE; in++)
		{
			if (RandomPercentage >= g_DemandTypeMap[pVehicle->m_DemandType].cumulative_info_class_percentage[in - 1] &&
				RandomPercentage < g_DemandTypeMap[pVehicle->m_DemandType].cumulative_info_class_percentage[in])
				pVehicle->m_InformationClass = in + 1; // return pretrip as 2 or enoute as 3
		}
	}

}

void g_ResetVehicleAttributeUsingDemandType()
{
	std::map<int, DTAVehicle*>::iterator iterVM;
	for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
	{
		DTAVehicle* pVehicle = iterVM->second;

		g_GetVehicleAttributes(pVehicle->m_DemandType, pVehicle->m_VehicleType, pVehicle->m_PricingType, pVehicle->m_InformationClass, pVehicle->m_VOT, pVehicle->m_Age);

	}
}




void g_AccessibilityMatrixGenerationForAllDemandTypes(string file_name, bool bTimeDependentFlag, double CurrentTime)
{


	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int DemandLoadingStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int DemandLoadingEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		DemandLoadingStartTimeInMin = CurrentTime;
		DemandLoadingEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculation interval " << DemandLoadingStartTimeInMin << " -> " << DemandLoadingEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;

	int total_demand_type = 2;

	int StatisticsIntervalSize = max(1, (DemandLoadingEndTimeInMin - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval);


	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float**** ODTravelTime = NULL;
	ODTravelTime = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDistance = NULL;
	ODDistance = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float**** ODDollarCost = NULL;
	ODDollarCost = Allocate4DDynamicArray<float>(total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);


	for (int d = 1; d <= total_demand_type; d++)
	for (int i = 0; i <= g_ODZoneIDSize; i++)
	for (int j = 0; j <= g_ODZoneIDSize; j++)
	for (int t = 0; t < StatisticsIntervalSize; t++)
	{

		if (i == j)
		{
			ODTravelTime[d][i][j][t] = 0.5;
			ODDistance[d][i][j][t] = 0.5;
			ODDollarCost[d][i][j][t] = 0.0;
		}
		else
		{
			ODTravelTime[d][i][j][t] = 0;
			ODDistance[d][i][j][t] = 0;
			ODDollarCost[d][i][j][t] = 0.0;
		}
	}


	if (bTimeDependentFlag)
		cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
	else
		cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


//#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{


		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0


		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

		if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
		{
			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
		}

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{
			int PricingType = demand_type;

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_VehicleID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{

						bDistanceCost = false;
						for (int departure_time = DemandLoadingStartTimeInMin; departure_time < DemandLoadingEndTimeInMin; departure_time += g_AggregationTimetInterval)
						{

							g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, departure_time, PricingType, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo



							// to each destination zone
							for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
							{

								int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
								if (dest_node_index >= 0) // convert node number to internal node id
								{

									int time_interval_no = (departure_time - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval;
									ODTravelTime[PricingType][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
									ODDistance[PricingType][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];
									ODDollarCost[PricingType][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDollarCostAry[dest_node_index];


								}

							} //for each destination zone
						}  // departure time
					}  // with origin node numbers 
				} // current thread	

			}  // origin zone

		} // each demand type 

	}  // multiple threads


	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "w");
	if (st != NULL)
	{
		//write header:
		fprintf(st, "origin_zone_id, destination_zone_id, departure_time_in_min,");
		CString str;

		for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
		{

			str.Format("demand_type_%d_generalized_travel_time,demand_type_%d_distance,demand_type_%d_dollar_cost,", demand_type, demand_type, demand_type);
			fprintf(st, str);

		}

		//str.Format("demand_type_%d_generalized_travel_time_diff,demand_type_%d_distance_diff,demand_type_%d_dollar_cost_diff,", total_demand_type, total_demand_type, total_demand_type);
		fprintf(st, str);


		fprintf(st, "\n");



		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{
			// to each destination zone
			for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
			{


				for (int departure_time = DemandLoadingStartTimeInMin; departure_time < DemandLoadingEndTimeInMin; departure_time += g_AggregationTimetInterval)
				{

					int time_interval_no = (departure_time - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval;
					

					int origin_zone = iterZone->first;
					int destination_zone = iterZone2->first;

					fprintf(st, "%d,%d,%d,",
						origin_zone,
						destination_zone, departure_time);

					for (int demand_type = 1; demand_type <= total_demand_type; demand_type++)
					{

						fprintf(st, "%4.2f,%4.2f,%4.2f,",

							ODTravelTime[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDistance[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDollarCost[demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);
					}

					//fprintf(st, "%4.2f,%4.2f,%4.2f,",

					//	ODTravelTime[total_demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] -
					//	ODTravelTime[1][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],

					//	ODDistance[total_demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] - 
					//	ODDistance[1][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],

					//	ODDollarCost[total_demand_type][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]-
					//	ODDollarCost[1][iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);


					fprintf(st, "\n");
				}  // each department type
			}


		}
		fclose(st);

	}
	else
	{
		cout << "File " << file_name.c_str() << " cannot be opened." << endl;
		getchar();
		exit(0);

	}

	Deallocate4DDynamicArray(ODTravelTime, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	Deallocate4DDynamicArray(ODDistance, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);
	Deallocate4DDynamicArray(ODDollarCost, total_demand_type + 1, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

}


void g_AgentBasedAccessibilityMatrixGeneration(string file_name, bool bTimeDependentFlag, int PricingType, double CurrentTime)
{
	bool bDistanceCost = false;

	bool bRebuildNetwork = false;

	if (bTimeDependentFlag == true)
		bRebuildNetwork = true;


	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	int DemandLoadingStartTimeInMin = g_DemandLoadingStartTimeInMin;
	int DemandLoadingEndTimeInMin = g_DemandLoadingEndTimeInMin;

	if (bTimeDependentFlag)
	{
		bUseCurrentInformation = false;  // then time dependent travel time wil be used
	}


	if (bUseCurrentInformation == true)
	{
		DemandLoadingStartTimeInMin = CurrentTime;
		DemandLoadingEndTimeInMin = CurrentTime + 1;

	}
	cout << "calculation interval " << DemandLoadingStartTimeInMin << " -> " << DemandLoadingEndTimeInMin << "min; with an aggregation time interval of " << g_AggregationTimetInterval << endl;


	int StatisticsIntervalSize = max(1, (DemandLoadingEndTimeInMin - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval);


	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float*** ODTravelTime = NULL;
	ODTravelTime = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	float*** ODDistance = NULL;
	ODDistance = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, StatisticsIntervalSize);

	for (int i = 0; i <= g_ODZoneIDSize; i++)
	for (int j = 0; j <= g_ODZoneIDSize; j++)
	for (int t = 0; t < StatisticsIntervalSize; t++)
	{

		if (i == j)
		{
			ODTravelTime[i][j][t] = 0.5;
			ODDistance[i][j][t] = 0.5;
		}
		else
		{
			ODTravelTime[i][j][t] = 0;
			ODDistance[i][j][t] = 0;
		}
	}


	if (bTimeDependentFlag)
		cout << "calculating time-dependent skim matrix on " << number_of_threads << " processors ... " << endl;
	else
		cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;


#pragma omp parallel for
	for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
	{


		// create network for shortest path calculation at this processor
		int	id = omp_get_thread_num();  // starting from 0


		//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

		if (bRebuildNetwork || g_TimeDependentNetwork_MP[id].m_NodeSize == 0)
		{
			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...
		}

		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{

			if ((iterZone->first%number_of_threads) == ProcessID)
			{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

				int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_VehicleID/100.0f as random number between 0 and 1, so we can reproduce the results easily

				if (origin_node_indx >= 0) // convert node number to internal node id
				{

					bDistanceCost = false;
					for (int departure_time = DemandLoadingStartTimeInMin; departure_time < DemandLoadingEndTimeInMin; departure_time += g_AggregationTimetInterval)
					{
						g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, departure_time, PricingType, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo


						// to each destination zone
						for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
						{

							int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
							if (dest_node_index >= 0) // convert node number to internal node id
							{

								int time_interval_no = (departure_time - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval;
								ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
								ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];


							}

						} //for each demand type
					}  // departure time
				}  // with origin node numbers 
			} // current thread	

		}  // origin zone

	}  // multiple threads


	if (bTimeDependentFlag == true) // time-dependent skim files
	{
		for (int departure_time = DemandLoadingStartTimeInMin; departure_time < DemandLoadingEndTimeInMin; departure_time += g_AggregationTimetInterval)
		{

			CString str;
			int time_interval_no = departure_time / g_AggregationTimetInterval;


			if (PricingType == 1)
				str.Format("skim%d.csv", time_interval_no);
			if (PricingType == 2)
				str.Format("skim_HOV_%d.csv", time_interval_no);
			if (PricingType == 3)
				str.Format("skim_truck_%d.csv", time_interval_no);
			if (PricingType == 4)
				str.Format("skim_transit_%d.csv", time_interval_no);


			FILE* st = NULL;
			fopen_s(&st, str, "w");
			if (st != NULL)
			{


				// from each origin zone
				for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
				{
					// to each destination zone
					for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
					{

						int time_interval_no = (departure_time - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval;
						fprintf(st, "%d,%d,%4.2f,%4.2f\n",
							iterZone->first,
							iterZone2->first,
							ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]);

					}
				}


				fclose(st);
			}

		}

	}
	else // real time skim file
	{

		// step w: calculate experienced travel time for complete vehile trips

		ODStatistics** ODMOEArray = NULL;

		int total_number_of_zones = g_ZoneMap.size();

		ODMOEArray = AllocateDynamicArray<ODStatistics>(total_number_of_zones, total_number_of_zones);

		int i, j;
		for (i = 0; i < total_number_of_zones; i++)
		for (j = 0; j < total_number_of_zones; j++)
		{

			ODMOEArray[i][j].OriginZoneNumber = 0;
			ODMOEArray[i][j].DestinationZoneNumber = 0;
			ODMOEArray[i][j].TotalVehicleSize = 0;
			ODMOEArray[i][j].TotalCompleteVehicleSize = 0;
			ODMOEArray[i][j].TotalTravelTime = 0;
			ODMOEArray[i][j].TotalDistance = 0;

		}

		std::map<int, DTAVehicle*>::iterator iterVM;
		for (iterVM = g_VehicleMap.begin(); iterVM != g_VehicleMap.end(); iterVM++)
		{

			DTAVehicle* pVehicle = iterVM->second;

			int origin_zone_no = g_ZoneMap[pVehicle->m_OriginZoneID].m_ZoneSequentialNo;
			int destination_zone_no = g_ZoneMap[pVehicle->m_DestinationZoneID].m_ZoneSequentialNo;

			ODMOEArray[origin_zone_no][destination_zone_no].TotalVehicleSize += 1;
			int arrival_time_window_begin_time_in_min = CurrentTime - g_AggregationTimetInterval;
			if (pVehicle->m_PricingType == PricingType && pVehicle->m_NodeSize >= 2 && pVehicle->m_bComplete && pVehicle->m_ArrivalTime >= arrival_time_window_begin_time_in_min)  // with physical path in the network
			{


				ODMOEArray[origin_zone_no][destination_zone_no].OriginZoneNumber = pVehicle->m_OriginZoneID;
				ODMOEArray[origin_zone_no][destination_zone_no].DestinationZoneNumber = pVehicle->m_DestinationZoneID;


				ODMOEArray[origin_zone_no][destination_zone_no].TotalCompleteVehicleSize += 1;
				ODMOEArray[origin_zone_no][destination_zone_no].TotalTravelTime += pVehicle->m_TripTime;
				ODMOEArray[origin_zone_no][destination_zone_no].TotalDistance += pVehicle->m_Distance;
			}
		}


		for (i = 0; i < total_number_of_zones; i++)
		for (j = 0; j < total_number_of_zones; j++)
		{

			if (ODMOEArray[i][j].TotalCompleteVehicleSize >= 1)
			{

				ODTravelTime[i][j][0]
					= ODMOEArray[i][j].TotalTravelTime / max(1, ODMOEArray[i][j].TotalCompleteVehicleSize);

				ODDistance[i][j][0]
					= ODMOEArray[i][j].TotalDistance / max(1, ODMOEArray[i][j].TotalCompleteVehicleSize);
			}
		}
		if (ODMOEArray != NULL)
			DeallocateDynamicArray<ODStatistics>(ODMOEArray, total_number_of_zones, total_number_of_zones);

		//

		FILE* st = NULL;
		fopen_s(&st, file_name.c_str(), "w");
		if (st != NULL)
		{
			//fprintf(st, "from_zone_id,to_zone_id,trip_time_in_min,trip_distance_in_mile\n");

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{
				// to each destination zone
				for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
				{

					for (int departure_time = DemandLoadingStartTimeInMin; departure_time < DemandLoadingEndTimeInMin; departure_time += g_AggregationTimetInterval)
					{
						int time_interval_no = (departure_time - DemandLoadingStartTimeInMin) / g_AggregationTimetInterval;
						fprintf(st, "%d,%d,%4.2f,%4.2f\n",
							iterZone->first,
							iterZone2->first,
							ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no],
							ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][time_interval_no]
							);

					}

				}
			}
		}
		else
		{
			cout << "File " << file_name << " cannot be opened. Please check." << endl;

			fclose(st);
			g_ProgramStop();
		}

		fclose(st);

	}

	if (ODTravelTime != NULL)
		Deallocate3DDynamicArray<float>(ODTravelTime, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

	if (ODDistance != NULL)
		Deallocate3DDynamicArray<float>(ODDistance, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);


}

void g_AgentBasedAccessibilityMatrixGenerationExtendedSingleFile(string file_name, double CurrentTime)
{

	bool bDistanceCost = true;

	bool bRebuildNetwork = true;

	// find unique origin node
	// find unique destination node
	int number_of_threads = g_number_of_CPU_threads();


	// calculate distance 
	int node_size = g_NodeVector.size();
	int link_size = g_LinkVector.size();

	bool  bUseCurrentInformation = true;

	//cout << "allocating memory for time-dependent ODMOE data...for " << g_ODZoneIDSize << " X " <<  g_ODZoneIDSize << "zones for " << 
	//	StatisticsIntervalSize << " 15-min time intervals" << endl;
	float*** ODTravelTime = NULL;
	ODTravelTime = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, MAX_PRICING_TYPE_SIZE);

	float*** ODDistance = NULL;
	ODDistance = Allocate3DDynamicArray<float>(g_ODZoneIDSize + 1, g_ODZoneIDSize + 1, MAX_PRICING_TYPE_SIZE);

	for (int i = 0; i <= g_ODZoneIDSize; i++)
	for (int j = 0; j <= g_ODZoneIDSize; j++)
	for (int t = 0; t < MAX_PRICING_TYPE_SIZE; t++)
	{

		if (i == j)
		{
			ODTravelTime[i][j][t] = 0.5;
			ODDistance[i][j][t] = 0.5;
		}
		else
		{
			ODTravelTime[i][j][t] = 0;
			ODDistance[i][j][t] = 0;
		}
	}

	cout << "calculating real-time skim matrix on " << number_of_threads << " processors ... " << endl;

	for (int pricing_type = 1; pricing_type < MAX_PRICING_TYPE_SIZE; pricing_type++)
	{

#pragma omp parallel for
		for (int ProcessID = 0; ProcessID < number_of_threads; ProcessID++)
		{


			// create network for shortest path calculation at this processor
			int	id = omp_get_thread_num();  // starting from 0


			//special notes: creating network with dynamic memory is a time-consumping task, so we create the network once for each processors

			g_TimeDependentNetwork_MP[id].BuildPhysicalNetwork(0, -1, g_TrafficFlowModelFlag, bUseCurrentInformation, CurrentTime);  // build network for this zone, because different zones have different connectors...

			// from each origin zone
			for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
			{

				if ((iterZone->first%number_of_threads) == ProcessID)
				{ // if the remainder of a zone id (devided by the total number of processsors) equals to the processor id, then this zone id is 

					int origin_node_indx = iterZone->second.GetRandomOriginNodeIDInZone((0) / 100.0f);  // use pVehicle->m_VehicleID/100.0f as random number between 0 and 1, so we can reproduce the results easily

					if (origin_node_indx >= 0) // convert node number to internal node id
					{

						bDistanceCost = false;
						g_TimeDependentNetwork_MP[id].TDLabelCorrecting_DoubleQueue(origin_node_indx, CurrentTime, pricing_type, DEFAULT_VOT, bDistanceCost, false, true);  // g_NodeVector.size() is the node ID corresponding to CurZoneNo


						// to each destination zone
						for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
						{

							int dest_node_index = iterZone2->second.GetRandomDestinationIDInZone((0) / 100.0f);
							if (dest_node_index >= 0) // convert node number to internal node id
							{

								ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][pricing_type] = g_TimeDependentNetwork_MP[id].LabelCostAry[dest_node_index];
								ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][pricing_type] = g_TimeDependentNetwork_MP[id].LabelDistanceAry[dest_node_index];


							}

						} //for each destination zone
					}  // with origin node numbers 
				} // current thread	

			}  // origin zone

		}  // multiple threads

	}
	FILE* st = NULL;
	fopen_s(&st, file_name.c_str(), "w");
	if (st != NULL)
	{


		// from each origin zone
		for (std::map<int, DTAZone>::iterator iterZone = g_ZoneMap.begin(); iterZone != g_ZoneMap.end(); iterZone++)
		{
			// to each destination zone
			for (std::map<int, DTAZone>::iterator iterZone2 = g_ZoneMap.begin(); iterZone2 != g_ZoneMap.end(); iterZone2++)
			{

				for (int t = 1; t < MAX_PRICING_TYPE_SIZE; t++)
				{

					fprintf(st, "%d,%d,%4.2f,%4.2f\n",
						iterZone->first,
						iterZone2->first,
						ODTravelTime[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][t],
						ODDistance[iterZone->second.m_ZoneSequentialNo][iterZone2->second.m_ZoneSequentialNo][t]);
				}
			}
		}


		fclose(st);
	}
	if (ODTravelTime != NULL)
		Deallocate3DDynamicArray<float>(ODTravelTime, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);

	if (ODDistance != NULL)
		Deallocate3DDynamicArray<float>(ODDistance, g_ODZoneIDSize + 1, g_ODZoneIDSize + 1);


}

