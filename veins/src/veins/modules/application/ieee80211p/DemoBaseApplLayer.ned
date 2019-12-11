//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

package org.car2x.veins.modules.application.ieee80211p;
import org.car2x.veins.base.modules.IBaseApplLayer;

//
// Demo application layer base class.
//
// @author David Eckhoff
//
// @see DemoBaseApplLayer
// @see Nic80211p
// @see Mac1609_4
// @see PhyLayer80211p
//
simple DemoBaseApplLayer like IBaseApplLayer
{
    parameters:
        @class(Veins::DemoBaseApplLayer);
        int headerLength = default(88bit) @unit(bit); //header length of the application

        bool sendBeacons = default(true); //tell the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconUserPriority = default(7); //the user priority (UP) of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages

        int dataLengthBits = default(1024bit) @unit(bit); //the length of a data packet
        bool dataOnSch = default(false); //tells the applayer whether to use a service channel for datapackets or the control channel
        int dataUserPriority = default(7); //the default user priority (UP) for data packets

        bool avoidBeaconSynchronization = default(true); //don't start beaconing directly after node was created but delay to avoid artifical synchronization

        bool sendWSA = default(false);
        int wsaLengthBits = default(250bit) @unit(bit);
        double wsaInterval =  default(1s) @unit(s);
      	
      	//RSUs 
        int NumberofRSU = default(5);
   		int RSU0X = default(1000);int RSU0Y = default(1000);
 		int RSU1X = default(1000);int RSU1Y = default(1000);
 		int RSU2X = default(1000);int RSU2Y = default(1000);
 		int RSU3X = default(1000);int RSU3Y = default(1000);
 		int RSU4X = default(1000);int RSU4Y = default(1000);
 		int RSU5X = default(1000);int RSU5Y = default(1000);
 		int RSU6X = default(1000);int RSU6Y = default(1000);
 		int RSU7X = default(1000);int RSU7Y = default(1000);
 		int RSU8X = default(1000);int RSU8Y = default(1000);
 		int RSU9X = default(1000);int RSU9Y = default(1000);
 		int RSU10X = default(1000);int RSU10Y = default(1000);
 		 		
 		//distances to neighbors
 		double DistanceThreshold = default(350m) @unit(m); 		
      
      	//density threshold
 		double DensityThreshold = default(400m) @unit(m); 		
      
      
        //NODE NEIGHBORS TABLE (NNT)
        int UpdateNodeNeighborsTable = default(2s) @unit(s);
        int Packet_Size = default(1000bytes) @unit(bytes); 
        	
        //charging stations 
        int BatteryCapacity = default(24kWh) @unit(kWh);
        int FastChargingStationPower = default(50kW) @unit(kW);
        int numberofCSs = default(4);
        double BatteryThreshold = default(0.3);
        bool ClosestCSinDistance = default(false);
        
        // CS Locations
        double CS1_x_Coord = default(1000);double CS1_y_Coord = default(1000);
        double CS2_x_Coord = default(1000);double CS2_y_Coord = default(1000);
        string CS1 = default("gneE5");
        string CS2 = default("gneE2");

        //save file 
        string pathToSaveTXT = default("/root/Projects/untitled");

		//density
		int Max_EV_Density = default(100);
		int Max_FFV_Density = default(10);
		
		//print debug
		bool printDebug = default(false);
		
		//results output file
		string ResultsFile = default("ResultsFile");
		
		//run number of repetition
		int run_num = default(0);
		
		//config name
		string Conf_Name = default("Conf_Name");
		
		//vehicles checking interval IAT
		int Check_Density_Interval =  default(10s) @unit(s);   
		
		//max number of routes
		int Numm_of_Vehicles_Routes = default(300);
		        
        //Routing protocol
        int MaxNumberofHops = default(10);  //max number of hops to forward msg
        
        //2hGAR LUIS
        bool Tabu = default(true);
      
        //2hGAR LUIS
        bool LongTabu = default(false);
        
       	//Expiration of message in buffer
      	int Buffer_TimeOut = default(5s) @unit(s);
      	
      	//Num_of_Metrics for MMMR
      	int Num_of_Metrics_MMMR = default(2);
      	
      	//Time to start sending msgs
      	int Time_to_Start_Stop_sending_MSG = default(20s) @unit(s);
      	
      	//parameter for compute end of send msgs
      	int SimTimeParameter = default(300s) @unit(s);
      	
      	//Median_msg_interval
      	int Median_msg_interval = default(5s) @unit(s);
      	
      	//usa solo MMMR
      	bool MMMR_Only = default(false);
      	
      	//usado para banner nada mas
      	int repeat_number = default(0);
      	
    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
}
