//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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
#pragma once

//#ifndef BEACON_H_
//#define BEACON_H_

#include "veins/veins.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/utils/SimpleAddress.h"
#include <iomanip>      // std::setw
#include <vector>

using namespace std;
using namespace Veins;

class BeaconList {
private:
    typedef struct beacon {
            string typeNode;
            int idMsg;
            int idVehicle;
            double time = 0;
            Coord SenderCoord;
            double dsr = 0;
            double dsd = 0;
            double drd = 0;
            double Nv = 0;
            double wdist = 0;
            double abe = 0;
            double GS = -1;
            double Local_N_GS = -1;
            /////////////////////  LUIS  //////////////////////////////
            LAddress::L2Type NH_Address = 0;
            double NH_GS = 1;
            double NH_Dst_to_Dest = 0;
            ////////////////////////////////////////////////////////
            // self messages for timers
            cMessage *beaconStartMsg;  ///< start of each beacon
            beacon* next;
    }* beaconPtr;

    beaconPtr head;
    beaconPtr curr;
    beaconPtr temp;

public:
    BeaconList();
    void AddBeacon(string b_typeNode,int b_idMsg,int b_idVehicle,double b_time,Coord b_SenderCoord,double b_dsr,double b_dsd,double b_drd,double b_Nv,double b_wdist,double abe,double b_GS,LAddress::L2Type b_NH_Address,double b_NH_GS,double NH_Dst_to_Dest,double Local_N_GS);
    void UpdateBeacon(string b_typeNode,int b_idMsg,int b_idVehicle,double b_time,Coord b_SenderCoord,double b_dsr, double b_dsd, double b_drd, double b_Nv, double b_wdist,double abe,double b_GS,LAddress::L2Type b_NH_Address,double b_NH_GS,double NH_Dst_to_Dest,double Local_N_GS);
    void DeleteBeacon(int b_idVehicle);
    void PrintBeacons(int CurrNodeAddress,string IN_OUT);
    void SortBeacons(int Num_of_Metrics);                           /// numbero de metrics (BW, Distance,Density)
    void PurgeBeacons(double b_ttl);
    int CounterBeacons();
    bool SearchBeaconTreeID(int treeID);
    bool SearchBeaconNodeID(int NodeID);
    int SearchBeaconRSU();                                          //PABLO REVISA SI HAY UN RSU EN LA TABLA DE VECINOS y devuelve el SENDER ADDRESS DEL RSU
    int SearchClstNODE(Coord DstNodeCoord);
    int TopTable();
    double TopTableNv();                                            //REGRESA LA DENSIDAD DEL TOP NODE
    double TopTabledisToDestination();
    double topTableDtoS ();
    Coord SearchBeaconCoord(int NodeID);                            //ME regresa las coord delo nodeid si esta en la tabla
    double TopTable_U_dist();                                       //REGRESA LA U_DIST  DEL TOP NODE
    Coord TopTableNghCoord();                                       //REGRESA LAS COORD DEL TOP NODE
    bool Search_RSU_NNT();                                          //REGRESA TRUE SI HAY UN RSU EN LA NNT
    double TopTableNH_GS();
    bool SearchRSUid(int RSU_ID);
///========================================================================================================================
    LAddress::L2Type NH_GS(int nodeID);
    void CheckForLoops(int myId);                                ///CANDIDATE NODE --> LOOOPS
    void NHinCurrentNNT(int myId);                               ///CANDIDATE NODE --> NH in my NNT signfica que el NH de mi vecino tambien es mi vecino; entonces descarto este nodo como candidato ya que puede enviar directamente al NH
    void TabuListSearch(vector <int> TabuList, int nodeID);      ///CANDIDATE NODE --> Busca si tengo nodos en la Tabulist en mi NNT para marcarlos como no elegibles
    int TwohGAR_Conditions(bool MMMR_Only, int OriginalSenderID, bool TL, vector <int> TabuList, int myID, string MsgType, double N_DstToRSU, bool printDebug, int MsgID);
    double n_S(int nexthopAddress);
    double n_DsttoRSU(int nexthopAddress);
};


//#endif /* BEACON_H_ */
