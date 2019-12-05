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

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/beacon.h"

#include <algorithm>    // std::min_element, std::max_element#include <algorithm>    // std::min_element, std::max_element
#include <iomanip>      // std::setw
#include <iostream>
#include <fstream>



using namespace std;


namespace Veins {

/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to DemoBaseApplLayer, adding WSA
 *
 */

class TraCIDemo11p : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;
    double CurrentConsumption_Battery;
    double PercentageofSoC;
    double AccumulatedConsumption;
//**************************new code pablo***********************************
    bool ev_already_arrived_to_cs;
    int Msg_Interval_Counter;
    int msg_interval_send_msg;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    void Go_to_CS(string road,int CS_Selected);                 //REroute a una nuevo destion 'road'
    int Arrived_to_CS();                                        //VERIFICA SI EL EV llego A LA CSs y devuelve el cs number
    void At_CS(int CS);                                         //procss when ev arrived to cs CS
    void Update_CS_Conditions();                                //actualiza las condiciones en las CSs
    int Threshold_Random();                                     //threshold de la batteria variable
    double ChargingTime();                                      // calcula el tiempo de carga
    void Save_EV_Data_To_File(int CS_Selected);                 //guarda los datos en un txt
    void ReadDatafromFile(int car_ID);
    void Save_EV_Data_Set_Tittle();
    void Save_EV_Data_At_EV_Arrival(int cs_arrived);
    void Send_Msg();
    void CREATE_MSG_AND_SAVE_IN_BUFFER();
    void TRY_TO_SEND_MSG_IN_BUFFER();

};
} // namespace Veins
