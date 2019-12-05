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

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"


using namespace Veins;

Define_Module(Veins::TraCIDemo11p);

void TraCIDemo11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
//#######################################################################################//
//############################//electric vehicle//#######################################//
//#######################################################################################//
        CurrentConsumption_Battery = 0;
        AccumulatedConsumption = 0;
        PercentageofSoC=0;
        reroute = false;
        ev_already_arrived_to_cs= false;
//#######################################################################################//
//############################//  PROTOCOL       //#######################################//
//#######################################################################################//
        Msg_Interval_Counter = 0;           //USADO PARA EL INTERVALO DE ENVIO DE MSG
        std::random_device rd;
        std::mt19937 e2(rd());
        std::uniform_int_distribution<> dist(2, 6);
        msg_interval_send_msg = dist(e2);

    }

}

void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa)
{
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void TraCIDemo11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);

    findHost()->getDisplayString().setTagArg("i", 1, "green");

    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getDemoData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        // repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01, 0.2), wsm->dup());
    }
}

void TraCIDemo11p::handleSelfMsg(cMessage* msg)
{
    if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)){
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {stopService();delete (wsm);}
        else {scheduleAt(simTime() + 1, wsm);}
    }

    else {
///=================================================================================================================///
        DemoBaseApplLayer::handleSelfMsg(msg);      ///BEACON
///=================================================================================================================///
        if(simTime() >= Time_to_Start_Stop_sending_MSG){Send_Msg();}   ///  CREO EL MSG PARA ENVIAR
///=================================================================================================================///
        DemoBaseApplLayer::BUFFER_MSG_TIMEOUT();     ///DELETED EXPIRED MSGs in BUFFER
///=================================================================================================================///
    }
}

void TraCIDemo11p::Send_Msg(){                                      //crea un beacon y envia el MSG con el SREQ = true

///=====================================================================================================///
///       SEND NEW MSG ACCORDING TO MSG_INTERVAL &&   AND THEN TRY TO SEND MSG IN BUFFER                ///
///=====================================================================================================///

    int Interval_To_Send_Beacons = msg_interval_send_msg;                 ////DEPENDS ON BEACON INTERVAL -> msg_interval per beacons

    if(Msg_Interval_Counter == Interval_To_Send_Beacons && simTime() <= TimeToEndSendingMSGs){

        CREATE_MSG_AND_SAVE_IN_BUFFER();                                                                  //CREA UN MSG NUEVO Y LO GUADA EN EL BUFFER -> NO SE ENVIA SOLO SE FUARDA EN EL BUFFER

        if(!Buffer_List.empty() && ListBeacon.CounterBeacons() != 0){TRY_TO_SEND_MSG_IN_BUFFER();}        //LUEGO DE CREAR EL MSG INTENTA ENVIAR MSG EN EL BUFFER

        Msg_Interval_Counter = 0;

    }
///=====================================================================================================///
///                                 TRY TO SEND MSG IN BUFFER                                           ///
///=====================================================================================================///
    else{

        if(!Buffer_List.empty() && ListBeacon.CounterBeacons() != 0){TRY_TO_SEND_MSG_IN_BUFFER();}

        Msg_Interval_Counter++;
    }
}

void TraCIDemo11p::TRY_TO_SEND_MSG_IN_BUFFER(){

    for(list<DemoBaseApplLayer::Buffer_Element_Type>::iterator it=Buffer_List.begin();it!=Buffer_List.end();it++){

        if(!Buffer_List.empty()){

            DemoSafetyMessage* New_Msg = new DemoSafetyMessage();                                                //CREA UN NUEVO MSG
            populateWSM(New_Msg);                                                                                // Actualiza campos del nuevo beacon  TabuList ={0,0,0};

            DemoBaseApplLayer::Buffer_Element_Type Buffer_Msg = *it;

            if(Tabu){for(int i=0;i<TabuList.size();i++){New_Msg->setTabuList(i, Buffer_Msg.TabuList_MSG[i]);}}

            int OriginalSenderID = Buffer_Msg.SenderID;

            if(FWD_MSG(OriginalSenderID, New_Msg,"fwd",Buffer_Msg.MsgId)){                                                     //SE ACTUALIZAN LOS CAMOPS DEL MSG PARA ENVIAR

                int MSG_Hops = Buffer_Msg.Hops;

                if(MSG_Hops < MaxNumberofHops){

                    bool NH_is_RSU = false;

                    New_Msg->setUserPriority(5);
                    New_Msg->setBuffer_TreeId(Buffer_Msg.MsgId);
                    New_Msg->setHopsCounter(Buffer_Msg.Hops + 1);
                    New_Msg->setSenderAddress(OriginalSenderID);

                    if(Tabu){
                        if(ListBeacon.SearchRSUid(New_Msg->getRecipientAddress())){NH_is_RSU = true;}
                        DemoBaseApplLayer::TabuList_Read_Update(New_Msg,"UPDATE",New_Msg->getRecipientAddress(),NH_is_RSU);
                    }

                    if(printDebug){std::cerr<<myId<<"  -- DEL:"<<Buffer_Msg.MsgId<<"          Buffer Size:"<<Buffer_List.size()-1<<"    T:"<<simTime()<<endl;}
                    it = Buffer_List.erase(it);                                                                 //BORRA EL MENSAJE DEL BUFFER
                    --it;

                    sendDown(New_Msg);                                                                          //ENVIA EL MSG

                }else{
                    if(printDebug){std::cerr<<myId<<"  -- DEL:"<<Buffer_Msg.MsgId<<"  MAX HOPS"<<"    T:"<<simTime()<<endl;}
                    it = Buffer_List.erase(it);                                                                 //BORRA EL MENSAJE DEL BUFFER supera MAX HOP COUNT
                    --it;
                }

            }else{delete(New_Msg);}                                                                             // SI NO SE PUEDE ENVIAR EL MSG -> SE BORRA
        }
    }
}

void TraCIDemo11p::CREATE_MSG_AND_SAVE_IN_BUFFER(){

    bool sendMSG = false;

    DemoSafetyMessage* New_Msg = new DemoSafetyMessage();
    populateWSM(New_Msg);                                                                                     // Actualiza campos del nuevo beacon  TabuList ={0,0,0};

    DemoBaseApplLayer::SAVE_MSG_S_SIGNAL(New_Msg);                                                            /// PARA ESTADISTICAS GUARDO CADA MENSAJE CREADO
    DemoBaseApplLayer::BUFFER_MSG(New_Msg,"MSG");                                                             /// NEW MSG A BUFFER CON LA TL ACTUALIZADA
    delete(New_Msg);
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj){
    DemoBaseApplLayer::handlePositionUpdate(obj);
}

void TraCIDemo11p::At_CS(int CS){ //procedimiento cuendo el ev ya a llegado a la cs
  switch (CS) {
      case 1: {CS1_queue++;
      break;
      }
      case 2: {CS2_queue++;
      break;
      }
  }
  Update_CS_Conditions(); //actualizo cada handle update las condiciones de las CS
}

int TraCIDemo11p::Arrived_to_CS(){                              //VERIFICA SI EL EV llego A LA CSs
    int ev_arrived_to_cs = 0;
    std::string RoadID = mobility->getRoadId();

    if (!RoadID.empty()){
        for (int i=1;i<=numberofCSs;i++){
            if (RoadID==ChargingStations[i].edgeID){
                ev_arrived_to_cs = i;
                Save_EV_Data_At_EV_Arrival(i);                  //GUARDO DATOS A LA LLEGADA DEL EV
                break;
            }
        }
    }else
        std::cerr<<"ERROR: NO SE PUEDE LEER EL ROAD ACTUAL DEL EV"<<endl;

    return ev_arrived_to_cs;
}

void TraCIDemo11p::Update_CS_Conditions(){
    ChargingStations[1].queue = CS1_queue;
    ChargingStations[2].queue = CS2_queue;
    ChargingStations[3].queue = CS3_queue;
    ChargingStations[1].WaitingTime = CS1_WaitingTime;
    ChargingStations[2].WaitingTime = CS2_WaitingTime;
    ChargingStations[3].WaitingTime = CS3_WaitingTime;
}

int TraCIDemo11p::Threshold_Random(){
    std::random_device rd;
    //
    // Engines
    //
    std::mt19937 e2(rd());
    //std::knuth_b e2(rd());
    //std::default_random_engine e2(rd()) ;
    //
    // Distribtuions
    //
    std::uniform_real_distribution<> dist(0, 100);
    //std::normal_distribution<> dist(2, 2);
    // std::student_t_distribution<> dist(5);
    //  std::poisson_distribution<> dist(2);
    //std::extreme_value_distribution<> dist(0,2);

    int randomnumber = floor(dist(e2));
    /*
    std::map<int, int> hist;
    for (int n = 0; n < 10000; ++n) {
        ++hist[std::floor(dist(e2))];
    }
    for (auto p : hist) {
        std::cout << std::fixed << std::setprecision(1) << std::setw(2)
        << p.first << ' ' << std::string(p.second/200, '*') << '\n';
    }
    */
    return randomnumber;
}

double TraCIDemo11p::ChargingTime(){
//    double Chargingtime = ((BatteryCapacity)*(1-Battery_SoC()))/(FastChargingStationPower);  //charging time en horas
  //  Chargingtime = Chargingtime*60; //Chargingtime en minutos
    double Chargingtime=1000;//temporal
    return Chargingtime;
}

void TraCIDemo11p::Save_EV_Data_To_File(int CS_Selected){
  //  if(!set_tittle){Save_EV_Data_Set_Tittle();}

    low_batt_file.open("low_batt_file.txt", ios::out | ios::app );  //para leer datos ios::in
    if (low_batt_file.is_open()){
 //       low_batt_file <<getParentModule()->getId()<<" "<<Battery_SoC()<<" "<<simTime().dbl()<<" "<<CS_Selected<<'\n';
        low_batt_file.close();
    }
    else cout << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< '\n';
}

void TraCIDemo11p::Save_EV_Data_Set_Tittle(){
    low_batt_file.open("ev_data.txt", ios::out | ios::app );  //para leer datos ios::in
    if (low_batt_file.is_open()){
        low_batt_file << "id" << setw(20) << "SoC" << setw(20) << "Time(sec)" << setw(20) << "CS_Selected"<<'\n';
        low_batt_file.close();
        set_tittle=true; //permite inicializar solo una vez el tittle
    }
    else std::cerr << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< endl;
}

void TraCIDemo11p::ReadDatafromFile(int car_ID){ //lee si un ev car id tien low battery y ha hecho un reroute
    string line;
    int id,cs_selected;
    double time_soc,soc;

    low_batt_file.open("low_batt_file.txt", ios::in);  //para leer datos ios::in

    if (low_batt_file.is_open()){
        while (low_batt_file >> id >> soc >> time_soc >> cs_selected){
            if (id==car_ID){
                low_batt_file.seekg (0, ios::end);
                //     end = myfile.tellg();
                //     Save_EV_Data_Arrival(end);
                cout << id << " " << soc << " " << time_soc << " " << cs_selected << " " << '\n';
            }
        }
        low_batt_file.close();
    }
    else std::cerr << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< endl;

}

void TraCIDemo11p::Save_EV_Data_At_EV_Arrival(int cs_arrived){  //guada datos del ev cuando hace un reroute ---> LOW BATTERY

    ev_arrival_file.open("ev_arrival_file.txt", ios::out | ios::app );  //para leer datos ios::in
    if (ev_arrival_file.is_open()){
//        ev_arrival_file << getParentModule()->getId() << " " << Battery_SoC() << " " << simTime().dbl() <<" "<< cs_arrived << " " << ChargingTime()<< '\n';  //Chargingtime en minutos
        ev_arrival_file.close();
    }
    else cout << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< '\n';
}


