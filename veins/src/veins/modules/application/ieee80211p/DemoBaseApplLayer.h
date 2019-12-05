//
// Copyright (C) 2016 David Eckhoff <eckhoff@cs.fau.de>
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


#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/DemoBaseApplLayerToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/application/traci/beacon.h" //contiene la tabla de beacons para cada vehiculo
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"

#include <map>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>

//#############################  GLOBAL VARIABLES #######################################
extern int CS1_queue;
extern int CS2_queue;
extern int CS3_queue;
extern double CS1_WaitingTime;
extern double CS2_WaitingTime;
extern double CS3_WaitingTime;
extern bool set_tittle; //permite iniializar el tittle
extern int g_addFFV_ID;
extern int g_addEV_ID;
extern double g_VDensityTimer;
extern int g_EVs;
extern int g_FFVs;
extern bool print_settings;
//########################################################################################

using namespace std;
namespace Veins {

using Veins::AnnotationManager;
using Veins::AnnotationManagerAccess;
using Veins::TraCICommandInterface;
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;

/**
 * @brief
 * Demo application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see DemoBaseApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class DemoBaseApplLayer : public BaseApplLayer {

public:
    ~DemoBaseApplLayer() override;
    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    enum DemoApplMessageKinds {
        SEND_BEACON_EVT,
        SEND_WSA_EVT,
    };

    struct Buffer_Element_Type{int MsgId;simtime_t MsgArivalTime;int Hops;int SenderID;vector <int> TabuList_MSG={0,0,0};};


protected:
    /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
    void handleLowerMsg(cMessage* msg) override;

    /** @brief handle self messages */
    void handleSelfMsg(cMessage* msg) override;

    /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int hops = 0);

    /** @brief this function is called upon receiving a BaseFrame1609_4 */
    virtual void onWSM(BaseFrame1609_4* wsm){};

    /** @brief this function is called upon receiving a DemoSafetyMessage, also referred to as a beacon  */
    //virtual void onBSM(DemoSafetyMessage* bsm){};
    virtual void onBSM(DemoSafetyMessage* bsm);

    /** @brief this function is called upon receiving a DemoServiceAdvertisement */
    virtual void onWSA(DemoServiceAdvertisment* wsa){};

    /** @brief this function is called every time the vehicle receives a position update signal */
    virtual void handlePositionUpdate(cObject* obj);

    /** @brief this function is called every time the vehicle parks or starts moving again */
    virtual void handleParkingUpdate(cObject* obj);

    /** @brief This will start the periodic advertising of the new service on the CCH
     *
     *  @param channel the channel on which the service is provided
     *  @param serviceId a service ID to be used with the service
     *  @param serviceDescription a literal description of the service
     */
    virtual void startService(Channel channel, int serviceId, std::string serviceDescription);

    /** @brief stopping the service and advertising for it */
    virtual void stopService();

    /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
     *
     * @param interval the interval length of the periodic message
     * @param chantype the type of channel, either type_CCH or type_SCH
     */
    virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, ChannelType chantype);

    /**
     * @brief overloaded for error handling and stats recording purposes
     *
     * @param msg the message to be sent. Must be a WSM/BSM/WSA
     */
    virtual void sendDown(cMessage* msg);

    /**
     * @brief overloaded for error handling and stats recording purposes
     *
     * @param msg the message to be sent. Must be a WSM/BSM/WSA
     * @param delay the delay for the message
     */
    virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

    /**
     * @brief helper function for error handling and stats recording purposes
     *
     * @param msg the message to be checked and tracked
     */
    virtual void checkAndTrackPacket(cMessage* msg);

    /**
          * @preprocessa los valores del beacon tree id y MAC ID para agregar o actualizar beeacons en la tabla luego
          *
          * @param treeid , MAC ID
          */
    virtual void Read_Beacon_Fields(DemoSafetyMessage* bsm);

    /**
       * @prepara los valores del beacon recibido para agraegar a la tabla de beacons
       *
       * @param NODE TYPE, COORDS, ARRVIAL TIME
       */

    virtual void Update_NH_Beacon_Fields(DemoSafetyMessage* bsm);
    /**
       * @devuelve las coordenadas el RSU mas ceracano al Nodeposition pasado como parametro
       *
       * @param nodeposition
       */
    virtual Coord CoordClosestRSU(Coord NodePosition);
    /**
         * @define el RSU positions vector (coords) en funcion del numbero configurado en el .ini
         *
         * @param sender node position
         */
    virtual vector<Coord> RSU_Vector(int num_of_RSU_in_INI);
    /**
       * @Actualiza las distancias de referencia para hacer ordenar la tabla  de vecinos || Actualiza la wdist Formula Ahmad???
       *
       * @param sender node position
       */
    virtual double UpdateBeaconDistances_calc_U_dist_Nhg(Coord CurrentNodePosition,Coord NghNodePosition);
    /**
        * @Actualiza la densidad Formula Ahmad???
        *
        * @param
        */
    virtual double NodeDensity();
    /**
            * @Estima el ancho de banda disponible
            *
            * @param
            */
    virtual double calcBandwidthEstimation(DemoSafetyMessage* bsm);
   /**
            * @REVISO el estado de bateria del EV cada handleposition (default=1s)
            *
            * @param
            */
    virtual void EV_LowBattery();
    /**
              * @Actualiza la NNT con el beacon recibido
              *
              * @param
              */
    virtual void UpdateNNT(DemoSafetyMessage* bsm);
    /**
              * @Antes de enviar EL MSG, en caso de LOW BATTERY (SREQ=true) actualizo el next hop y hop count
              *
              * @param
              */
    virtual bool FWD_MSG(int OriginalSenderID, DemoSafetyMessage* bsm, string MsgType, int MsgID);
    /**
              * @  AT NODE *** si se recibe un msg del rsu con SRESP = true se hace el reroute hacia la CS
              *
              * @param bsm
              */
    virtual void SRESP_received_Reroute_SUMO();
    /**
              * @  AT NODE *** se decide el NH con el simple routing protcol. Closest node to destiantion (RSU->EV)
              *
              * @param bsm,
              */
    virtual void Send_to_Closest_Node_to_Destination(DemoSafetyMessage* bsm);
    /**
              * @RETURN random number uniform distribution
              *
              * @param max number to random number
              */
    virtual int RandomNumber(int maxnumber);
    /**
              * @DE ACUERDO AL METODO DE SEELCCION (DISTANCE) se selecciona un CS para el current vehicle
              *
              * @param
              */
    virtual string CS_Selection();
    /**
              * @REROUTE to edge ID
              *
              * @param edgeID de la CS selected
              */
    virtual void Go_to_CS(string edgeID);
    /**
              * @  actualiza el numero de saltos. CASE 1: NEW BEACON CASE 2: RECEIVED BSM AND FORWARD
              *
              ** @param bsm
              ** */
    virtual int HopsUpdate(DemoSafetyMessage* bsm);
    /**
              * @  2hGAR
              *
              * @param
              */
    virtual bool TwohGAR(int OriginalSenderID, DemoSafetyMessage* bsm,string MsgType, int MsgID);
    /**
              * @  AT NODE *** (SREQ) update beacon bsm field with next hop according to mmmr
              *
              * @param bsm,
              */
    virtual bool At_Node_UpdateNextHopFields_Once_MMMR_Select_NH(DemoSafetyMessage* bsm,LAddress::L2Type recipientAddress);
    /**
              * @ AT RSU *** (SRESP) update beacon bsm field with next hop according CLOSER NODE TO DESTINATION NODE ----> PENDIENTE PROTOCOLO PARA ENVIA AL NODO ORIGEN DESDE EL RSU *
              *
              * @param bsm,
              */
    virtual void At_RSU_UpdateNextHopFields(DemoSafetyMessage* bsm);
    /**
              * @  TABU LIST UPBADE FROM/TO BEACON
              *
              * @param bsm and next hop to add to tabulist
              */
    virtual void TabuList_Read_Update(DemoSafetyMessage* bsm,string ReadOrUpdate, LAddress::L2Type nexthopAddress,bool NHisRSU);
    /**
             * @  update NNT and return true if NNT is empy false else
             *
             * @param bsm
             * */
    virtual bool NNT_Update();
    /**
             * @  AT NODE *** se decide si se debe enviar a un rsu o a un EV
             *
             * @param bsm,
             */
    virtual LAddress::L2Type  RSU_To_EV(Coord RecipientCoord, LAddress::L2Type RecipientAddress);
    /**
                * @  AT NODE *** IMPORTANTE REVISA LAS CONDCIONES DEL 2HGAR DE LOS NODOS EN LA NNT
                *
                * @param bsm,
                */
    virtual void SAVE_MSG_S_SIGNAL(DemoSafetyMessage* bsm);

    virtual void FWD_MSG_TO_NH(DemoSafetyMessage* bsm, LAddress::L2Type nexthopAddress, string MsgType, double My_Curr_DistanceToRSU, bool NHisRSU);

    virtual void BUFFER_MSG(DemoSafetyMessage* bsm, string MsgType);

    virtual void UPDATE_BEACON_NH_FIELDS_WTIH_CURRENT_NODE_INFO(DemoSafetyMessage* bsm, double My_Curr_DistanceToRSU);

    virtual void PRINT_BUFFER();

    virtual void BUFFER_MSG_TIMEOUT();

    virtual double MMMR_Score(double U_dist, double NodeDensity, double NodeBandwidth);

    virtual double Local_N_Score_respect_n_entry(Coord n_position, double Local_N_Bandwidth);

    virtual void Update_NH_Fields(DemoSafetyMessage* bsm, LAddress::L2Type nexthopAddress);
    ///======================================================================================///
    ///======================  MANTEIN VEHICLES DENSITY ====================================///
    ///======================================================================================///
    virtual void Density();                           //mantiene la densidad de EV y   FFV definidos en el .ini
    virtual void Complete_FFVs(int Running_Vehicles);
    virtual void EV_Density();
    virtual void FFV_Density();
    virtual void Add_FFV();
    ///======================================================================================///
    ///============================= SAVE RESULTS  =========================================///
    ///======================================================================================///
    virtual void Save_SREQ_MSG(int treeID, string Send_Recevied, int hops, simtime_t Creation_Arrival_Time);
    virtual void Save_SREQ_MSG_Tittle();
    ///======================================================================================///

protected:
    /* pointers ill be set when used with TraCIMobility */
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;

    AnnotationManager* annotations;
    DemoBaseApplLayerToMac1609_4Interface* mac;

    /* support for parking currently only works with TraCI */
    bool isParked;

    /* BSM (beacon) settings */
    uint32_t beaconLengthBits;
    uint32_t beaconUserPriority;
    simtime_t beaconInterval;
    bool sendBeacons;

    /* WSM (data) settings */
    uint32_t dataLengthBits;
    uint32_t dataUserPriority;
    bool dataOnSch;

    /* WSA settings */
    int currentOfferedServiceId;
    std::string currentServiceDescription;
    Channel currentServiceChannel;
    simtime_t wsaInterval;

    /* state of the vehicle */
    Coord curPosition;
    double curSpeed;
    string vtype;       //EV OR FFV(PASENGER)
    LAddress::L2Type myId = 0;
    int mySCH;

    /* stats */
    uint32_t generatedWSMs;
    uint32_t generatedWSAs;
    uint32_t generatedBSMs;
    uint32_t receivedWSMs;
    uint32_t receivedWSAs;
    uint32_t receivedBSMs;

    /* messages for periodic events such as beacon and WSA transmissions */
    cMessage* sendBeaconEvt;
    cMessage* sendWSAEvt;

    //#######################################################################################//
    //################################       DEBUG     ######################################//
    //#######################################################################################//
      bool printDebug;
    //#######################################################################################//
    //################################       RSU     ########################################//
    //#######################################################################################//
    int NumberofRSU;
    Coord RSU0;Coord RSU1;Coord RSU2;Coord RSU3;Coord RSU4;Coord RSU5;Coord RSU6;Coord RSU7;Coord RSU8;Coord RSU9;Coord RSU10;
    //#######################################################################################//
    //                                      TABLE OF BEACONS (NNT)                           //
    //#######################################################################################//
    int UpdateNodeNeighborsTable;
    BeaconList ListBeacon;
    LAddress::L2Type Table_Ngh_NodeId;             //MAC identificdor
    string Table_Ngh_NodeType;                     //rsu or node
    int Table_Ngh_Msg_TreeID;                      //busca el msg en la tabla de beacons
    Coord Table_Ngh_Coord;
    double Table_Beacon_ArrivalTime;
    int Packet_Size;
    LAddress::L2Type Msg_RecipienAddress;
    LAddress::L2Type Msg_DestinationAddress;
    string Node_Type_Destination;
    int Num_of_Metrics_MMMR;
    int Time_to_Start_Stop_sending_MSG;
    int SimTimeParameter;
    simtime_t TimeToEndSendingMSGs;
    //#######################################################################################//
    //############################ ELECTRIC VEHICLE   #######################################//
    //#######################################################################################//
    int BatteryCapacity;
    double BatteryThreshold;
    double MyBatterySoC;
    double curBattery;
    string nodeType;
    bool SREQ;
    bool SRESP_Received;
    bool reroute;
    bool EV_Service_Msg;
    //#######################################################################################//
    //############################ DISTANCES TO NEIGHBORS ###################################//
    //#######################################################################################//
    struct NodeRefDist {double dsr; double dsd; double drd;} DistanceToNeighbors;
    double DistanceThreshold;
    double DensityThreshold;
    //#######################################################################################//
    //###########  CHARGING STATIONS -> DEPENDS ON SUMO MAP         #########################//
    //#######################################################################################//
    struct CS {int id; Coord CS_Location; std::string edgeID; double WaitingTime; int queue;} CS1, CS2;
    std::map<int,double> MapDistancetoCSs; //mapeo de cdistancias del nodo a cada cs
    std::map<int,CS> ChargingStations; //MAP CHARGING STATIONS
    int numberofCSs;
    int FastChargingStationPower;
    bool ClosestCSinDistance;
    string CS_Selected_ID;
    //#######################################################################################//
    //#########################  SAVE DATA TO TXT############################################//
    //#######################################################################################//
    fstream low_batt_file;
    fstream ev_arrival_file;
    fstream SREQ_MSG;
    string pathToSaveTXT;
    int run_num;
    string SavToFile;
    string Conf_Name;
    //#######################################################################################//
    //#########################  ROUTING PROTOCOL  +#########################################//
    //#######################################################################################//
    int Conter_SREQ_Msgs;
    int Conter_SRESP_Msgs;
    int MaxNumberofHops;
    bool MMMR_Only;
    //#######################################################################################//
    //#########################  DENSITY         ############################################//
    //#######################################################################################//
    int Max_EV_Density;
    int Max_FFV_Density;
    int Check_Density_Interval;
    int Initialized_Vehicles;
    int Total_Vehicles_Running;
    int Numm_of_Vehicles_Routes;
    //#######################################################################################//
    //#########################  LUIS 2HGAR      ############################################//
    //#######################################################################################//
    LAddress::L2Type NH_Address;
    double NH_GS;
    double NH_Dst_to_Dest;
    bool Tabu;
    vector <int> TabuList;
    double CurrentNodeBestNH;
    //#######################################################################################//
    //#########################      BUFFER      ############################################//
    //#######################################################################################//
    simtime_t Buffer_TimeOut;
    simtime_t Free_Buffer_SEND_msg;
    DemoBaseApplLayer::Buffer_Element_Type Msg_Info;
    list<DemoBaseApplLayer::Buffer_Element_Type> Buffer_List;  //INCLUYE MSG TREEID Y ARRIVAL_TIME
   // list<DemoSafetyMessage*> Buffer_MSG;  //INCLUYE MSG TREEID Y ARRIVAL_TIME
};

} // namespace Veins
