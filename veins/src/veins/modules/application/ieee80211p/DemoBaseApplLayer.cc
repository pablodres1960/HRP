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

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

//#############################  GLOBAL VARIABLES #######################################
int CS1_queue=0;
int CS2_queue=0;
int CS3_queue=0;
double CS1_WaitingTime=0;
double CS2_WaitingTime=0;
double CS3_WaitingTime=0;
bool set_tittle = false;
int g_addFFV_ID=1000;
int g_addEV_ID=3000;
int g_EVs=0;
int g_FFVs=0;
double g_VDensityTimer=0;
bool print_settings = false;
//#############################################################################################################################
bool map_compare(const std::pair<int, double>& lhs,const std::pair<int, double>& rhs){return lhs.second < rhs.second;}
//##############################################################################################################################

using namespace Veins;

void DemoBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");

        dataLengthBits = par("dataLengthBits");
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");

        wsaInterval = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

//#######################################################################################//
//################################       DEBUG  ########################################//
//#######################################################################################//
        printDebug = par("printDebug").boolValue();
//#######################################################################################//
//################################       RSU     ########################################//
//#######################################################################################//
        NumberofRSU = par("NumberofRSU");
        //READ RSU POSITIONS
        RSU0.x = par("RSU0X").longValue();RSU0.y = par("RSU0Y").longValue();RSU0.z = 40;
        RSU1.x = par("RSU1X").longValue();RSU1.y = par("RSU1Y").longValue();RSU1.z = 40;
        RSU2.x = par("RSU2X").longValue();RSU2.y = par("RSU2Y").longValue();RSU2.z = 40;
        RSU3.x = par("RSU3X").longValue();RSU3.y = par("RSU3Y").longValue();RSU3.z = 40;
        RSU4.x = par("RSU4X").longValue();RSU4.y = par("RSU4Y").longValue();RSU4.z = 40;
        RSU5.x = par("RSU5X").longValue();RSU5.y = par("RSU5Y").longValue();RSU5.z = 40;
        RSU6.x = par("RSU6X").longValue();RSU6.y = par("RSU6Y").longValue();RSU6.z = 40;
        RSU7.x = par("RSU7X").longValue();RSU7.y = par("RSU7Y").longValue();RSU7.z = 40;
        RSU8.x = par("RSU8X").longValue();RSU8.y = par("RSU8Y").longValue();RSU8.z = 40;
        RSU9.x = par("RSU9X").longValue();RSU9.y = par("RSU9Y").longValue();RSU9.z = 40;
        RSU10.x = par("RSU10X").longValue();RSU10.y = par("RSU10Y").longValue();RSU10.z = 40;
//#######################################################################################//
//############################//NODE NEIGHBORS TABLE ####################################//
//#######################################################################################//
        UpdateNodeNeighborsTable = par("UpdateNodeNeighborsTable");
        Packet_Size = par("Packet_Size");
        //msg_interval = par("msg_interval").doubleValue();
        Num_of_Metrics_MMMR = par("Num_of_Metrics_MMMR").longValue();
        Time_to_Start_Stop_sending_MSG = par("Time_to_Start_Stop_sending_MSG").longValue();
        SimTimeParameter = par("SimTimeParameter").longValue();
        TimeToEndSendingMSGs = SimTimeParameter - Time_to_Start_Stop_sending_MSG;
//#######################################################################################//
//################################## DISTANCES TO NEIGHBORS #############################//
//#######################################################################################//
        DistanceThreshold = par("DistanceThreshold");
        DensityThreshold = par("DensityThreshold");
//#######################################################################################//
//############################//electric vehicle//#######################################//
//#######################################################################################//
        BatteryCapacity =  par("BatteryCapacity").longValue();
        BatteryThreshold = par("BatteryThreshold").doubleValue();
        nodeType = getParentModule()->getName();
        curBattery = 0;
        SREQ = false;
        SRESP_Received = false;
        reroute = false;

//#######################################################################################//
////#########  Charging Stations debe actualizarse segun el map #########################//
//#######################################################################################//
        numberofCSs = par("numberofCSs").longValue();
        FastChargingStationPower =par("FastChargingStationPower").longValue();
        ClosestCSinDistance = par("ClosestCSinDistance").boolValue();

         //gneE42
        CS1.id = 1;
        CS1.CS_Location.x = par("CS1_x_Coord").doubleValue();
        CS1.CS_Location.y = par("CS1_y_Coord").doubleValue();
        CS1.CS_Location.z = 0;
        CS1.edgeID = par("CS1").stringValue();
        //gneE33
        CS2.id = 2;
        CS1.CS_Location.x = par("CS1_x_Coord").doubleValue();
        CS1.CS_Location.y = par("CS1_y_Coord").doubleValue();
        CS2.CS_Location.z= 0;
        CS2.edgeID = par("CS2").stringValue();

        ChargingStations[1]=CS1;
        ChargingStations[2]=CS2;

//#######################################################################################//
//#########################  ROUTING PROTOCOL  +#########################################//
//#######################################################################################//
        Conter_SREQ_Msgs = 0;
        Conter_SRESP_Msgs = 0;
        MaxNumberofHops = par("MaxNumberofHops").longValue();
        MMMR_Only = par("MMMR_Only").boolValue();
        Median_msg_interval = par("Median_msg_interval").longValue();
//#######################################################################################//
//#########################  2hGAR LUIS  ################################################//
//#######################################################################################//
        Tabu = par("Tabu").boolValue();
        TabuList = {0,0,0};
        LongTabu = par("LongTabu").boolValue();
        LongTabuList = {0,0,0,0,0,0};

//#######################################################################################//
//#########################  SAVE DATA TO TXT############################################//
//#######################################################################################//
        pathToSaveTXT = par("pathToSaveTXT").stdstringValue();
        run_num = par("run_num").longValue();
        Conf_Name = par("Conf_Name").stringValue();
        SavToFile = pathToSaveTXT+Conf_Name+"_"+std::to_string(run_num)+".txt";
//#######################################################################################//
//#########################     DENSITY       ###########################################//
//#######################################################################################//
        Numm_of_Vehicles_Routes = par("Numm_of_Vehicles_Routes").longValue();
        Max_EV_Density = par("Max_EV_Density").longValue();
        Max_FFV_Density = par("Max_FFV_Density").longValue();
        Check_Density_Interval = par("Check_Density_Interval").longValue();
        Total_Vehicles_Running = Max_EV_Density + Max_FFV_Density;
//########################      BUFFER     ######################################################//
        Buffer_TimeOut = par("Buffer_TimeOut").longValue();
//########################      PRINT SETTIGNG     ######################################################//

        if(!print_settings){
           // cout<<"\n";
            std::cerr<<"=========================="<<endl;
            std::cerr<<"Settings:"<<Conf_Name<<endl;
            std::cerr<<"=========================="<<endl;
            if(Tabu){
                if(LongTabu){
                    std::cerr<<"LongTabuList(TL):"<<LongTabu<<endl;
                    std::cerr<<"TL size:"<<LongTabuList.size()<<endl;
                }
                else{
                    std::cerr<<"TabuList(TL):"<<Tabu<<endl;
                    std::cerr<<"TL size:"<<TabuList.size()<<endl;
                }
            }
            std::cerr<<"Buffer:"<<Buffer_TimeOut<<"s"<<endl;
            std::cerr<<"MMMR:"<< MMMR_Only<<endl;
            std::cerr<<"Hops Max:"<<MaxNumberofHops<<endl;
            std::cerr<<"Sim time:"<<SimTimeParameter<<endl;
            std::cerr<<"=========================="<<endl;
            print_settings = true;
        }
//###########################################################################################//

    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        // simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2 * mac->getSwitchingInterval() << "). This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
    }
}

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    // check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        // firstEvent is within a sch interval
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        // firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int hops)
{
//===============================   SAVE TREEID   ====================================================================
    wsm->setBuffer_TreeId(wsm->getTreeId());
//===================================================================================================
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);
//===================================================================================================
    wsm->setCurrentNodeAddress(myId); // store MAC address for quick access
    wsm->setCurBattery(curBattery); //EV VEHICLE BATTERY SOC
    wsm->setNodeType(nodeType.c_str()); //NODO O RSU
    wsm->setNodeDensity(NodeDensity()); //densidad del current node en su NNT
//===================================================================================================
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    simtime_t tbt = myMacp->getTotalBusyTime();
    simtime_t idleTime =(simTime()-tbt)/simTime();
    wsm->setIdleTime(idleTime.dbl());
//======================================== PABLO ==========================================================
    wsm->setSREQ(SREQ);
    wsm->setSRESP(SRESP_Received);
    wsm->setHopsCounter(hops);
    wsm->setFinalSenderPosition(curPosition);
    wsm->setSenderAddress(myId);
///======================================= LUIS 2hGAR ========================================================///

///======================================= Inicialize to 0 TabuList =========================================
    TabuList = {0,0,0};
    int TL_size = TabuList.size();
    for (int i=0; i < TL_size;i++){wsm->setTabuList(i,TabuList[i]);}

    LongTabuList = {0,0,0,0,0,0};
    int Long_TL_size = LongTabuList.size();
    for (int j=0; j < Long_TL_size;j++){wsm->setLongTabuList(j,LongTabuList[j]);}

//======================================= Inicialize NH_GS ================================================
    if(nodeType == "rsu"){
        wsm->setNH_GS(1);
        wsm->setNH_Address(myId);
        wsm->setNH_DstToDestination(0);
    }else{
        double DistToDestination = curPosition.distance(CoordClosestRSU(curPosition));
        wsm->setNH_Address(myId);
        wsm->setNH_DstToDestination(DistToDestination);        //puede introducir demoras
        wsm->setNH_GS(-1);    //GS distancia   - > LUIS infinito
    }
///==========================================================================================================///

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1)); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj){

    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());

    if (FindModule<TraCIMobility*>::findSubModule(getParentModule())){
        TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());
        curSpeed = mySpeed->getSpeed();
        vtype = mySpeed->getVehicleCommandInterface()->getTypeId();
//===========================  VEHICLES DENSITY CHECKED EVERY Check_Density_Interval SECONDs ========================================================
        Density();
//===============================  LOW BATTERY CHECKED EVERY 1 SECOND ===========================================================
        if(traciVehicle->getTypeId() != "passenger"){                         //vehicle type defined in sumo .conf
            curBattery = traciVehicle->getCurrentBatteryCapacity();           //electric vehiclle with battery device
            if(!SREQ){EV_LowBattery();}                                       // compruebo si el EV tiene bateria baja
        }else{ curBattery = 0;}                                               //passegner vehicle
    }else{curSpeed = 0;}                                                      //RSU static node
}

///=====================================================================================================================================///
///                        BEACON FIELDS PROCESSING (GET || COMPUTE || UPDATE)
///=====================================================================================================================================///

void DemoBaseApplLayer::Update_NH_Beacon_Fields(DemoSafetyMessage* bsm) {  ///--> FROM BEACON

    Table_Ngh_NodeType = bsm->getNodeType();                                  //rsu or node
    Table_Ngh_Coord = bsm->getSenderPos();                                    //posicion del node que envia el beacon
    Table_Beacon_ArrivalTime =  bsm->getArrivalTime().dbl();                  //timepo de llegada del beacon

///==================================  NEW FIELDS 2hGAR =========================================================
///================== Llamar funciones del NNT para elegir los nodos candidatos de mi NNT  ======================

    NH_Address = bsm->getNH_Address();                                        // Indica el next hop del current node
    NH_GS = bsm->getNH_GS();
    NH_Dst_to_Dest = bsm->getNH_DstToDestination();

    if(nodeType != "rsu"){curPosition = mobility->getPositionAt(simTime());}   //actualizo la posicion del nodo para calcular la distancia al RSU
}

void DemoBaseApplLayer::Read_Beacon_Fields(DemoSafetyMessage* bsm) {       ///--> FROM BEACON

    Table_Ngh_NodeId = bsm->getCurrentNodeAddress();                   // MAC identificdor del beacon recibido
    Table_Ngh_Msg_TreeID = bsm->getTreeId();                           // busca el msg en la tabla de beacons
    Msg_RecipienAddress = bsm->getRecipientAddress();                  // Next hop address
    Msg_DestinationAddress = bsm->getDestinationAddress();             // Final recipient
    Node_Type_Destination = bsm->getNode_Type_Destination();           // Destination RSU or EV
}

int DemoBaseApplLayer::HopsUpdate(DemoSafetyMessage* bsm){
    int HopCount = bsm->getHopsCounter();                                 //HOP COUNT SENDER ADDRESS UPDATED JUST WHEN SENDER NODE
    //Case: New beacon  HopCount == 0
    if(HopCount == 0){bsm->setSenderAddress(myId);}                       //SI ES EL SENDER ORIGINAL SE CONFIGURA SU ID ADDRESS y las coordenadas del nodo que origina el beacon
    //Case: Forward msg
    bsm->setHopsCounter(HopCount+1);                                      // incremento en 1 el numero de HOPS CUANDO NODE REENVIA MSG. DE ESTO DEPENDE EL ONBSM PARA ACTUALIZAR LA NNT

    return HopCount;
}

///=====================================================================================================================================///
///                           SELF MESSAGE -> BEACON
///=====================================================================================================================================///

void DemoBaseApplLayer::handleSelfMsg(cMessage* msg){  //SE ENCARGA DEL ENVIO DE MENSAJES

    switch (msg->getKind()) {
        case SEND_BEACON_EVT: {
            DemoSafetyMessage* bsm = new DemoSafetyMessage();
///==============================================================================================================================
            populateWSM(bsm);                                       //actualiza campos del nuevo beacon
////==============================================================================================================================
            FWD_MSG(bsm->getSenderAddress(),bsm,"beacon",bsm->getTreeId());                 //FILTER TYPE OF MSSAGES AND DECIDE SI REENVIAR EL MSG O ENVIAR A BUFFER
////==============================================================================================================================
            sendDown(bsm);
            scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
            break;
        }
        case SEND_WSA_EVT: {
            DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
            populateWSM(wsa);
            sendDown(wsa);
            scheduleAt(simTime() + wsaInterval, sendWSAEvt);
            break;
        }
        default: {
            if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

///=====================================================================================================================================///
///                           ON BEACON  -> (BEACON OR MSG)
///=====================================================================================================================================///

void DemoBaseApplLayer::onBSM(DemoSafetyMessage* bsm) {   //SE PROCESA LOS CAMPOS DEL EL MSG RECIBIDO  --> SOLO VEHICLES

///==========================================================================================================================
///===================  JUST  NODES ========   RSU has its own onBSM function   =============================================
///==========================================================================================================================

    Read_Beacon_Fields(bsm);
    bool Msg_Processed = ListBeacon.SearchBeaconTreeID(Table_Ngh_Msg_TreeID);    //VERIFICA SI YA HE PROCESADO ESTE BEACON TREEID. EVITA GUARDAR BEACONS FORWARDED

///======================================= ACTUALIZA NNT  -- NODE  ==========================================================
    //actualiza la NNT solo con beacons no con MSGs
    if(!bsm->getSREQ()){if(bsm->getHopsCounter() <= 1){if(!Msg_Processed){UpdateNNT(bsm);}}}         //unpdate NNT solo de los vecinos a 1 hop

///=================================   FORWARD NODE TO NODE WITH DST RSU   ==================================================
    if(Msg_RecipienAddress == myId){                                             //el beacon  es para el current node y hace el reenvio
        if(Node_Type_Destination == "rsu"){                                      //Verifico el campo en el beacon para ver si el destino del msg es un rsu (RSU->node)

            if(FWD_MSG(bsm->getSenderAddress(),bsm,"fwd",bsm->getBuffer_TreeId())){                      //actualizo los campos del beacon y veo si hay un next hop conveniente
                bsm->setCurrentNodeAddress(myId);
                sendDown(bsm->dup());                                            ///envio copia del beacon -> Forward msg*****
            }else{
                BUFFER_MSG(bsm,"fwd");                                           /// SOY EL MEJOR CANDIDATO PARA ENVIAR -> LLENO LOS CAMPOS NH CON MIS DATOS Y BUFFER MSG
            }
        }

///===========================    REVISA SI EL RSU  ENVIA SRESP ====== RSU -> NODE ===========================================

        else if((Node_Type_Destination == "EV") && (bsm->getSRESP())){           //'EV' no tiene que ver con el .rou
            if(Msg_DestinationAddress == myId){SRESP_received_Reroute_SUMO();}   //Msg final destination is current node (DESTIANTION ADDRESS=myId) || Si se recibe un mensaje de un rsu con final dst myid, se hace un reroute a la CS mas cercana
            else{
               // Foward_NtoN(bsm, "fwd");                                       // SRESP -> PENDIENTE PROTOCOLO DE ENRUTAMIENTO RSU -> NODE ********
            }
        }
    }   /// ELSE IF MSG IS NOT FOR ME  OR IS BEACON ->  CAPA MAC FILTRA EL BEACON SI NO ES PARA MI  ***********************
}

///=====================================================================================================================================///
///                                FORWARD MSG
///=====================================================================================================================================///

bool DemoBaseApplLayer::FWD_MSG(int OriginalSenderID, DemoSafetyMessage* bsm, string MsgType, int MsgID){  /// FILTER MSG TYPE (BEACON, MSG, FWD) and DECIDE IF FWD OR BUFFER

///===============  HOP COUNT UPDATE CASE 1: NEWBEACON  CASE 2: RECEIVED BSM AND FORWARD ================================================
    int HopCount = HopsUpdate(bsm);
///===============  FIRST STEP -> CHECK IF CURRENT NODE HAS NEIGHBORS (NNT != empy)  ==============================================

    bool SendMsg = false;
    bool empy = true;

    if(nodeType == "node"){
        if(MsgType == "beacon" || MsgType == "MSG"){empy = NNT_Update();}       //NO ES NECESARIO ACTUALIZAR EN EL FWD SOLO NEW BEACON
        else if(MsgType == "fwd"){empy = false;}                                //EN CASO DE FWD SE INTENTA ENVIAR. SI NO TIENE NH, EL MSG SE ENVIA AL BUFFER

///====================  Update BSM field and decide if forward SREQ MSG   =====================================================================================

        if (!empy){
            string NodeTypeDestination = bsm->getNode_Type_Destination();

///================= EV -> RSU (2hGAR - > MMMR)   ==============================================================================================================

            if(NodeTypeDestination == "rsu"){SendMsg = TwohGAR(OriginalSenderID,bsm,MsgType,MsgID);}                   /// AQUI DECIDE SI REENVIAR O NO EL BEACON (2HGAR-> DEPENDE DE SI TENGO CANDIDATOS O NO)

///================= RSU -> EV (REENVIO RSU BEACON)   ==============================================================================================================

            else if(NodeTypeDestination == "EV"){Send_to_Closest_Node_to_Destination(bsm);}     /// PENDIENTE RSU -> EV ROUTING PROTOCOL. POR AHORA CLOSSET NODE TO DESTINATION

        }  ///ELSE -> FWD  SendMsg = false -> se agrega al Buffer del current hop node en la funcion anterior
    }      ///ELSE -> RSU NO HACE REENVIO

    return SendMsg;
}

///=====================================================================================================================================///
///                          ROUTING PROTOCOL  -> 2hGAR
///=====================================================================================================================================///

bool DemoBaseApplLayer::TwohGAR(int OriginalSenderID, DemoSafetyMessage* bsm,string MsgType, int MsgID){   ///antes de enviar el beacon tengo que actualizar el current next hop fields EXTRA FIELD LUIS 2hgar

    bool SendMsg = false;
    bool NHisRSU = false;
    LAddress::L2Type nexthopAddress = 0;
    double My_Curr_DistanceToRSU = curPosition.distance(CoordClosestRSU(curPosition));
///=====================================================================================================================================///
///                                                    TABU LIST READ
///=====================================================================================================================================///
    if(MsgType == "fwd" && Tabu){                      //obtengo del beacon la lista de nodos no elegibles     //*********LA TABU LIST CONTIENE LOS ULTIMOS SALTOS DEL BEACON
        if(LongTabu){LongTabuList_Read_Update(bsm,"READ",nexthopAddress,NHisRSU);}
        else{TabuList_Read_Update(bsm,"READ",nexthopAddress,NHisRSU);}
    }

///=====================================================================================================================================///
///                              1.       REVISO SI HAY UN RSU EN CURRENT NODES' NNT
///=====================================================================================================================================///
    int idRSU = ListBeacon.SearchBeaconRSU();                                ///Return 0 if no rsu. Return Nid(senderaddrress) if rsu in NNT of current node
    if (idRSU != 0){nexthopAddress = idRSU;NHisRSU = true;}                  // SI HAY UN RSU ENTONCES NH ES RSU ID Y NHisRSU TRUE para que no se agrege en la TabuLIst
///=====================================================================================================================================///
///                              2.       2 HGAR Y MMMR CONTITIONS TO SELECT NEXT HOP
///=====================================================================================================================================///
    else{                                                                    // NO HAY UN RSU EN NNT ENTONCES REVISO MI NEXT HOP. TwohGAR_AND_MMMR_Conditions -> REVISA LAS CONDICIONES DEL 2HGAR PARA FILTRAR LOS NODOS CANDIDATOS Y LUEGO SE USA EL MMMR PARA SELECCIONAR EL NEXT HOP
        nexthopAddress = ListBeacon.TwohGAR_Conditions(MMMR_Only, OriginalSenderID,Tabu,TabuList,LongTabu,LongTabuList,myId,MsgType, My_Curr_DistanceToRSU,printDebug,MsgID);    ///CHECK 2hGAR conditions for all n in NNT
    }
///=====================================================================================================================================///
///  REENVIO MSG AL  nexthopAddress ID YA SEA UN RSU O UN NH SELECCIONADO EN  TwohGAR_AND_MMMR_Conditions  ||  PRIMER MENSAJE DE SREQ
///=====================================================================================================================================///

    if(nexthopAddress != myId){                                              //TENGO UN NEXT HOP EN MI NNT Y ACTUALIZO ESTOS DATOS

        if(!MMMR_Only){Update_NH_Fields(bsm,nexthopAddress);}

        if(MsgType == "MSG" || MsgType == "fwd"){                           //FILTRO LOS BEACONS *****************
            FWD_MSG_TO_NH(bsm,nexthopAddress,MsgType,My_Curr_DistanceToRSU,NHisRSU);
            SendMsg = true;
        }
    }

///===================================================================================================================================================///
///   SI DESPUES DE REVISAR MIS VECINOS NO ENCUENTRO NI UN RSU NI UN BEST NEXT HOP (NH) NO REENVIO MENSAJE -> YO SOY EL MEJOR CANDIDATO PARA REENVIO
///===================================================================================================================================================///

    else{  ///CURRENT NODE IS THE BEST NEXT HOP -> sendmsg = false -> se agrega beacon al buffer del current node y no es necesario actualizar NH_Fields
        if(MsgType == "beacon"){
            UPDATE_BEACON_NH_FIELDS_WTIH_CURRENT_NODE_INFO(bsm, My_Curr_DistanceToRSU);     // SOY EL MEJOR CANDIDATO PARA ENVIAR -> LLENO LOS CAMPOS NH CON MIS DATOS ACTUALIZADOs
        }
    }

    return SendMsg;
}

void DemoBaseApplLayer::FWD_MSG_TO_NH(DemoSafetyMessage* bsm, LAddress::L2Type nexthopAddress, string MsgType, double My_Curr_DistanceToRSU, bool NHisRSU){

    At_Node_UpdateNextHopFields_Once_MMMR_Select_NH(bsm,nexthopAddress);                             //ACTUALIZA EL RECIPIENT ADDRESS PARA REENVIO DE BEACON

    if(MsgType == "fwd" && Tabu){                                                                            //TL UPDATE
        if(LongTabu){LongTabuList_Read_Update(bsm,"UPDATE",nexthopAddress,NHisRSU);}
        else{TabuList_Read_Update(bsm,"UPDATE",nexthopAddress,NHisRSU);}
    }
}

void DemoBaseApplLayer::Update_NH_Fields(DemoSafetyMessage* bsm, LAddress::L2Type nexthopAddress){      ///UPDATE NH FIELD WITH NEXTHOP DATA from NNT

    bsm->setNH_Address(nexthopAddress);
    bsm->setNH_GS(ListBeacon.n_S(nexthopAddress));
    bsm->setNH_DstToDestination(ListBeacon.n_DsttoRSU(nexthopAddress));
}

void DemoBaseApplLayer::UPDATE_BEACON_NH_FIELDS_WTIH_CURRENT_NODE_INFO(DemoSafetyMessage* bsm, double My_Curr_DistanceToRSU){

///====================================================================================================================================///
///                                  IN CASE OF MSG TYPE = BEACON -> JUST UPDATE NH FIELDS
///====================================================================================================================================///
    bsm->setNH_Address(myId);
    bsm->setNH_GS(-1);
    bsm->setNH_DstToDestination(My_Curr_DistanceToRSU);
}

///=====================================================================================================================================///
///                               TABU LIST (TL)
///=====================================================================================================================================///

void DemoBaseApplLayer::TabuList_Read_Update(DemoSafetyMessage* bsm,string ReadOrUpdate, LAddress::L2Type nexthopAddress,bool NHisRSU){

    if(ReadOrUpdate == "READ"){for(int i=0;i<TabuList.size();i++){TabuList[i] = bsm->getTabuList(i);}}
    else if (ReadOrUpdate == "UPDATE"){

        if(!NHisRSU){                                            ///SI EL NEXTHOP ES UN RSU NO LO AGREGO AL TABU LIST

            TabuList[2] = TabuList[1];
            TabuList[1] = TabuList[0];
            TabuList[0] = nexthopAddress;
            for(int i=0;i<TabuList.size();i++){bsm->setTabuList(i, TabuList[i]);}

        }else{for(int i=0;i<TabuList.size();i++){bsm->setTabuList(i, TabuList[i]);}}
    }
}

void DemoBaseApplLayer::LongTabuList_Read_Update(DemoSafetyMessage* bsm,string ReadOrUpdate, LAddress::L2Type nexthopAddress,bool NHisRSU){

    if(ReadOrUpdate == "READ"){for(int i=0;i<LongTabuList.size();i++){LongTabuList[i] = bsm->getLongTabuList(i);}}
    else if (ReadOrUpdate == "UPDATE"){

        if(!NHisRSU){                                            ///SI EL NEXTHOP ES UN RSU NO LO AGREGO AL TABU LIST
            LongTabuList[5] = LongTabuList[4];
            LongTabuList[4] = LongTabuList[3];
            LongTabuList[3] = LongTabuList[2];
            LongTabuList[2] = LongTabuList[1];
            LongTabuList[1] = LongTabuList[0];
            LongTabuList[0] = nexthopAddress;

            for(int i=0;i<LongTabuList.size();i++){bsm->setLongTabuList(i, LongTabuList[i]);}
        }else{for(int i=0;i<LongTabuList.size();i++){bsm->setLongTabuList(i, LongTabuList[i]);}}
    }
}

double DemoBaseApplLayer::Local_N_Score_respect_n_entry(Coord n_position, double Local_N_Bandwidth){              ///CALCULA EL GS LOCAL del nodo que recive el beacon.

///============================================================================================///
///                         COMPUTE CURRENT N MMMR SCORE                                       ///
///============================================================================================///
    double Local_U_dist;                                            //u dist computation
    int Neighbors = ListBeacon.CounterBeacons();

    if(Neighbors == 0){                                               ///CALCULO GS CUANDO LA NNT ESTA EMPY
        Local_U_dist = 0;                                            //NO TENGO VECINOS MI SCORE DEBE SER 0
        /*
        double TR = DistanceThreshold;
        double DistToDestination = curPosition.distance(CoordClosestRSU(curPosition));
        if (DistToDestination <= TR){Local_U_dist = 1;}
        else {Local_U_dist = (TR/DistToDestination);}
         */
    }else if(Neighbors != 0){                                        ///CALCULO GS EN BASE al current n node in N NNT
///============================================================================================///
///        INGRESO PARAMETROS CAMBIADOS PARA CALCULAR EL GS CON RESPECTO AL NODO ACTUAL        ///
///============================================================================================///
        Local_U_dist = UpdateBeaconDistances_calc_U_dist_Nhg(n_position,curPosition);   // Calculo el U-dist del nodo actual  -> Note is computed with updated data RFESPECT CURRENT NODE AND TOP ENTRY IN NNT AS AN BSM INPUT FORN IT
    }
///============================================================================================///
    double Local_NodeDensity = NodeDensity();                     //OBTAIN LOCAL N DENSITY computation
///============================================================================================///
    double Local_N_Score_respect_n = MMMR_Score(Local_U_dist,Local_NodeDensity, Local_N_Bandwidth);   ///valor a devolver con el score del current node con respecto al n node en la NNT
    return Local_N_Score_respect_n;
}

double DemoBaseApplLayer::MMMR_Score(double U_dist, double NodeDensity, double NodeBandwidth){
    double N_Score = 0;
///============================================================================================///
///                         2 METRICS => DISTANCIA & DENSIDAD                                  ///
///============================================================================================///
    if(Num_of_Metrics_MMMR == 2){
        N_Score = (U_dist + NodeDensity)*0.5;
    }
///============================================================================================///
///                 3 METRICS => DISTANCIA & DENSIDAD & BANDWIDTH                              ///
///============================================================================================///
    else if(Num_of_Metrics_MMMR == 3){
        N_Score = (U_dist + NodeDensity + (NodeBandwidth/6000000))*0.33;   //assume 6Mbps
    }
    else{
        std::cerr<<"ERROR -> NUMBERO DE METRICAS INVALIDO"<<endl;
    }
    return N_Score;
}

///=====================================================================================================================================///
///                                        BUFFER MSG
///=====================================================================================================================================///

void DemoBaseApplLayer::BUFFER_MSG(DemoSafetyMessage* bsm, string MsgType){

///====================================================================================================================================///
///                                             CARRY AND FORWARD FUNCTION
///====================================================================================================================================///

    if(MsgType == "MSG" || MsgType == "fwd"){
        Msg_Info.MsgArivalTime = simTime() + Buffer_TimeOut;                                               ///EXPIRATION TIME IN BUFFER;
        Msg_Info.MsgId = bsm->getBuffer_TreeId();
        Msg_Info.SenderID = bsm->getSenderAddress();

        int hops = bsm->getHopsCounter();
        if(MsgType == "fwd"){Msg_Info.Hops = hops-1;}               ///EN EL CASO QUE UN MSG RECIBIDO NO SE A PODIDO ENVIAR SE DEBE REDUCIR LOS HOPS
        else{Msg_Info.Hops = hops;}

        ///SAVE TABU LIST IN MSG TO BUFFER
        if(Tabu){
            if(LongTabu){
                for(int i=0;i<LongTabuList.size();i++){
                    Msg_Info.LongTabuList_MSG[i] = bsm->getLongTabuList(i);
                }
            }
            else{
                for(int i=0;i<TabuList.size();i++){
                    Msg_Info.TabuList_MSG[i] = bsm->getTabuList(i);
                }
            }
        }

        Buffer_List.push_back(Msg_Info);                                                                   /// ADD MSG INFO IN LAST POSITION IN BUFFER

        if(printDebug){PRINT_BUFFER();}
    }
}

void DemoBaseApplLayer::PRINT_BUFFER(){

    list<DemoBaseApplLayer::Buffer_Element_Type>::iterator it;
    DemoBaseApplLayer::Buffer_Element_Type Buffer_Msg;

    for(it=Buffer_List.begin();it!=Buffer_List.end();it++){
        Buffer_Msg = *it;
        std::cerr<<myId<<"  -- ADD:"<<  Buffer_Msg.MsgId <<"          H:"<<Buffer_Msg.Hops<<"         SA:"<<Buffer_Msg.SenderID<<"    EXP_TIME:"<<Buffer_Msg.MsgArivalTime<<"  T:"<<simTime()<<endl;
    }

}

void DemoBaseApplLayer::BUFFER_MSG_TIMEOUT(){

    if(!Buffer_List.empty()){

        DemoBaseApplLayer::Buffer_Element_Type Buffer_Msg;
        list<DemoBaseApplLayer::Buffer_Element_Type>::iterator it;
        list<DemoBaseApplLayer::Buffer_Element_Type>::iterator DEL;

        bool Expired_MSG = false;
        bool Expired_MSG_Loop = false;

        while(!Expired_MSG && !Expired_MSG_Loop){

            for(it=Buffer_List.begin();it!=Buffer_List.end();it++){
                Buffer_Msg = *it;
                if(simTime() > Buffer_Msg.MsgArivalTime){
                    DEL = it;
                    Expired_MSG = true;
                }
            }
            Expired_MSG_Loop = true;
        }

        if(Expired_MSG){

           if(printDebug){
                DemoBaseApplLayer::Buffer_Element_Type Buffer_Msg_Temp = *DEL;
                std::cerr<<myId<<"  -- EXP:"<<Buffer_Msg_Temp.MsgId<<"        Buff_size:"<<Buffer_List.size()<<"    T:"<<simTime()<<endl;
            }
           Buffer_List.erase(DEL);                  ///BORRO EL MSG CON TIMEOUT
        }
    }
}

///=====================================================================================================================================///
///                          RSU VECTOR ||  CLOSEST RSU || SIMPLE ROUTING PROTOCOL || CREATE SRESP MSG
///=====================================================================================================================================///

Coord DemoBaseApplLayer::CoordClosestRSU(Coord NodePosition) {   // ESTA FUNCION DEPENDE DEL NUMERO DE RSUS EN EL MAPA

///================================ ADD RSUs ===============================================================
    int num_of_RSU_in_INI = NumberofRSU;                            //initialize number of rsus from .ini
    vector<Coord> CoordRSU = RSU_Vector(num_of_RSU_in_INI);
///=========================================================================================================

    int NumberofRSUVector = CoordRSU.size();
    if(num_of_RSU_in_INI != NumberofRSUVector){if(printDebug)std::cerr<<"ERROR  => Verify Number of RSU in .ned, .ini, .cc"<<endl;}

    Coord CoordClstRSU;
    double MaxDistancia = 10000;
    double DtoRSU[num_of_RSU_in_INI]={0};

    for (int i=0;i<num_of_RSU_in_INI;i++){
        DtoRSU[i] = NodePosition.distance(CoordRSU[i]);   //distancia euclidiana
        if (DtoRSU[i] < MaxDistancia){
            MaxDistancia = DtoRSU[i];
            CoordClstRSU = CoordRSU[i];
        }
    }
    return CoordClstRSU;
}

vector<Coord> DemoBaseApplLayer::RSU_Vector(int num_of_RSU_in_INI){
    vector<Coord> CoordRSU;
    switch(num_of_RSU_in_INI){
        case 0:{std::cerr<<"Error no RSU configured in omnetpp.ini"<<endl;break;}
        case 1:{CoordRSU={RSU0};break;}
        case 2:{CoordRSU={RSU0,RSU1}; break;}
        case 3:{CoordRSU={RSU0,RSU1,RSU2}; break;}
        case 4:{CoordRSU={RSU0,RSU1,RSU2,RSU3}; break;}
        case 5:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4}; break;}
        case 6:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5}; break;}
        case 7:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5,RSU6}; break;}
        case 8:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5,RSU6,RSU7}; break;}
        case 9:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5,RSU6,RSU7,RSU8}; break;}
        case 10:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5,RSU6,RSU7,RSU8,RSU9}; break;}
        case 11:{CoordRSU={RSU0,RSU1,RSU2,RSU3,RSU4,RSU5,RSU6,RSU7,RSU8,RSU9,RSU10}; break;}
        default:{std::cerr<<"RSU number invalid"<<endl;break;}
    }
    return CoordRSU;
}

void DemoBaseApplLayer::Send_to_Closest_Node_to_Destination(DemoSafetyMessage* bsm){
    Coord FinalNodeCoord = bsm->getFinalSenderPosition();
    LAddress::L2Type FinalNodeAddress = bsm->getDestinationAddress();                    // beacon comes from RSU
    LAddress::L2Type recipientAddress = RSU_To_EV(FinalNodeCoord,FinalNodeAddress);      /// *****  simple routing protocol. Closest node to destination

    bsm->setRecipientAddress(recipientAddress);
    bsm->setSenderAddress(myId);
    bsm->setSenderPos(curPosition);
    bsm->setSenderSpeed(curSpeed);
    bsm->setCurrentNodeAddress(myId);

    sendDown(bsm->dup());                                                          ///envio copia del beacon -> Forward TO NH EV
}

LAddress::L2Type  DemoBaseApplLayer::RSU_To_EV(Coord RecipientCoord,  LAddress::L2Type RecipientAddress){
///========================================================================================
///                     PENDIENTE ROUTING PROTOCOL RSU--> EV
///========================================================================================
    LAddress::L2Type nexthopid = 0;

    if ((ListBeacon.CounterBeacons()) !=0){
        if(ListBeacon.SearchBeaconNodeID(RecipientAddress)){nexthopid = RecipientAddress;}        //SI ENCUENTRO EL NODE ID EN MI TABLA, LE ENVIO DIRECTAMENTE
        else{nexthopid = ListBeacon.SearchClstNODE(RecipientCoord);}                              //BUSCA EL NODO QUE ESTE MAS CERCA AL NODO DESTINO
    }else{nexthopid = LAddress::L2BROADCAST();}                                                   // si no tengo aquien enviar, no hay beacon en mi tabla ?????

    return nexthopid;
}

void DemoBaseApplLayer::At_RSU_UpdateNextHopFields(DemoSafetyMessage* bsm){

///===========================================  NNT  update   ====================================================================================
    ListBeacon.PurgeBeacons(UpdateNodeNeighborsTable);                  // ANTES DE ENVIAR EL BEACON ACTUALIZO LA TABLA DE VECINOS
    ListBeacon.SortBeacons(Num_of_Metrics_MMMR);                             // Ordeno la NNT antes de enviar el beacon
///===============================================================================================================================================

///====================================================================================================================================
///====================================   CREATE SERVICE RESPONSE (SRESP Msg)  AT RSU =================================================
///====================================================================================================================================

    Coord FinalNodeCoord = bsm->getFinalSenderPosition();
    LAddress::L2Type FinalNodeAddress = bsm->getSenderAddress();                      // BEACON COMES FROM SOURCE NODE (sender address)
    LAddress::L2Type recipientAddress = RSU_To_EV(FinalNodeCoord,FinalNodeAddress);   /// NH CLosest node to destination node **** ROUTING PROTOCOL PENDIENTE RSU->NODE

    DemoSafetyMessage* Newbsm = new DemoSafetyMessage();                              //create SRESP
    populateWSM(Newbsm,recipientAddress,1);

    Newbsm->setDestinationAddress(FinalNodeAddress);
    Newbsm->setNode_Type_Destination("EV");

    Newbsm->setSenderAddress(myId);
    Newbsm->setSenderPos(curPosition);
    Newbsm->setSenderSpeed(curSpeed);
    Newbsm->setFinalSenderPosition(FinalNodeCoord);
    Newbsm->setSREQ(true);
    Newbsm->setSRESP(true);                                                            //habilita busqueda de CLosest CS

    sendDown(Newbsm);
    Conter_SRESP_Msgs++;
}

///=====================================================================================================================================///
///                                    NODE NEIGHBOR TABLE (NNT)
///=====================================================================================================================================///

void DemoBaseApplLayer::UpdateNNT(DemoSafetyMessage* bsm) {   //SE ACTUALIZA LA NNT con el beacon recibido

    Update_NH_Beacon_Fields(bsm);
///=========================================================================================================///
///                                COMPUTE n MMMR SCORE RESPECT N & CLOSEST RSU                       ///
///=========================================================================================================///
    //================================================= BANDWIDTH ===========================================
    double abe = 0;                                                     ///SAME FOR BOTH NODES sender receiver
    if(Num_of_Metrics_MMMR == 3){abe = calcBandwidthEstimation(bsm);}   ///SAME FOR BOTH NODES sender receiver
    //================================================= U_DIST ==============================================//
    double U_dist_Nhg = UpdateBeaconDistances_calc_U_dist_Nhg(curPosition,Table_Ngh_Coord);                              ///OJO MODIFICA nuevamente distances
    double DistanceToNeighbors_dsr = DistanceToNeighbors.dsr;
    double DistanceToNeighbors_dsd = DistanceToNeighbors.dsd;
    double DistanceToNeighbors_drd = DistanceToNeighbors.drd;
    //================================================= DENSITY =============================================
    double b_NodeDensity = bsm->getNodeDensity();
   //============================================  n  MMMR  Score  ========================================///
    double n_S;
    if(Table_Ngh_NodeType == "node"){n_S = MMMR_Score(U_dist_Nhg,b_NodeDensity,abe);}
    else{n_S = 1;}                                                                       ///MAX GS TO RSU
    //=====================================================================================================///
    double Local_N_S = -1; //initialized pending update

    if(ListBeacon.SearchBeaconNodeID(Table_Ngh_NodeId)){ListBeacon.UpdateBeacon(Table_Ngh_NodeType, Table_Ngh_Msg_TreeID ,Table_Ngh_NodeId,Table_Beacon_ArrivalTime, Table_Ngh_Coord, DistanceToNeighbors_dsr, DistanceToNeighbors_dsd, DistanceToNeighbors_drd, b_NodeDensity, U_dist_Nhg,abe,n_S,NH_Address,NH_GS,NH_Dst_to_Dest,Local_N_S);}
    else{ListBeacon.AddBeacon(Table_Ngh_NodeType, Table_Ngh_Msg_TreeID ,Table_Ngh_NodeId,Table_Beacon_ArrivalTime, Table_Ngh_Coord,  DistanceToNeighbors_dsr , DistanceToNeighbors_dsd, DistanceToNeighbors_drd, b_NodeDensity, U_dist_Nhg,abe,n_S,NH_Address,NH_GS,NH_Dst_to_Dest,Local_N_S);}

    ListBeacon.PurgeBeacons(UpdateNodeNeighborsTable);

///=========================================================================================================///
///                                COMPUTE LOCAL N MMMR SCORE RESPECT n & CLOSEST RSU                       ///
///=========================================================================================================///
//============================================  N  MMMR  Score  ========================================///
    Local_N_S = Local_N_Score_respect_n_entry(Table_Ngh_Coord,abe);                                               ///OJO MODIFICA distances

    if(ListBeacon.SearchBeaconNodeID(Table_Ngh_NodeId)){ListBeacon.UpdateBeacon(Table_Ngh_NodeType, Table_Ngh_Msg_TreeID ,Table_Ngh_NodeId,Table_Beacon_ArrivalTime, Table_Ngh_Coord, DistanceToNeighbors_dsr, DistanceToNeighbors_dsd, DistanceToNeighbors_drd, b_NodeDensity, U_dist_Nhg,abe,n_S,NH_Address,NH_GS,NH_Dst_to_Dest,Local_N_S);}
//======================================================================================================///

    ListBeacon.SortBeacons(Num_of_Metrics_MMMR);
    if(printDebug){ListBeacon.PrintBeacons(myId, "IN");}
}

bool DemoBaseApplLayer::NNT_Update(){    ///Update NNT and return true if NNT is empy

///===========================================  NNT  update   ====================================================================================
    ListBeacon.PurgeBeacons(UpdateNodeNeighborsTable);                          // ANTES DE ENVIAR EL BEACON ACTUALIZO LA TABLA DE VECINOS
    ListBeacon.SortBeacons(Num_of_Metrics_MMMR);                                     // Ordeno la NNT antes de enviar el beacon
///===============================================================================================================================================
    bool empy;
    if (ListBeacon.CounterBeacons() == 0){empy = true;}
    else{empy = false;}

    return empy;
}

///=====================================================================================================================================///
///                                    MMMR -> DENSITY, WDIST, BW
///=====================================================================================================================================///

void DemoBaseApplLayer::At_Node_UpdateNextHopFields_Once_MMMR_Select_NH(DemoSafetyMessage* bsm,LAddress::L2Type recipientAddress){

    bsm->setRecipientAddress(recipientAddress);
    bsm->setDestinationAddress(LAddress::L2ANYCASTRSU());                   ///DIRECCION ANYCAST PARA QUE EL MENSAJE ALCANCE EL RSU MAS CERCANO
    bsm->setSenderPos(curPosition);
    bsm->setSenderSpeed(curSpeed);
    bsm->setSREQ(true);                                                     ///utilizado por ahora para estadisticas
}

void DemoBaseApplLayer::SAVE_MSG_S_SIGNAL(DemoSafetyMessage* bsm){

    int New_MSG_SenderAddress = bsm->getSenderAddress();
    if(New_MSG_SenderAddress == myId){
        Conter_SREQ_Msgs++;
        Save_SREQ_MSG(bsm->getBuffer_TreeId(),"S",bsm->getHopsCounter());          ///save sreq mgs PARA ESTADISTICAS
    }

    ///IMPRIME EL FORWARD PERO NO CONTABILIZA PARA ESTADISTICAS
    if (printDebug){std::cerr<<myId<<"  -- NEW MSG --> "<<bsm->getRecipientAddress()<<"   B_TreeId:"<<bsm->getBuffer_TreeId()<<"  T:"<<simTime()<<endl;}
}

double DemoBaseApplLayer::NodeDensity(){ ///number of neighbors----- let // DENSIDAD EN CADA NODO ??????? --> contNV=c+f?????
    int contNv1=0;
    double contNv=0;// to count neighbors' numbers and include them in the beacon
    contNv1 = (ListBeacon.CounterBeacons());

    if(contNv1 <= DensityThreshold){ // que es DistanceThreshold ??? un limite del RSU o que es ????
        contNv= (-0.000025*contNv1*contNv1)+(0.01*contNv1); // ?????
    }
    return contNv; /// this is the value to return and add it into the table...
}

double DemoBaseApplLayer::calcBandwidthEstimation(DemoSafetyMessage* bsm){
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    simtime_t tbt = myMacp->getTotalBusyTime();
    double k=(((2 * (SLOTLENGTH_11P + SIFS_11P).dbl())+(double(myMacp->gethNumBackoff())*SLOTLENGTH_11P.dbl()))/beaconInterval); //additional overhead
    //collision probability of the hello messages--f (m, N, s)
    double f_m_N_s=(-7.4754*10e-5) * (headerLength+Packet_Size) - (8.9836*10e-3)*(ListBeacon.CounterBeacons()) - (1.4289*10e-3)*curSpeed + 1.9846;
    f_m_N_s>1 ? f_m_N_s=0:f_m_N_s;//fmNs is higher than 1, then fmNs=0---
    double ts = bsm->getIdleTime();//value of free time in channel/maybe remained time from sender/source
    double tr = (simTime().dbl()-tbt.dbl())/simTime().dbl(); //tiempo total que ha estado ocupado hasta este momento from receiver
    double myBitrate = ((DeciderResult80211*)((PhyToMacControlInfo*) bsm->getControlInfo())->getDeciderResult())->getBitrate(); //my transmission rate
    double abe1 = double((1-k)*(1-f_m_N_s)*ts*tr*myBitrate);//available Bw in the node (getPaterentModule)
  ///  abe1=abe1/1000; //---Bw value of each node_nhg... missing-> ABE/C (Bps)
    return abe1; //(kBps)
}

double DemoBaseApplLayer::UpdateBeaconDistances_calc_U_dist_Nhg(Coord CurrentNodePosition, Coord NghNodePosition){
    ///INITIALIZE
    DistanceToNeighbors.dsd = 0;
    DistanceToNeighbors.dsr = 0;
    DistanceToNeighbors.drd = 0;

   // Coord CurrentNodePosition = curPosition;
    //======================================= DSD ================================================
    //DISTANCIA DESDE EL NODO QUE ENVIA HACIA EL DESTINASTION (RSU)
    //devuelve distancia al rsu mas cercano del que envia el beacon (vecino)
    DistanceToNeighbors.dsd = NghNodePosition.distance(CoordClosestRSU(NghNodePosition));   //DISTANCIA EUCLIDIANA
    //============================================================================================

    //======================================= DSR ================================================
    //distancia a mi vecino del que recibo el beacon
    DistanceToNeighbors.dsr = CurrentNodePosition.distance(NghNodePosition);                 //DISTANCIA EUCLIDIANA
    //============================================================================================

    //======================================= DRD ================================================
    //DISTANCIA DEL QUE RECIBE AL RSU
    //devuelve distancia al rsu mas cercano del nodo actual
    DistanceToNeighbors.drd = CurrentNodePosition.distance(CoordClosestRSU(CurrentNodePosition)); //DISTANCIA EUCLIDIANA
    //============================================================================================

    // AHMAD PAPER Multimedia Multimetric Map-Aware Routing Protocol to Send Video-Reporting Messages Over VANETs in Smart Cities

    int TR = DistanceThreshold;
    double U_dist_Nhg = 0;

    if (DistanceToNeighbors.dsd < TR){                                                 //MI NEIGHBOR ESTA A UN SALTO DEL RSU
        U_dist_Nhg = 1;
    }else{
     // Ahmad  double t = DistanceToNeighbors.drd - TR;  //CURRENT NODE IS THE SENDER d(S,D)
     // Ahmad  U_dist_Nhg =  (((-1)*(DistanceToNeighbors.dsd)/t) + (DistanceToNeighbors.drd/t));
//===============================================================================================================================================
//======================================  MODIFICACION PABLO       ==============================================================================
//===============================================================================================================================================
        if(DistanceToNeighbors.dsd < DistanceToNeighbors.drd){                       // YO SENDER ESTOY MAS LEJOS AL RSU QUE MI VECINO, NO PENALIZO CON 0 A MI VECINO  Y CALCULO LA DISTANCIA EN PESO
            double t = TR - DistanceToNeighbors.drd;                                 //CURRENT NODE IS THE SENDER d(S,D)
            U_dist_Nhg =  (DistanceToNeighbors.dsd - DistanceToNeighbors.drd)/t;
        }
//////////////////// ELSE ESTOY MAS CERCA AL RSU QUE MI VECINO Y LO PENALIZO CON CERO///////////////////////
    }

    return U_dist_Nhg;
}

///=====================================================================================================================================///
///                                             VEINS
///=====================================================================================================================================///

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg) //SE ENCARGA DE LA RECEPCION DE MENSAJES
{
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete (msg);
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DemoBaseApplLayer::finish(){
///=============================    CHECK IF VEHICLE DESAPEAR ================================================
    if(nodeType != "rsu"){
         if(vtype == "passenger"){
             int FFVs = g_FFVs;
             if(FFVs!=0 && FFVs > 0){g_FFVs--;}
             else{g_FFVs=0;}
         }else{
             int EVs = g_EVs;
             if( EVs!=0 && EVs > 0){g_EVs--;}
             else{g_EVs=0;}
         }
     }else{
 ///#############################  GLOBAL VARIABLES REINITIALIZE IF RSU #######################################
         CS1_queue=0;
         CS2_queue=0;
         CS3_queue=0;
         CS1_WaitingTime=0;
         CS2_WaitingTime=0;
         CS3_WaitingTime=0;
         set_tittle = false;
         g_addFFV_ID=1000;
         g_addEV_ID=3000;
         g_VDensityTimer=0;
         g_EVs=0;
         g_FFVs=0;
         print_settings = false;
     }
///=======================================================================================================

    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs", receivedWSMs);

    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs", receivedBSMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs", receivedWSAs);
}

DemoBaseApplLayer::~DemoBaseApplLayer()
{
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

void DemoBaseApplLayer::startService(Channel channel, int serviceId, std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        error("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void DemoBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void DemoBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg){

    if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV_TRACE << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}

///=====================================================================================================================================///
///                        CS SELECTION, EV BATTERY, REROUTE
///=====================================================================================================================================///

string DemoBaseApplLayer::CS_Selection(){        //algortimo ue selecciona la CS para el ev requiring energy given its position
    int closest_CS_distance = 0;
    string edgeID;

    if(ClosestCSinDistance){

        for (int i=1;i<=numberofCSs;i++){
            MapDistancetoCSs[i] = traci->getDistance(curPosition, ChargingStations[i].CS_Location, true);
        }

        closest_CS_distance = std::min_element(MapDistancetoCSs.begin(), MapDistancetoCSs.end(), map_compare)->first;

        if (closest_CS_distance != 0)
            edgeID = ChargingStations[closest_CS_distance].edgeID;
        else
            std::cerr<<"ERROR: no se pudo calcular la distancia a las CSs"<<endl;
    }
//=====================================  OTROS ESQUEMAS DE CS SELECTION VAN AQUI ==========================================================
    else{
        std::cerr<<"ERROR: NO SE HA SELECCIONADO EL METODO SE SELECTION DEL A CS in .ini"<<endl;
    }
    return edgeID;
}

void DemoBaseApplLayer::Go_to_CS(string edgeID){
    traciVehicle->changeTarget(edgeID);
    reroute = true;                                              //para que no vuelva a esta funcion a otro enrutamiento
    if(printDebug){std::cerr<<myId<< " -- RRoute -->:"<<edgeID<<" T:"<<simTime().dbl()<<endl;}
    // Save_EV_Data_To_File(CS_Selected);                          //GUARDO DATOS DE LOW BATTERY
}

void DemoBaseApplLayer::EV_LowBattery(){
    double BT = (BatteryThreshold) * BatteryCapacity * 1000; //BatteryThreshold (%) || BatteryCapacity * 1000 (Wh)
    if(curBattery <= BT){
        SREQ = true;
        if (printDebug){std::cerr<<myId<<"  "<<"LB:"<<SREQ<<"  "<<curBattery<<" Time:"<<simTime().dbl()<<endl;}
    }
}

void DemoBaseApplLayer::SRESP_received_Reroute_SUMO(){
    SRESP_Received = true;
    CS_Selected_ID = CS_Selection();
    if(!CS_Selected_ID.empty()){
        Go_to_CS(CS_Selected_ID);
    }
    ///STATISTICS
}

///=====================================================================================================================================///
///                      SAVE DATA, MANTAIN DENSITY, RANDOM NUMBER
///=====================================================================================================================================///

void DemoBaseApplLayer::Density(){

    if(simTime() == 5){
        int Initialized_Vehicles = mobility->getCommandInterface()->getactivevehicles();
        Complete_FFVs(Initialized_Vehicles);
    }
    if(simTime().dbl() >= Check_Density_Interval){
        double VDTimer = g_VDensityTimer;
        if(simTime() >= VDTimer){
            int Running_Vehicles = mobility->getCommandInterface()->getactivevehicles();
            if(Running_Vehicles <= Total_Vehicles_Running){
                FFV_Density();                                                  //mantain FFFV vehicles density
                EV_Density();                                                   //mantain EV vehicles density
                g_VDensityTimer = simTime().dbl() + Check_Density_Interval;
            }
        }
    }
}

void DemoBaseApplLayer::Complete_FFVs(int Running_Vehicles){            //executes at the begining
    g_FFVs = Running_Vehicles;                                           //ASUMO QUE COMIENZO CON SOLO FFVs en el .rou (flows)
    while (Running_Vehicles < Max_FFV_Density){
        Add_FFV();
        Running_Vehicles = mobility->getCommandInterface()->getactivevehicles();
    }
}

void DemoBaseApplLayer::FFV_Density(){
    int FFVs = g_FFVs;
    if(FFVs < Max_FFV_Density){
        Add_FFV();
    }
}

void DemoBaseApplLayer::Add_FFV(){
    string vtype = "passenger";
    string nodeID = std::to_string(g_addFFV_ID++);                                          //ID of the new FFV
    string routeId = std::to_string(RandomNumber(Numm_of_Vehicles_Routes));                                     // Random route for the new vehicle (routes are defined in ,rou)
    bool success = mobility->getCommandInterface()->addVehicle(nodeID, vtype, routeId, simTime(), 0.0, 0.0,0);
    if (success){g_FFVs++;}
}

void DemoBaseApplLayer::EV_Density(){
    int EVs = g_EVs;
    if(EVs < Max_EV_Density ){
        string vtype = "EV2";
        string nodeID = std::to_string(g_addEV_ID++);
        string routeId = std::to_string(RandomNumber(Numm_of_Vehicles_Routes));
        bool success = mobility->getCommandInterface()->addVehicle(nodeID, vtype, routeId, simTime(), 0.0, 0.0,0);
        if (success){g_EVs++;}
    }
}

void DemoBaseApplLayer::Save_SREQ_MSG(int treeID, string Send_Recevied, int hops){

    if(!set_tittle){Save_SREQ_MSG_Tittle();}

    SREQ_MSG.open(SavToFile, ios::out | ios::app );  //para leer datos ios::in

    if (SREQ_MSG.is_open()){
        SREQ_MSG <<Send_Recevied<< " " <<myId << " " << treeID << " " << hops << " " << simTime()<<'\n';
        SREQ_MSG.close();
    }
    else cout << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< '\n';

}

void DemoBaseApplLayer::Save_SREQ_MSG_Tittle(){

    SREQ_MSG.open(SavToFile, ios::out | ios::out );                                    //para leer datos ios::in

    if (SREQ_MSG.is_open()){
        SREQ_MSG << "S/R" << "|" << "MAC_ID" << "|" << "SREQ_MSG_ID" << "|" << "HOPS" << "|" << "Time(sec)"<< "|" << "Creation/Arrival_Time" <<'\n';
        SREQ_MSG.close();
        set_tittle=true;                                                                    //permite inicializar solo una vez el tittle

    }
    else std::cerr << "ERROR NO SE PUEDE ABRIR EL ARCHIVO"<< endl;
}

int DemoBaseApplLayer::RandomNumber(int maxnumber){
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
    std::uniform_real_distribution<> dist(0, maxnumber);
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

///=====================================================================================================================================///

///=====================================================================================================================================///
