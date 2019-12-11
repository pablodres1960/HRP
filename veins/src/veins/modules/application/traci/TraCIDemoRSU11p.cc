//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
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

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"

using namespace Veins;

Define_Module(Veins::TraCIDemoRSU11p);

void TraCIDemoRSU11p::onWSA(DemoServiceAdvertisment* wsa)
{
    // if this RSU receives a WSA for service 42, it will tune to the chan
    if (wsa->getPsid() == 42) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
    }
}

void TraCIDemoRSU11p::onWSM(BaseFrame1609_4* frame)
{
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);

    // this rsu repeats the received traffic update in 2 seconds plus some random delay
    sendDelayedDown(wsm->dup(), 2 + uniform(0.01, 0.2));
}

void TraCIDemoRSU11p::handleSelfMsg(cMessage* msg)
{
    if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        // send this message on the service channel until the counter is 3 or higher.
        // this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {
            // stop service advertisements
            stopService();
            delete (wsm);
        }
        else {
            scheduleAt(simTime() + 1, wsm);
        }
    }
    else {
           DemoBaseApplLayer::handleSelfMsg(msg);  //funcion por defecto para el envio de beacons
    }
}

void TraCIDemoRSU11p::onBSM(DemoSafetyMessage* bsm) {   ///SE PROCESA EL MSG RECIVIDO en el RSU

    Read_Beacon_Fields(bsm);
    bool Msg_Processed = ListBeacon.SearchBeaconTreeID(Table_Ngh_Msg_TreeID);                //VERIFICA SI YA HE PROCESADO ESTE BEACON TREEID. EVITA GUARDAR BEACONS FORWARDED

///===================== ACTUALIZA NNT ========================================================================
/*
    if((Table_Ngh_NodeId != myId) && (!Msg_Processed)){                                      //SI YO SOY EL QUE ORIGINO EL BEACON O YA HE RECIBIDO ESTE BEACONS TREEID NO ACTUALIZO MI TABLA
      if(bsm->getHopsCounter() <= 1){UpdateNNT(bsm);}
    }
    */
///=================== RSU SEND SRESP TO NODE  =========================================================================

    if((Node_Type_Destination == "rsu") && (bsm->getSREQ()) && (Msg_RecipienAddress == myId)){
        if (!bsm->getSRESP() && EV_Service_Msg){                        ///SOLO EVS GENERAN MENSAJES Y SE GENERA UNA SRESP del RSU
            //At_RSU_UpdateNextHopFields(bsm);                           //SRESP RSU -> NODE SEND NEW BEACON WITH SRESP=true  FOR cs SELECTION
        }

        if(printDebug){std::cerr<<myId<<"   --    R    --> "<<bsm->getSenderAddress()<<"  B_TreeId:"<<bsm->getBuffer_TreeId()<<"   H:"<<bsm->getHopsCounter()<<"   SA:"<<bsm->getSenderAddress()<<"   T:"<<simTime()<<endl;}

        if(simTime() <= TimeToEndSendingMSGs){                          ///deja de recibir los ultimos mensajes
            Save_SREQ_MSG(bsm->getBuffer_TreeId(),"R",bsm->getHopsCounter());      //GUARDA LOS MSG QUE LLEGAN CON SREQ TRUE
        }
    }
}
