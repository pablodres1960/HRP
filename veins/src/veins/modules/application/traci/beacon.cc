#include "veins/modules/application/traci/beacon.h"
//=======================================================================================
//=======    TABLA DE BEACONS CREADA POR C/ IZA MODIFICADA POR PABLO ====================
//=======================================================================================
using namespace std;


BeaconList::BeaconList(){
    head=NULL;
    curr=NULL;
    temp=NULL;
}

//=======================================================================================
//ADD beacon to NNT
//=======================================================================================

void BeaconList::AddBeacon(std::string b_typeNode, int b_idMsg, int b_idVehicle, double b_time, Coord b_SenderCoord, double b_dsr, double b_dsd,double b_drd, double b_Nv, double b_wdist,double b_abe,double b_GS,LAddress::L2Type b_NH_Address,double b_NH_GS,double NH_Dst_to_Dest,double Local_N_GS){
    beaconPtr n = new beacon; //new means dynamic memory reservation
    n->next = NULL;
    n->typeNode = b_typeNode;
    n->idMsg = b_idMsg;
    n->idVehicle = b_idVehicle;
    n->time = b_time;
    n->SenderCoord = b_SenderCoord;
    n->dsr = b_dsr;
    n->dsd = b_dsd;
    n->drd = b_drd;
    n->Nv = b_Nv;
    n->wdist = b_wdist;
    n->abe = b_abe;
    n->GS = b_GS;
    n->NH_Address = b_NH_Address;
    n->NH_GS = b_NH_GS;
    n->NH_Dst_to_Dest = NH_Dst_to_Dest;
    n->Local_N_GS = Local_N_GS;

    if(head != NULL){
        curr = head;
        while(curr->next != NULL){curr = curr->next;}
        curr->next=n;
    }
    else{head=n;}
}

//=======================================================================================
//DELETE beacon nodeID
//=======================================================================================

void BeaconList::DeleteBeacon(int delb_idVehicle){
    beaconPtr delPtr = NULL;
    temp = head;
    curr = head;

   while(curr !=NULL && curr->idVehicle != delb_idVehicle){
        temp = curr;
        curr = curr->next;

    }

    if(curr == NULL){delete delPtr;}
    else{
        delPtr = curr;
        curr = curr->next;
        temp->next = curr;
        if(delPtr==head){
            head= head->next;
            temp = NULL;
        }
        delete delPtr;
    }
}

//=======================================================================================
//PRINT NNT de cada nodo
//=======================================================================================

void BeaconList::PrintBeacons(int CurrNodeAddress, string IN_OUT){
    curr = head;
    EV<<"------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
    EV<<IN_OUT<<" -> CurrNodeAddress = "<<CurrNodeAddress<<endl;
    EV<<"------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
    EV<<"TreeMsg_ID"<<setw(15)<<"MAC_Address"<<setw(15)<<"NH_Adress"<<setw(10)<<"NH_GS"<<setw(20)<<"NH_Dst_to_Dest"<<setw(12)<<"n_S"<<setw(15)<<"Local_S"<<setw(15)<<"NodeType"<<setw(15)<<"dsr"<<setw(20)<<"|D-ToRSU|dsd"<<setw(15)<<"drd"<<setw(16)<<"Density"<<setw(15)<<"wdist"<<setw(15)<<"ABE"<<setw(15)<<"ArrivalTime"<<endl;
    while(curr != NULL)
    {
        EV<<curr->idMsg<<setw(15)<<curr->idVehicle<<setw(15)<<curr->NH_Address<<setw(15)<<curr->NH_GS<<setw(15)<<curr->NH_Dst_to_Dest<<setw(15)<<curr->GS<<setw(15)<<curr->Local_N_GS<<setw(15)<<curr->typeNode<<setw(15)<<curr->dsr<<setw(15)<<curr->dsd<<setw(20)<<curr->drd<<setw(15)<<curr->Nv<<setw(15)<<curr->wdist<<setw(15)<<curr->abe<<setw(15)<<curr->time<<endl;
        curr = curr->next;
    }
    EV<<"------------------------------------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
}

//=======================================================================================
//RETURN mac address at the top of NNT para luego ser configurada como receiver address ***NODO SELECCIONADO PARA EL REENVIO ***
//=======================================================================================

int BeaconList::TopTable(){
    int nodeid=0;
    curr = head;

    if (curr != NULL){nodeid = curr->idVehicle;}

    return nodeid;
}

//=======================================================================================
//RETURN densidad del top table vehicle
//=======================================================================================

double BeaconList::TopTableNv(){
    double nodedensity=0;
    curr = head;

    if (curr != NULL){nodedensity=curr->Nv;}

    return nodedensity;
}

//=======================================================================================
//RETURN densidad del top table vehicle
//=======================================================================================

Coord BeaconList::TopTableNghCoord(){
    Coord NghCoord;
    curr = head;

    if (curr != NULL){NghCoord=curr->SenderCoord;}

    return NghCoord;
}

//=======================================================================================
//RETURN top table U_DIST
//=======================================================================================

double BeaconList::TopTable_U_dist(){
    double U_dist=0;
    curr = head;

    if (curr != NULL){U_dist=curr->wdist;}

    return U_dist;
}

//=======================================================================================
//RETURN DSD value (distodestination -> distancia desde el que envia el beacon a la posicion del RSU.) at the top of the list
//=======================================================================================

double BeaconList::TopTabledisToDestination(){ ///
    double dsd=0;
    curr = head;

    if (curr != NULL){dsd=curr->dsd;}

    return dsd;
}

//=======================================================================================
//RETURN DSR  del primer nodo de la NNT (Distancia del nodo que recibe el beacon al nodo que origina (no el nodo que reenvia) el beacon)
//=======================================================================================

double BeaconList::topTableDtoS(){
    double dsr=0;
    curr = head;

    if (curr != NULL){dsr=curr->dsr;}

    return dsr;
}

//=======================================================================================
//RETURN cuenta del numbero de vecinos -> se usa para la densidad
//=======================================================================================

int BeaconList::CounterBeacons(){
    curr = head;
    int counter = 0;

    while(curr != NULL)
    {
        curr = curr->next;
        counter++;
    }
    return counter;
}

//=======================================================================================
//RETURN true if the nodeID es encontrado en la NNT
//=======================================================================================

bool BeaconList::SearchBeaconNodeID(int NodeID){
    bool foundBeacon=false;
  //  beaconPtr delPtr = NULL;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->idVehicle != NodeID){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){foundBeacon = true;}
  //  delete delPtr;
    return foundBeacon;
}

//=======================================================================================
//RETURN true if the nodeID es encontrado en la NNT and is an RSU
//=======================================================================================

bool BeaconList::SearchRSUid(int RSU_ID){
    bool foundBeacon=false;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->idVehicle != RSU_ID && curr->typeNode != "rsu"){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){foundBeacon = true;}
    return foundBeacon;
}

//=======================================================================================
//RETURN true if the treeID es encontrado en la NNT
//=======================================================================================

bool BeaconList::SearchBeaconTreeID(int treeID){
    bool foundBeacon = false;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->idMsg != treeID ){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){foundBeacon = true;}
    return foundBeacon;
}

//=======================================================================================
//RETURN coord del nodeID enviado como parametro
//=======================================================================================

Coord BeaconList::SearchBeaconCoord(int NodeID){
    Coord NodeCoord;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->idVehicle != NodeID ){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){NodeCoord = curr->SenderCoord;}
    return NodeCoord;
}

//=======================================================================================
//RETURN n entry score -> para actualizar NH fields
//=======================================================================================

double BeaconList::n_S(int nexthopAddress){
    double n_S = 0;
    curr = head;
    while(curr !=NULL && curr->idVehicle != nexthopAddress ){curr = curr->next;}
    if(curr != NULL){n_S = curr->GS;}           ///n_S es el calculado cuando llego el beacon. Luego se escoge este beacon como NH y obtengo sus parametros para los NH fields
    return n_S;
}

//=======================================================================================
//RETURN n entry distance to RSU -> para actualizar NH fields
//=======================================================================================

double BeaconList::n_DsttoRSU(int nexthopAddress){
    double n_dsttoRSU = 0;
    curr = head;
    while(curr !=NULL && curr->idVehicle != nexthopAddress ){curr = curr->next;}
    if(curr != NULL){n_dsttoRSU = curr->dsd;}           ///n_S es el calculado cuando llego el beacon. Luego se escoge este beacon como NH y obtengo sus parametros para los NH fields
    return n_dsttoRSU;
}

//=======================================================================================
//RETURN nodeID del nodo mas cercano al destino del packet. Devueleve 0 si no hay nodos en mi tabla
//=======================================================================================

int BeaconList::SearchClstNODE(Coord DstNodeCoord){
    int ClosestnodeID = 0;
    double mindistance = 4000;
    double currdistance = 0;
    Coord currNeighborNodeCoord;

    curr = head;
    while(curr != NULL) {
        if (curr->typeNode != "rsu"){
            currNeighborNodeCoord = curr->SenderCoord;
            currdistance = currNeighborNodeCoord.distance(DstNodeCoord);
            if(currdistance < mindistance){mindistance = currdistance;ClosestnodeID = curr->idVehicle;}
            curr = curr->next;
        }else{curr = curr->next;}
    }
    return ClosestnodeID;
}

//=======================================================================================
//RETURN RSUid ENCONTRADO en la NNT
//=======================================================================================

int BeaconList::SearchBeaconRSU(){
    int idRSU=0;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->typeNode != "rsu"){
        temp = curr;
        curr = curr->next;
    }

    if(curr == NULL){idRSU = 0;}
    else{idRSU = curr->idVehicle;}
    return idRSU;
}

//=======================================================================================
//RETURN true if RSU is Found
//=======================================================================================

bool BeaconList::Search_RSU_NNT(){

    bool RSU_Found = false;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->typeNode != "rsu"){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){RSU_Found = true;}
    return RSU_Found;
}

//=======================================================================================
//ACTUALIZA los campos del beacon que ya estaba en la tabla de vecinos
//=======================================================================================

void BeaconList::UpdateBeacon(std::string b_typeNode, int b_idMsg, int b_idVehicle, double b_time, Coord b_SenderCoord, double b_dsr, double b_dsd,double b_drd, double b_Nv, double b_wdist, double b_abe,double b_GS,LAddress::L2Type b_NH_Address,double b_NH_GS,double NH_Dst_to_Dest,double Local_N_GS){
    beaconPtr n = new beacon;
    n = head;
    while(n != NULL)
    {
        if(n->idVehicle == b_idVehicle)
        {
            n->typeNode = b_typeNode;
            n->idMsg = b_idMsg;
            n->idVehicle = b_idVehicle;
            n->time = b_time;
            n->SenderCoord = b_SenderCoord;
            n->dsr = b_dsr;
            n->dsd = b_dsd;
            n->drd = b_drd;
            n->Nv = b_Nv;
            n->wdist = b_wdist;
            n->abe = b_abe;
            n->GS = b_GS;
            n->NH_Address = b_NH_Address;
            n->NH_GS = b_NH_GS;
            n->NH_Dst_to_Dest = NH_Dst_to_Dest;
            n->Local_N_GS = Local_N_GS;
            return;
        }
        else
            n = n->next;
    }
}

//=======================================================================================
//Ordena las entradas de la NNT para luego seleccionar el primer vecino como candidato para el reenvio
//=======================================================================================

void BeaconList::SortBeacons(int Num_of_Metrics){//organize
   double GS_temp = 0;//means: global score temporal
   double GS_curr = 0;
   bool changeMade;

   do{
       changeMade = false;
       curr = head;

       while(curr != NULL && curr->next != NULL) {
           temp = curr;
           curr = curr->next;
///============================================================================================///
///                         2 METRICS => DISTANCIA & DENSIDAD                                  ///
///============================================================================================///
           if(Num_of_Metrics == 2){
               GS_temp = ((temp->wdist) + (temp->Nv))*0.5;
               GS_curr = ((curr->wdist) + (curr->Nv))*0.5;
           }
///============================================================================================///
///                 3 METRICS => DISTANCIA & DENSIDAD & BANDWIDTH                              ///
///============================================================================================///
           else if(Num_of_Metrics == 3){
               GS_temp = ((temp->wdist) + (temp->Nv) + (temp->abe/6000000))*0.33;
               GS_curr = ((curr->wdist) + (curr->Nv) + (curr->abe/6000000))*0.33;
           }
           else{std::cerr<<"ERROR -> NUMBERO DE METRICAS INVALIDO"<<endl;}

           if(curr && GS_curr > GS_temp){
                   changeMade = true;
                   swap(temp->typeNode, curr->typeNode);
                   swap(temp->idMsg, curr->idMsg );
                   swap(temp->idVehicle, curr->idVehicle);
                   swap(temp->time, curr->time );
                   swap(temp->SenderCoord, curr->SenderCoord);
                   swap(temp->dsr, curr->dsr);
                   swap(temp->dsd, curr->dsd);
                   swap(temp->drd, curr->drd);
                   swap(temp->Nv, curr->Nv);
                   swap(temp->wdist, curr->wdist);
                   swap(temp->abe, curr->abe);
                   swap(temp->GS,curr->GS);
                   swap(temp->NH_Address,curr->NH_Address);
                   swap(temp->NH_Dst_to_Dest,curr->NH_Dst_to_Dest);
                   swap(temp->NH_GS,curr->NH_GS);
                   swap(temp->Local_N_GS,curr->Local_N_GS);
           }
        }
   } while(changeMade);
}

//=======================================================================================
// 2hGAR/MMMR CONDITIONS
//=======================================================================================

int BeaconList::TwohGAR_Conditions(bool MMMR_Only, int Original_sender, bool TL, vector <int> TabuList, int myID,string MsgType, double N_DstToRSU, bool printDebug, int MsgID){

    int NH_Id = myID;  //temp
    double Last_n_NH_S = -2;            // -1 is the default Score for nodes without neighbors
    double Last_n_S = -2;               // se usa para inicializar MMMR
    beaconPtr Aux = NULL;
    curr = head;

    while(curr != NULL){

        bool Elegible_N = true;
        int n = curr->idVehicle;
        int Condition = 0;
        double n_S = curr->GS;


        while(Elegible_N){
///=====================================================================================///
///                    TABU LIST CONDITION   -> BEACON TL={0,0,0}                       ///
///=====================================================================================///
            if(MsgType != "beacon"){                                                                   ///EL BEACON NO REVISA TL. ARRANCA CON TL={0,0,,0}
                if(TL){                                                                                /// REVISO SI EL n entry en N NNT esta en la TabuList
                    for(int i=0;i<TabuList.size();i++){
                        if(n == TabuList[i]){Condition = 1;break;}
                    }
                }
            }
///=====================================================================================///
///                             MMMR CONDITIONS                                         ///
///=====================================================================================///

            if(MsgType != "beacon" && Original_sender == n){Condition = 6;break;}                                             //ENVIO EL MSG AL MISMO NODO ORIGEN INDICA PROBLEMAS DE LOOPS

            if(MMMR_Only){
                if(Condition == 0){
                    if(curr->Local_N_GS == n_S){
                        if(curr->dsd > N_DstToRSU){Condition = 7;break;}  //test sin esto
                    }else{
                        if(curr->Local_N_GS > n_S){Condition = 8;break;}
                    }
                }
            }

///=====================================================================================///
///                             NH IN NNT CONDITIONS                                    ///
///=====================================================================================///
          if(!MMMR_Only){

              int n_NH = curr->NH_Address;

              if(Condition == 0 && n_NH == myID){Condition = 9;break;}               ///EL NEXT HOP DE n soy yo N. NO es candidato elegible

              if(Condition == 0 && n != n_NH){                                       ///  EXCEPTION ->  n = n_NH -> el next hop es el propio nodo

                  Aux = head;
                  while(Aux != NULL && n_NH != Aux->idVehicle){Aux = Aux->next;}                                   ///Elegible status TRUE
                  if(Aux != NULL){Condition = 3;break;}

              }
          }


///=====================================================================================///
            Elegible_N = false;                                                                              ///termina el while de las condiciones
        }

///=============================================================================================///
///    2hGAR (n_NH_DisttoRSU < N_DisttoRSU) && (2 hops check score -> n_NH_S > Last_n_NH_S )
///=============================================================================================///

        if(Condition == 0){                                                // Condition = 0 means node could be selected as next hop
            if(!MMMR_Only){
                if(curr->NH_Dst_to_Dest < N_DstToRSU){                         // n next hop distance to rsu is lower than N distance (current node) to rsu
                    if(curr->NH_GS > Last_n_NH_S){                             // curr->NH_GS  score of current n entry
                        NH_Id = n;
                        Last_n_NH_S = curr->NH_GS;
                    }
                }
            }else{
                if(n_S > Last_n_S){                             // curr->NH_GS  score of current n entry
                    NH_Id = n;
                    Last_n_S = n_S;
                }
            }
        }

        if(printDebug){
            if(MsgType != "beacon"){
                if(Condition == 0){std::cerr<<myID<<"  -- "<<MsgType<<"  to:"<<n<<"        MsgID:"<<MsgID<<"   E:"<<NH_Id<<"  C:"<<Condition<<"   T:"<<simTime()<<endl;}
                else{std::cerr<<myID<<"  -- "<<MsgType<<"  to:"<<n<<"        MsgID:"<<MsgID<<"   NE:"<<n<<"  C:"<<Condition<<"   T:"<<simTime()<<endl;}
            }
        }

        curr = curr->next;              ///next n in N NNT
    }

    return NH_Id;
}


//=======================================================================================
////BORRA ELEMENTOS OUTDATED DE LA NNT -> SI LA ENTRADA ES MENOR A 2 segundos por default
//=======================================================================================

void BeaconList::PurgeBeacons(double b_ttl){
    curr = head;
    double TimeExpited = simTime().dbl() - b_ttl ;
    // BORRA el primer elemento de la tabla si esta outdated
    if (curr != NULL){
        if (curr->time < TimeExpited){DeleteBeacon(curr->idVehicle);}
    }

    while(curr != NULL && curr->next != NULL){
           if(curr->next->time < TimeExpited){
              temp = curr->next;
              curr->next = temp->next;
              free(temp);
           }else{curr = curr->next;}
       }
}

//=======================================================================================
//RETURN top table GS
//=======================================================================================

double BeaconList::TopTableNH_GS(){
    double T_GS=0;
    curr = head;

    if (curr != NULL){T_GS=curr->NH_GS;}

    return T_GS;
}


//=======================================================================================
//RETURN NH of NodeID
//=======================================================================================

LAddress::L2Type BeaconList::NH_GS(int nodeID){
    LAddress::L2Type NHofNodeID = 0;
    temp = head;
    curr = head;

    while(curr !=NULL && curr->idVehicle != nodeID){
        temp = curr;
        curr = curr->next;
    }

    if(curr != NULL){NHofNodeID = curr->NH_Address;}
    return NHofNodeID;
}
