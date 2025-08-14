/*Owner: Abdul Wadud
tCode: 1
Mode: l
Tyoe: casual mobility + application 
source: https://www.nsnam.org/docs/models/html/lte-user.html
*/

#include<fstream>
#include<iostream>
#include<string>
#include<stdio.h>

/*ns3 modules*/
#include<ns3/core-module.h>
//#include<ns3/internet-module.h>
#include<ns3/network-module.h>
//#include<ns3/application-module.h>
#include<ns3/mobility-module.h>
//#include<ns3/point-to-point-module.h>

/*Lte Module*/
#include<ns3/lte-module.h>

/*oran module*/
//#include<ns3/oran-module.h>

/*NetAnim Module*/
#include <ns3/netanim-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TestCodeZero");

int main(int argc, char *argv[])
{
    int nEnbs = 1; //number of enbs
    int nUes = 2; //number of ues

    // Creating 1 empty enbNode and 2 ueNodes
    NodeContainer enbNodes;
    enbNodes.Create(nEnbs);
    NodeContainer ueNodes;
    ueNodes.Create(nUes);

    // Configure and install mobility to each of the nodes
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(enbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    // Create a helper object of LteModule to use the properties of it to the ues and enbs
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

    // Installing lte protocol stacks on enbs
    NetDeviceContainer enbDevs;
    enbDevs = lteHelper->InstallEnbDevice(enbNodes);

    //Similarly, installing lte protocol stacks on ues
    NetDeviceContainer ueDevs;
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // Attach the Ues to the enbs, only one enb so the index is 0 for that one
    lteHelper->Attach(ueDevs, enbDevs.Get(0));

    //Activate a data radio bearer between each UE and the enb it is attached to
    enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
    EpsBearer bearer(q);
    lteHelper->ActivateDataRadioBearer(ueDevs, bearer);

    // Set the simulator stop time, otherwise it will run forever
    Simulator::Stop(Seconds(2));

    // Run the simulator
    Simulator::Run();

    // Cleanup and exit the simulator
    Simulator::Destroy();
    return 0;
}
 