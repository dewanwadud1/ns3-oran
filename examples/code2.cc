/*Owner: Abdul Wadud
tCode: 2
Mode: l
Type: extensive mobility + application 
source: https://www.nsnam.org/docs/models/html/lte-user.html


Six new increments from code 1:
1. NetAnim Visualisation
2. Give ues and enbs real location coordinates
3. Add evolved-packet-core and remote host
4. Enable traces and KPIs with flow Monitor
5. Play with Channel and Scheduler
6. Perform handover experiments
*/

#include<fstream>
#include<iostream>
#include<string>
#include<stdio.h>

/*ns3 modules*/
#include<ns3/core-module.h>
#include<ns3/internet-module.h>
#include<ns3/network-module.h>
#include<ns3/applications-module.h>
#include<ns3/mobility-module.h>
#include<ns3/point-to-point-module.h>

/*Lte Module*/
#include<ns3/lte-module.h>

/*oran module*/
//#include<ns3/oran-module.h>

/*Flow Monitor*/
#include <ns3/flow-monitor-helper.h>

/*NetAnim Module*/
#include <ns3/netanim-module.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TestCodeTwo");


void PrintFlowStats (Ptr<FlowMonitor> monitor)
{
  monitor->CheckForLostPackets ();
  auto stats = monitor->GetFlowStats ();
  for (auto &s : stats)
  {
    double thr = s.second.rxBytes * 8.0 /
                 (s.second.timeLastRxPacket.GetSeconds () -
                  s.second.timeFirstTxPacket.GetSeconds ()) / 1e6;
    std::cout << "Flow " << s.first
              << " throughput=" << thr << " Mbps\n";
  }
  Simulator::Schedule (Seconds (1), &PrintFlowStats, monitor);
}

int main(int argc, char *argv[])
{
    uint32_t nEnbs = 2; //number of enbs
    uint32_t nUes = 2; //number of ues


    //  Choose via command-line: --scheduler=PF or RR  --loss=Friis or ITU
    std::string scheduler = "PF";
    std::string loss      = "Friis";
    CommandLine cmd;
    cmd.AddValue ("scheduler", "PF | RR | TBFQ …", scheduler);
    cmd.AddValue ("loss",      "Friis | LogDistance | ITU …", loss);
    cmd.Parse (argc, argv);


    // Creating 1 empty enbNode and 2 ueNodes
    NodeContainer enbNodes, ueNodes, remoteHostContainer;
    enbNodes.Create(nEnbs);
    ueNodes.Create(nUes);
    remoteHostContainer.Create(1);

    // EPC Helper
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    // Create a helper object of LteModule to use the properties of it to the ues and enbs
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    // PGW node
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Point to point link between pgw and remote host
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHostContainer.Get(0));

    // Selecting Scheduler and Pathloss Model
    lteHelper->SetSchedulerType (
        "PF" == scheduler  ? "ns3::PfFfMacScheduler"
    : "RR" == scheduler  ? "ns3::RrFfMacScheduler"
                        : "ns3::PfFfMacScheduler");   // default

    /* ----- path-loss model ----- */
    if (loss == "Friis")
    {
        lteHelper->SetPathlossModelType (FriisPropagationLossModel::GetTypeId ());
    }
    else
    {
        lteHelper->SetPathlossModelType (LogDistancePropagationLossModel::GetTypeId ());
    }

    // Install internet stack
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(ueNodes);

    // Assign IP address
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    // Set up routing on remote host
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting(remoteHostContainer.Get(0)->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // give each UE a default route
    //Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
    Ptr<Ipv4StaticRouting> s = ipv4RoutingHelper.GetStaticRouting (
                                ueNodes.Get(u)->GetObject<Ipv4>());
    s->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }


    /*Milestone 2 starts here*/
    // Configure and install mobility to each of the nodes
    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                    "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0),
                                    "DeltaX", DoubleValue(100.0), "DeltaY", DoubleValue(0.0),
                                    "GridWidth", UintegerValue(1), "LayoutType", StringValue("RowFirst"));
    enbMobility.Install(enbNodes);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel"); // or ConstantVelocity for movement
    ueMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                    "MinX", DoubleValue(50.0), "MinY", DoubleValue(0.0),
                                    "DeltaX", DoubleValue(100.0), "DeltaY", DoubleValue(0.0),
                                    "GridWidth", UintegerValue(1), "LayoutType", StringValue("RowFirst"));
    ueMobility.Install(ueNodes);
    /*Milestone 2 starts here*/


    /*Milestone 1 stats here*/

    // Export the mobility and packet metadata to netAnim
    AnimationInterface anim("code1-anim.xml");
    anim.UpdateNodeDescription(enbNodes.Get(0), "eNB");
    anim.UpdateNodeColor(enbNodes.Get(0), 0, 255, 0); 
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
        std::ostringstream oss;
        oss << "UE" << i;
        anim.UpdateNodeDescription(ueNodes.Get(i), oss.str());
        anim.UpdateNodeColor(ueNodes.Get(i), 255, 0, 0); // red
    }
    anim.UpdateNodeDescription(pgw, "PGW");
    anim.UpdateNodeColor(pgw, 0, 0, 255); // blue

    anim.SetConstantPosition (epcHelper->GetSgwNode (), 110, 30);

    anim.UpdateNodeDescription(remoteHostContainer.Get(0), "RemoteHost");
    anim.UpdateNodeColor(remoteHostContainer.Get(0), 255, 0, 255); // magenta

    anim.EnablePacketMetadata(true); // Ensures NetAnim renders packets

     /*Milestone 1 ends here*/

    // Installing lte protocol stacks on enbs
    NetDeviceContainer enbDevs;
    enbDevs = lteHelper->InstallEnbDevice(enbNodes);

    //Similarly, installing lte protocol stacks on ues
    NetDeviceContainer ueDevs;
    ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // intall ip addresses to the ues
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    // Attach the Ues to the enbs, only one enb so the index is 0 for that one
    lteHelper->AttachToClosestEnb (ueDevs, enbDevs);

    //Establish connections between enbnodes with X2 interface
    lteHelper->AddX2Interface (enbNodes);

    //Activate a data radio bearer between each UE and the enb it is attached to
    // switch to the EPC-safe bearer call *or* rely on the implicit default bearer
    Ptr<EpcTft> tft = EpcTft::Default ();
    for (uint32_t u = 0; u < ueDevs.GetN (); ++u)
    {
    lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get(u),
                                            EpsBearer(EpsBearer::GBR_CONV_VOICE),
                                            tft);
    }

    // ---- KPIs: throughput, delay, jitter, loss ----
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> flowMon = fmHelper.InstallAll ();
    flowMon->SerializeToXmlFile ("flowmon.xml", false, false);  // snapshot at end

    Simulator::Schedule (Seconds (1), &PrintFlowStats, flowMon);

    // Set the simulator stop time, otherwise it will run forever
    Simulator::Stop(Seconds(30));

    // Enabling trace in different stacks
    lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnablePdcpTraces();


    // Run the simulator
    Simulator::Run();

    // Cleanup and exit the simulator
    Simulator::Destroy();
    return 0;
}
 