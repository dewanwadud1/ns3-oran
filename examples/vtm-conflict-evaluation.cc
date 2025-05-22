/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * NIST-developed software is provided by NIST as a public service. You may
 * use, copy and distribute copies of the software in any medium, provided that
 * you keep intact this entire notice. You may improve, modify and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software.
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO
 * WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY OPERATION OF
 * LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST
 * DOES NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE
 * SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
 * CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 *
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This
 * software is not intended to be used in any situation where a failure could
 * cause risk of injury or damage to property. The software developed by NIST
 * employees is not subject to copyright protection within the United States.
 */

#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/lte-module.h>
#include <ns3/mobility-module.h>
#include <ns3/network-module.h>
#include <ns3/oran-module.h>
#include <ns3/config-store.h>
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/energy-module.h"
//#include "ns3/device-energy-model.h"

// #include <ns3/test.h>

//#include "oran-reporter-lte-energy-efficiency.h"

#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NewOranHandoverUsingRSRPlm");

/**
 * Example of the ORAN models.
 *
 * The scenario consists of an LTE UE moving back and forth
 * between 2 LTE eNBs. The LTE UE reports to the RIC its location
 * and current Cell ID. In the RIC, an LM will periodically check
 * the RSRP and RSRQ of UE, and if needed, issue a handover command.
 *
 * This example demonstrates how to configure processing delays for the LMs.
 */



// Global counters
static uint32_t g_successfulHandover = 0;
static uint32_t g_unsuccessfulHandover = 0;
static uint64_t g_totalBytesReceived = 0;


static std::string s_trafficTraceFile = "traffic-trace.tr";
static std::string s_positionTraceFile = "position-trace.tr";
static std::string s_handoverTraceFile = "handover-trace.tr";
static std::string s_rsrpSinrTraceFile = "rsrp-sinr-trace.tr";
static std::string s_throughputTraceFile = "throughput-trace.tr";
static std::string s_energyTraceFile = "energy-trace.tr";
static std::string s_metricsTraceFile = "metrics-trace.tr";


// Callback for a successful handover event
void HandoverSuccessCallback(uint64_t imsi, uint16_t cellId, uint16_t rnti) {
    g_successfulHandover++;
    std::cout << Simulator::Now().As(Time::S) << " Successful handover: IMSI " << imsi 
              << " to cell " << cellId << " (RNTI " << rnti << ")" << std::endl;
}

// (Optional) Callback for handover attempts or failures
// You may need to add a trace (if available) for unsuccessful attempts.
// For example:
void HandoverAttemptCallback(uint64_t imsi, uint16_t srcCellId, uint16_t targetCellId, uint16_t rnti) {
    // For instance, count every attempt that is not followed by a success.
    g_unsuccessfulHandover++;
}

void NotifyHandoverFailure(std::string context, uint64_t imsi, uint16_t rnti, uint16_t targetCellId)
{
  // Increment a global counter (or log to a file) for any handover failure.
  g_unsuccessfulHandover++;
  std::ofstream out(s_handoverTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\t" << context
      << "\tIMSI:" << imsi << "\tRNTI:" << rnti 
      << "\tTargetCell:" << targetCellId << std::endl;
  out.close();
}


void RxTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom (to).GetPort () / 1000);
    std::ofstream out (s_trafficTraceFile, std::ios_base::app);
    out << Simulator::Now().GetSeconds() << "\tUE " << ueId << "\tRX " << p->GetSize() << std::endl;
    g_totalBytesReceived += p->GetSize();
}

void TxTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom (to).GetPort () / 1000);
    std::ofstream out (s_trafficTraceFile, std::ios_base::app);
    out << Simulator::Now().GetSeconds() << "\tUE " << ueId << "\tTX " << p->GetSize() << std::endl;
}

void ThroughputTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
    static std::ofstream out (s_throughputTraceFile, std::ios_base::app);
    out << Simulator::Now().GetSeconds() << "\t" << p->GetSize() << std::endl;
}


// Tracing rsrp, rsrq, and sinr
void LogRsrpRsrqSinr(Ptr<OutputStreamWrapper> stream, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, uint8_t sinr) {
    *stream->GetStream() << Simulator::Now().GetSeconds() << "\tRNTI: " << rnti
                         << "\tCell ID: " << cellId
                         << "\tRSRP: " << rsrp << " dBm"
                         << "\tRSRQ: " << rsrq << " dB"
                         << "\tSINR: " << static_cast<int>(sinr) << " dB" << std::endl;
}
 
// Callback function to log positions
void LogPosition(Ptr<OutputStreamWrapper> stream, Ptr<Node> node,  Ptr<const MobilityModel> mobility) {
    Vector pos = mobility->GetPosition();
    *stream->GetStream() << Simulator::Now().GetSeconds() << "\t" << node->GetId()
                         << "\t" << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}

void
NotifyHandoverEndOkEnb(uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    g_successfulHandover++;
    std::ofstream out(s_handoverTraceFile, std::ios_base::app);
    out << Simulator::Now().As(Time::S) << " eNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
    out.close();
}

void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(mobility->GetVelocity().x * -1, 0, 0));
    }

    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";

    if (!args.empty())
    {
        std::cout << " (" << args << ")";
    }
    std::cout << std::endl;
}

double CalculateRUCurrent(double txPowerDbm,
                          double pFixedW = 80.0,         // P_0
                          double etaPA = 0.3,            // PA efficiency
                          double deltaAf = 0.0,          // Antenna feeder loss (e.g., 0.5 for 3dB)
                          double deltaDC = 0.07,         // DC-DC loss
                          double deltaMS = 0.09,         // Mains supply loss
                          double deltaCool = 0.10,       // Cooling loss (macro only)
                          uint32_t nTrx = 1,
                          double voltage = 48.0)
{
    if (txPowerDbm <= 0.0) {
        // Sleep mode: Only minimal standby power
        double pSleepW = 5.0;  // Example per TRX sleep value
        return (nTrx * pSleepW) / voltage;
    }

    // Convert dBm to W
    double pTxW = pow(10.0, (txPowerDbm - 30) / 10.0);

    // Account for antenna feeder loss if any
    double paPowerW = pTxW / (etaPA * (1.0 - deltaAf));

    // Add fixed component power (RF, BB, mmWave)
    double pTotalW = paPowerW + pFixedW;

    // Apply power supply inefficiencies (EARTH-style)
    pTotalW = nTrx * pTotalW /
              ((1.0 - deltaDC) * (1.0 - deltaMS) * (1.0 - deltaCool));

    return pTotalW / voltage;
}

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    double txPower = 30.0; // default TxPower in dBm
    cmd.AddValue("txPower", "eNB TxPower in dBm", txPower);
    
    uint16_t numberOfUes = 4;
    uint16_t numberOfEnbs = 2;
    Time simTime = Seconds(30);
    Time maxWaitTime = Seconds(0.010); 
    std::string processingDelayRv = "ns3::NormalRandomVariable[Mean=0.005|Variance=0.000031]";
    double distance = 50; // distance between eNBs
    Time interval = Seconds(15);
    double speed = 1.5; // speed of the ue
    bool dbLog = false;
    Time lmQueryInterval = Seconds(5);
    std::string dbFileName = "oran-repository.db";
    std::string lateCommandPolicy = "DROP";
    
    
    // Energy Harvester variables
    double harvestingUpdateInterval = 1;  // seconds

    // Command line arguments
    //CommandLine cmd(__FILE__);
    cmd.AddValue("db-log", "Enable printing SQL queries results", dbLog);
    cmd.AddValue("max-wait-time", "The maximum amount of time an LM has to run", maxWaitTime);
    cmd.AddValue("processing-delay-rv",
                 "The random variable that represents the LMs processing delay",
                 processingDelayRv);
    cmd.AddValue("lm-query-interval",
                 "The interval at which to query the LM for commands",
                 lmQueryInterval);
    cmd.AddValue("late-command-policy",
                 "The policy to use for handling commands received after the maximum wait time "
                 "(\"DROP\" or \"SAVE\")",
                 lateCommandPolicy);
    cmd.AddValue("sim-time", "The amount of time to simulate", simTime);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    /*--- lte and epc helper ---*/
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>(); // create lteHelper
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>(); // create epcHelper
    lteHelper->SetEpcHelper(epcHelper); // connect lte to the evolved packet core, which is the core network
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler"); // Round-robin Frequency-first Mac Scheduler for resource distribution
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm"); // disable automatic handover

    // Getting the PGW node; it acts as a gateway between LTE and external network, such as- internet.
    Ptr<Node> pgw = epcHelper->GetPgwNode(); // PGW: Packet Data Network Gateway

    
    /*---- Creating RAN nodes using NodeContainer ----*/
    NodeContainer ueNodes; 
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);
    

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
        positionAlloc->Add(Vector(distance * i, 0, 20));
    }

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        // Coordinates of the middle point between the eNBs, minus the distance covered
        // in half of the interval for switching directions
        positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)), 0, 1.5));
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(speed, 0, 0));
    }


    // Schedule the first direction switch
    Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

    // Install LTE Devices in eNB and UEs
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);
    
    // Setting TxPower to 30dBm for all eNbs 
    // Assuming enbLteDevs is your NetDeviceContainer for eNodeBs
    for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it) {
        Ptr<NetDevice> device = *it;
        Ptr<LteEnbNetDevice> enbLteDevice = device->GetObject<LteEnbNetDevice>();
        if (enbLteDevice) {
            Ptr<LteEnbPhy> enbPhy = enbLteDevice->GetPhy();
            enbPhy->SetTxPower(txPower); // Set the transmission power to 30 dBm if 
        }
    }
    
    /** Energy Model **/
    /***************************************************************************/
    // Create Energy Source for eNB nodes
    /*
    BasicEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000)); // 10kJ initial energy

    EnergySourceContainer enbEnergySources = energySourceHelper.Install(enbNodes);

    // Create Device Energy Model (LTE Radio Energy Model)
    WifiRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(1.2));  // Current during transmission (Amperes)
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.8));  // Current during reception (Amperes)

    DeviceEnergyModelContainer enbEnergyModels = radioEnergyHelper.Install(enbLteDevs, enbEnergySources);
    */
     
    // Energy model setup
        BasicEnergySourceHelper energySourceHelper;
        energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100000));

        EnergySourceContainer enbEnergySources = energySourceHelper.Install(enbNodes);

        DeviceEnergyModelContainer enbEnergyModels;
        
        double currentA = CalculateRUCurrent(txPower, 80.0, 0.3, 0.5, 0.07, 0.09, 0.10, 64, 48.0);

        // double currentA = CalculateCurrent(txPower); // txPower is your existing parameter in dBm
        for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i) {
            Ptr<NetDevice> device = enbLteDevs.Get(i);
            Ptr<Node> node = device->GetNode();

            Ptr<EnergySource> source = enbEnergySources.Get(i);
            Ptr<SimpleDeviceEnergyModel> energyModel = CreateObject<SimpleDeviceEnergyModel>();
            energyModel->SetEnergySource(source);
            energyModel->SetNode(node);
            energyModel->SetCurrentA(currentA); 
            source->AppendDeviceEnergyModel(energyModel);
            enbEnergyModels.Add(energyModel);
        }
    
    /* energy harvester */
    BasicEnergyHarvesterHelper basicHarvesterHelper;
    // configure energy harvester
    basicHarvesterHelper.Set ("PeriodicHarvestedPowerUpdateInterval", TimeValue (Seconds (harvestingUpdateInterval)));
    basicHarvesterHelper.Set ("HarvestablePower", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=0.1]"));
    // install harvester on all energy sources
    EnergyHarvesterContainer harvesters = basicHarvesterHelper.Install (enbEnergySources);
    /***************************************************************************/

    // Install the IP stack on the UEs
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));


    // Attach all UEs to the first eNodeB
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));
    }

    // Add X2 interface
    lteHelper->AddX2Interface(enbNodes);
    
        // --- Traffic Application Setup ---
        // ***** Begin Traffic Application Setup *****
        // Create a remote host to generate traffic and connect it to the EPC
        NodeContainer remoteHostContainer;
        remoteHostContainer.Create(1);
        Ptr<Node> remoteHost = remoteHostContainer.Get(0);
        InternetStackHelper internetRemote;
        internetRemote.Install(remoteHostContainer);

        // Connect the remote host to the PGW via a point-to-point link:
        PointToPointHelper p2p;
        p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        p2p.SetDeviceAttribute("Mtu", UintegerValue(65000));
        p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
        NetDeviceContainer remoteDevices = p2p.Install(pgw, remoteHost);

        Ipv4AddressHelper ipv4Remote;
        ipv4Remote.SetBase("1.1.0.0", "255.255.255.0");
        Ipv4InterfaceContainer remoteIfaces = ipv4Remote.Assign(remoteDevices);

        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

        // Install traffic applications on the UEs and the remote host:
        // We'll install a PacketSink on each UE to receive traffic and an OnOffApplication
        // on the remote host to generate traffic.
        ApplicationContainer ueApps;
        ApplicationContainer remoteApps;
        uint16_t basePort = 1000;
        Ptr<UniformRandomVariable> onTimeRv = CreateObject<UniformRandomVariable>();
        onTimeRv->SetAttribute("Min", DoubleValue(1.0));
        onTimeRv->SetAttribute("Max", DoubleValue(5.0));
        Ptr<UniformRandomVariable> offTimeRv = CreateObject<UniformRandomVariable>();
        offTimeRv->SetAttribute("Min", DoubleValue(1.0));
        offTimeRv->SetAttribute("Max", DoubleValue(5.0));

        for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
            uint16_t port = basePort * (i + 1);
            
            // Install a PacketSink on each UE to receive traffic
            PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(Ipv4Address::GetAny(), port));
            ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(i));
            sinkApp.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
            ueApps.Add(sinkApp);
            sinkApp.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&ThroughputTrace));

            // Create an OnOffApplication on the remote host to send traffic to this UE.
            // (Since you have only one UE, this loop will iterate once.)
            Ptr<OnOffApplication> onOffApp = CreateObject<OnOffApplication>();
            remoteApps.Add(onOffApp);
            onOffApp->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIfaces.GetAddress(i), port)));
            // Since you have one UE, we use the eMBB profile:
            onOffApp->SetAttribute("DataRate", DataRateValue(DataRate("10Mbps")));
            onOffApp->SetAttribute("PacketSize", UintegerValue(1500));
            onOffApp->SetAttribute("OnTime", PointerValue(onTimeRv));
            onOffApp->SetAttribute("OffTime", PointerValue(offTimeRv));
            remoteHost->AddApplication(onOffApp);

            // Connect TX trace callbacks so that both traffic events and throughput are recorded.
            onOffApp->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
            // onOffApp->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&ThroughputTrace));
        }

        remoteApps.Start(Seconds(2));
        remoteApps.Stop(simTime + Seconds(10));
        ueApps.Start(Seconds(1));
        ueApps.Stop(simTime + Seconds(15));

        // ***** End Traffic Application Setup *****




    // ORAN Models -- BEGIN
    Ptr<OranNearRtRic> nearRtRic = nullptr;
    OranE2NodeTerminatorContainer e2NodeTerminatorsEnbs;
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();

    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",
                             TimeValue(maxWaitTime)); // 0 means wait for all LMs to finish
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue(lateCommandPolicy));

    // RIC setup
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover",
                                      "ProcessingDelayRv",
                                      StringValue(processingDelayRv));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
    
    // --- also run our new energy‐saving LM
    oranHelper->AddLogicModule("ns3::OranLmLte2LteEnergySaving",
                               "TargetEfficiency", DoubleValue(1e3),  // adjust as you like
                               "StepSize",       DoubleValue(1.0));

    nearRtRic = oranHelper->CreateNearRtRic();
    
    std::vector<Ptr<OranReporterLteEnergyEfficiency>> eeReporters;
    eeReporters.reserve(numberOfUes);

    // UE Nodes setup
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
            CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter = 
        CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranReporterLteEnergyEfficiency> eeReporter = 
        CreateObject<OranReporterLteEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator =
            CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
        
        eeReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

        for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
            Ptr<LteUeNetDevice> lteUeDevice = ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
            if (lteUeDevice)
            {
                Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
                uePhy->TraceConnectWithoutContext("ReportUeMeasurements", MakeCallback(&ns3::OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
            }
        }

        lteUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        lteUeTerminator->SetAttribute("RegistrationIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        lteUeTerminator->SetAttribute("SendIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        lteUeTerminator->AddReporter(locationReporter);
        lteUeTerminator->AddReporter(lteUeCellInfoReporter);
        lteUeTerminator->AddReporter(rsrpRsrqReporter);
        lteUeTerminator->AddReporter(eeReporter);
        
        eeReporters.push_back(eeReporter);

        lteUeTerminator->Attach(ueNodes.Get(idx));

        Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
    }
    
    // ENb Nodes setup
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteEnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));

    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    e2NodeTerminatorsEnbs.Add(oranHelper->DeployTerminators(nearRtRic, enbNodes));

    // DB logging to the terminal
    if (dbLog)
    {
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    // Activate and the components
    Simulator::Schedule(Seconds(1),
                        &OranHelper::ActivateAndStartNearRtRic,
                        oranHelper,
                        nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsEnbs);
    Simulator::Schedule(Seconds(2),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsUes);
    // ORAN Models -- END

    // Trace the end of handovers
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                                  MakeCallback(&NotifyHandoverEndOkEnb));
                                  
    // For tracing unsuccessful handovers
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
                MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
                MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
                MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
                MakeCallback(&NotifyHandoverFailure));

    // Assuming 'ueNodes' and 'enbNodes' are your NodeContainers
    Ptr<OutputStreamWrapper> mobilityTrace = Create<OutputStreamWrapper>("MobilityTrace.tr", std::ios::out);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, ueNodes.Get(i)));
    }
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = enbNodes.Get(i)->GetObject<MobilityModel>();
        mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, ueNodes.Get(i)));
    }
     
    // Tracing rsrp, rsrq, and sinr while setting up uePhy
    Ptr<OutputStreamWrapper> rsrpSinrTrace = Create<OutputStreamWrapper>("RsrpRsrqSinrTrace.tr", std::ios::out);
    for (NetDeviceContainer::Iterator it = ueLteDevs.Begin(); it != ueLteDevs.End(); ++it)
    {
        Ptr<NetDevice> device = *it;
        Ptr<LteUeNetDevice> lteUeDevice = device->GetObject<LteUeNetDevice>();
        if (lteUeDevice)
        {
            Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
            uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeBoundCallback(&LogRsrpRsrqSinr, rsrpSinrTrace));
        }
    }
    
        
    /* Enabling Tracing for the simulation scenario */
    lteHelper->EnablePhyTraces();
    lteHelper->EnableMacTraces();
    lteHelper->EnableRlcTraces();
    lteHelper->EnablePdcpTraces();
    

    Simulator::Stop(simTime);
    
    
    // turn on INFO‐level logging for our energy‐saving LM
    LogComponentEnable("OranLmLte2LteEnergySaving", LOG_LEVEL_INFO);

    // if you want timestamps on *all* log lines globally, do this once:
    // LogComponentEnable("Core", LOG_PREFIX_TIME);
    
    
    Simulator::Run();
    
    
          
    // Calculate Total Energy Consumption
    
    double totalEnergyConsumed = 0;
    for (uint32_t i = 0; i < enbEnergySources.GetN(); i++) {
        Ptr<BasicEnergySource> basicSource = DynamicCast<BasicEnergySource>(enbEnergySources.Get(i));
        double consumed = 100000 - basicSource->GetRemainingEnergy(); // initial - remaining
        totalEnergyConsumed += consumed;
    }

    // Calculate EE KPI (bits per Joule)
    double energyEfficiency = (g_totalBytesReceived * 8) / totalEnergyConsumed;
    
    double throughputMbps = (g_totalBytesReceived * 8.0) / simTime.GetSeconds() / 1e6;
    std::cout << "RESULT: " << txPower << "," << throughputMbps << ","
          << g_successfulHandover << "," << g_unsuccessfulHandover << "," << totalEnergyConsumed << "," << energyEfficiency << std::endl;

    //std::cout << "Total Energy Consumed (Joules): " << totalEnergyConsumed << std::endl;
    //std::cout << "Energy Efficiency (bits/Joule): " << energyEfficiency << std::endl; 
    
    for (auto& rep : eeReporters) {
      rep->ReportEnergyEfficiency(energyEfficiency);
    }
    
    
    Simulator::Destroy();
    
    return 0;
}
