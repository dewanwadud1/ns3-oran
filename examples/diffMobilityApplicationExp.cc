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

 #include <ns3/netanim-module.h>
 
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
 ReverseVelocity (NodeContainer nodes, Time interval)
 {
     for (uint32_t i = 0; i < nodes.GetN (); ++i)
     {
         Ptr<ConstantVelocityMobilityModel> cv =
             nodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
 
         Vector v = cv->GetVelocity ();
         cv->SetVelocity (Vector (-v.x, v.y, v.z));   // flip X direction
     }
     Simulator::Schedule (interval, &ReverseVelocity, nodes, interval);
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

 void MobilityProbe(uint32_t i, Ptr<Node> n){
    auto mm = n->GetObject<MobilityModel>();
    auto cv = n->GetObject<ConstantVelocityMobilityModel>();
    Vector p = mm->GetPosition();
    Vector v = cv ? cv->GetVelocity() : Vector(0,0,0);
    std::cout << Simulator::Now().GetSeconds() << "s UE_" << i
              << " pos=(" << p.x << "," << p.y << ") vel=("
              << v.x << "," << v.y << ")\n";
  }
 
 int
 main(int argc, char* argv[])
 {
     CommandLine cmd(__FILE__);
     double txPower = 30.0; // default TxPower in dBm
     cmd.AddValue("txPower", "eNB TxPower in dBm", txPower);
     
     uint16_t numberOfUes = 50;
     uint16_t numberOfEnbs = 2;
     Time simTime = Seconds(10);
     Time maxWaitTime = Seconds(0.010);
     std::string processingDelayRv = "ns3::NormalRandomVariable[Mean=0.005|Variance=0.000031]";
     double distance = 20; // distance between eNBs
     Time interval = Seconds(20);
     //double speed = 1.5; // speed of the ue
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
     
     NodeContainer vehicularUes; 

     /*Set uplink and downlink RB config*/
     // Set bandwidth before installation
     lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100)); // 100 resource blocks = 20MHz
     lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(100));

 
     // Install Mobility Model
    //  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    //  for (uint16_t i = 0; i < numberOfEnbs; i++)
    //  {
    //      positionAlloc->Add(Vector(distance * i, 0, 20));
    //  }
 
    //  for (uint16_t i = 0; i < numberOfUes; i++)
    //  {
    //      // Coordinates of the middle point between the eNBs, minus the distance covered
    //      // in half of the interval for switching directions
    //      positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)), 0, 1.5));
    //  }
 
    //  MobilityHelper mobility;
    //  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    //  mobility.SetPositionAllocator(positionAlloc);
    //  mobility.Install(enbNodes);
 
    //  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    //  mobility.Install(ueNodes);
 
    //  for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    //  {
    //      Ptr<ConstantVelocityMobilityModel> mobility =
    //          ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
    //      mobility->SetVelocity(Vector(speed, 0, 0));
    //  }


    /* ---------- Mobility Pattern Assignment (robust version) ------------------ */
    /* 1.  Role table ----------------------------------------------------------- */
    struct MobilityProfile
    {
    std::string name;
    double      speed;     // m/s (for CV model)
    double      z;         // antenna height
    };

    static const std::vector<MobilityProfile> kProfiles = {
    { "Pedestrian",  1.5, 1.5 },
    { "Cyclist",     5.5, 1.5 },
    { "Car",        15.0, 1.5 },
    { "Motorbike",  18.0, 1.5 },
    { "Bus",        15.0, 1.5 },
    { "Train",      25.0, 1.5 }
    };

    /* 2.  eNBs are static ------------------------------------------------------ */
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator> ();
    for (uint16_t k = 0; k < numberOfEnbs; ++k)
        enbPos->Add (Vector (distance * k, 0.0, 20.0));

    MobilityHelper mobEnb;
    mobEnb.SetPositionAllocator (enbPos);
    mobEnb.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobEnb.Install (enbNodes);

    /* 3.  UEs (one helper per node) ------------------------------------------- */
    for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
        Ptr<Node> ue = ueNodes.Get (i);
        const MobilityProfile& P = kProfiles[i % kProfiles.size ()];

        /* choose model + initial position */
        std::string model;
        Vector start = Vector ((distance / 2) - 5.0 * i, 0.0, P.z);

        if (P.name == "Pedestrian")
            model = "ns3::RandomWalk2dMobilityModel";
        else
        {
            model = "ns3::ConstantVelocityMobilityModel";
            if (P.name == "Bus")   start = Vector ( 25.0, -10.0, P.z);
            if (P.name == "Train") start = Vector (-10.0,   0.0, P.z);

            vehicularUes.Add (ue); 
        }

        MobilityHelper mh;
        if (model == "ns3::RandomWalk2dMobilityModel")
        {
            Rectangle box (-distance, 2 * distance, -distance, distance);
            mh.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue (box),
                                "Speed",  StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"));
        }
        else
            mh.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");

        mh.Install (ue);

        /* ---------------------------------------------------------------
        * RandomWalk2dMobilityModel checks that the initial position is
        * inside its Bounds rectangle.  Skip the SetPosition() call for
        * those UEs (they will keep the helper-chosen safe position).
        * --------------------------------------------------------------*/
            
        if (model != "ns3::RandomWalk2dMobilityModel")
        {
            if (auto mm = ue->GetObject<MobilityModel> ())
                mm->SetPosition (start);
        }
            
        /* constant-velocity UEs also get a speed vector */
        if (auto cv = ue->GetObject<ConstantVelocityMobilityModel> ())
        {
            cv->SetVelocity (Vector (P.speed, 0.0, 0.0));
        }
    }

    /*All remaining node with constant position Mobility*/
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it) {
        Ptr<Node> n = *it;
        if (!n->GetObject<MobilityModel>()) {
          n->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
        }
      }      

    // /* 4.  Safe trace hooks ----------------------------------------------------- */
    Ptr<OutputStreamWrapper> ueTrace  =
        Create<OutputStreamWrapper> ("MobilityTrace-UE.tr",  std::ios::out);
    Ptr<OutputStreamWrapper> enbTrace =
        Create<OutputStreamWrapper> ("MobilityTrace-eNB.tr", std::ios::out);

    /* hook UEs */
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
        if (mob) {
            mob->TraceConnectWithoutContext(
                "CourseChange",
                MakeBoundCallback(&LogPosition, ueTrace, ueNodes.Get(i)));
        } else {
            NS_LOG_ERROR("Node " << i << " missing MobilityModel—fix your mobility assignment!");
        }
    }

    /*Verify that all ue nodes has a mobility*/
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> mm = ueNodes.Get(i)->GetObject<MobilityModel>();
        if (!mm) {
            NS_LOG_ERROR("MobilityModel missing on UE " << i);
            // Optionally, assign a default model here, or exclude this node from simulation/applications/etc
        }
    }

    /* hook eNBs */
    for (uint32_t i = 0; i < enbNodes.GetN (); ++i)
    {
        auto mm = enbNodes.Get(i)->GetObject<MobilityModel> ();
        if (mm)
            mm->TraceConnectWithoutContext ("CourseChange",
                MakeBoundCallback (&LogPosition, enbTrace, enbNodes.Get(i)));
    }
    /* ---------- End Mobility Pattern Assignment ------------------------------ */


 
 
     // Schedule the first direction switch
     //Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);
     Simulator::Schedule (interval, &ReverseVelocity, vehicularUes, interval);

     // After mobility install:
    Simulator::Schedule(Seconds(0.1), &MobilityProbe, 4, ueNodes.Get(4)); // Bus
    Simulator::Schedule(Seconds(10),  &MobilityProbe, 4, ueNodes.Get(4));
    Simulator::Schedule(Seconds(0.1), &MobilityProbe, 5, ueNodes.Get(5)); // Train
    Simulator::Schedule(Seconds(10),  &MobilityProbe, 5, ueNodes.Get(5));
 
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

    /* Mobility to PGW and Remote Host*/
    // --- Give PGW and remote host a mobility model (to silence NetAnim warnings)
    NodeContainer infra;
    infra.Add (pgw);
    infra.Add (remoteHost);
    MobilityHelper mobInfra;
    mobInfra.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobInfra.Install (infra);

    // Optional: place them somewhere visible in NetAnim
    pgw->GetObject<MobilityModel> ()->SetPosition (Vector (-20.0, 20.0, 0.0));
    remoteHost->GetObject<MobilityModel> ()->SetPosition (Vector (-25.0, 20.0, 0.0));
    
    /* Mobility to PGW and Remote Host*/


     /*netAnim Trace Starts here*/

     // NetAnim: place AFTER all nodes are created and assigned mobility
     AnimationInterface anim ("lte-oran-animation.xml");
     anim.SetMaxPktsPerTraceFile (500000);
     // anim.EnablePacketMetadata (true); // keep disabled unless all tags are registered
     anim.SetMobilityPollInterval (Seconds (0.1));
     anim.EnableIpv4L3ProtocolCounters (Seconds (0), simTime);
     // Enable position updates for mobility
    anim.SetMobilityPollInterval(Seconds(0.1)); // Update positions every 0.1s
 
     // Label & color code nodes
     for (uint32_t i = 0; i < enbNodes.GetN (); ++i) {
         anim.UpdateNodeDescription (enbNodes.Get(i), "eNB_" + std::to_string(i));
         anim.UpdateNodeColor       (enbNodes.Get(i), 0, 255, 0);
         anim.UpdateNodeSize        (enbNodes.Get(i), 10, 10);
     }
     for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
         anim.UpdateNodeDescription (ueNodes.Get(i), "UE_" + std::to_string(i));
         anim.UpdateNodeColor       (ueNodes.Get(i), 0, 0, 255);
         anim.UpdateNodeSize        (ueNodes.Get(i), 5, 5);
     }
     // Optional: label infra nodes too
     anim.UpdateNodeDescription (pgw,        "PGW");
     anim.UpdateNodeColor       (pgw,        255, 165, 0);
     anim.UpdateNodeDescription (remoteHost, "RemoteHost");
     anim.UpdateNodeColor       (remoteHost, 128, 0, 128);
 
 
     /*netAnim Trace Ends here*/


    // Application installation
    ApplicationContainer ueApps;
    ApplicationContainer remoteApps;
    uint16_t basePort = 10000;

    for (uint16_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint16_t port = basePort + i;

        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(i));
        sinkApp.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
        ueApps.Add(sinkApp);
        sinkApp.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&ThroughputTrace));

        // Choose application parameters by UE index
        std::string dataRate;
        uint32_t packetSize;
        double onTime, offTime;

        if (i % 4 == 0) // eMBB: High throughput
        {
            dataRate = "10Mbps";
            packetSize = 1500;
            onTime = 0.5;
            offTime = 0.0;
        }
        else if (i % 4 == 1) // URLLC: Low latency, small packets
        {
            dataRate = "2Mbps";
            packetSize = 64;
            onTime = 0.005;
            offTime = 0.0;
        }
        else if (i % 4 == 2) // V2X: Periodic, moderate size, moderate interval
        {
            dataRate = "1Mbps";
            packetSize = 200;
            onTime = 0.01;
            offTime = 0.0;
        }
        else if (i % 4 == 3) // mMTC: Sparse, very small, low-rate
        {
            dataRate = "10kbps";
            packetSize = 50;
            onTime = 1.0;
            offTime = 4.0;
        }
        else // Default
        {
            dataRate = "10Mbps";
            packetSize = 512;
            onTime = 1.0;
            offTime = 1.0;
        }

        OnOffHelper onOffHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(ueIpIfaces.GetAddress(i), port));
        onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(dataRate)));
        onOffHelper.SetAttribute("PacketSize", UintegerValue(packetSize));
        onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(onTime) + "]"));
        onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(offTime) + "]"));

        ApplicationContainer onOffApp = onOffHelper.Install(remoteHost);
        onOffApp.Get(0)->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
        remoteApps.Add(onOffApp);
    }

    // Application start/stop
    remoteApps.Start(Seconds(0.1));
    remoteApps.Stop(simTime + Seconds(0.1));
    ueApps.Start(Seconds(0.05));
    ueApps.Stop(simTime);

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
     
     //std::vector<Ptr<OranReporterLteEnergyEfficiency>> eeReporters;
     //eeReporters.reserve(numberOfUes);
     
 
     // Instead, for gNB reporters:
     std::vector<Ptr<OranReporterLteEnergyEfficiency>> enbEeReporters;
     enbEeReporters.reserve(numberOfEnbs);
 
 
     // UE Nodes setup
     for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
     {
         Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
         Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
         Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
             CreateObject<OranReporterLteUeCellInfo>();
         Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter = 
         CreateObject<OranReporterLteUeRsrpRsrq>();
         //Ptr<OranReporterLteEnergyEfficiency> eeReporter = 
         //CreateObject<OranReporterLteEnergyEfficiency>();
         Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator =
             CreateObject<OranE2NodeTerminatorLteUe>();
 
         locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
         
         appLossReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
         
         remoteApps.Get(idx)->TraceConnectWithoutContext(
             "Tx",
             MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
         ueApps.Get(idx)->TraceConnectWithoutContext(
             "Rx",
             MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
 
         lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
 
         rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
         
         //eeReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
 
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
         lteUeTerminator->AddReporter(appLossReporter);
         lteUeTerminator->AddReporter(lteUeCellInfoReporter);
         lteUeTerminator->AddReporter(rsrpRsrqReporter);
         //lteUeTerminator->AddReporter(eeReporter);
         
         //eeReporters.push_back(eeReporter);
 
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
                             
     oranHelper->AddReporter("ns3::OranReporterLteEnergyEfficiency",
                             "Trigger",
                             StringValue("ns3::OranReportTriggerPeriodic"));
 
     e2NodeTerminatorsEnbs.Add(oranHelper->DeployTerminators(nearRtRic, enbNodes));
     
     //Ptr<OranReporterLteEnergyEfficiency> enbEeReporters = 
     //    CreateObject<OranReporterLteEnergyEfficiency>();
     // --- Attach energy‐efficiency reporters to each eNB terminator ---
     for (auto it = e2NodeTerminatorsEnbs.Begin(); it != e2NodeTerminatorsEnbs.End(); ++it)
     {
       Ptr<OranE2NodeTerminatorLteEnb> enbTerm =
         DynamicCast<OranE2NodeTerminatorLteEnb>(*it);
       Ptr<OranReporterLteEnergyEfficiency> rpt =
         CreateObject<OranReporterLteEnergyEfficiency>();
       rpt->SetAttribute("Terminator", PointerValue(enbTerm));
       enbTerm->AddReporter(rpt);
       enbEeReporters.push_back(rpt);
     }
     
 
 
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
         mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, enbNodes.Get(i)));
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

     /*NetAnim Packet Tracing*/
     anim.EnableIpv4L3ProtocolCounters(Seconds(0), simTime); // Track IPv4 packets
     //anim.EnablePacketTrackingForDevice(ueLteDevs); // Track packets for UE devices
     //anim.EnablePacketTrackingForDevice(enbLteDevs); // Track packets for eNodeB devices

 
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
     
     // At end of sim, push the gNB‐side KPI into the repository:
     //for (auto& rep : enbEeReporters) {
     //  rep->ReportEnergyEfficiency(energyEfficiency);
     //}
 
     
     
     Simulator::Destroy();
     
     return 0;
 }
 