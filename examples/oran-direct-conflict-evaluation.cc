/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Example scenario for direct conflict evaluation with ORAN.
 * 
 * This scenario creates 4 eNBs and 30 UEs that are divided into three service classes:
 *  - eMBB: UEs 0-9 use ConstantVelocityMobilityModel (moderate speed).
 *  - URLLC: UEs 10-19 use RandomWaypointMobilityModel with pre-generated waypoints.
 *  - mMTC: UEs 20-29 use RandomWaypointMobilityModel with a different set of pre-generated waypoints.
 *
 * The UEs generate traffic according to their service demands, and the system uses an
 * RSRP-based handover LM. Various performance metrics are traced (e.g., traffic, positions,
 * handovers, RSRP/RSRQ/SINR, throughput, and combined metrics including CQI).
 *
 * Simulation duration: 3600 seconds.
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"      // for PacketSinkHelper, OnOffApplication
#include "ns3/random-variable-stream.h"   // for UniformRandomVariable

#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include <limits>
#include <stdio.h>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("OranDirectConflictEvaluation");

// Trace file names
static std::string s_trafficTraceFile    = "traffic-trace.tr";
static std::string s_positionTraceFile   = "position-trace.tr";
static std::string s_handoverTraceFile   = "handover-trace.tr";
static std::string s_rsrpSinrTraceFile   = "rsrp-sinr-trace.tr";
static std::string s_throughputTraceFile = "throughput-trace.tr";
static std::string s_energyTraceFile     = "energy-trace.tr";
static std::string s_metricsTraceFile    = "metrics-trace.tr";

// ----- Traffic Trace Callbacks -----
void RxTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
  uint16_t ueId = (InetSocketAddress::ConvertFrom (to).GetPort () / 1000);
  std::ofstream out (s_trafficTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\tUE " << ueId << "\tRX " << p->GetSize() << std::endl;
}

void TxTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
  uint16_t ueId = (InetSocketAddress::ConvertFrom (to).GetPort () / 1000);
  std::ofstream out (s_trafficTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\tUE " << ueId << "\tTX " << p->GetSize() << std::endl;
}

// ----- Mobility Trace Callback -----
void LogPosition (Ptr<OutputStreamWrapper> stream, Ptr<Node> node, Ptr<const MobilityModel> mobility)
{
  Vector pos = mobility->GetPosition();
  *stream->GetStream() << Simulator::Now().GetSeconds() << "\tNode " << node->GetId()
                       << "\t" << pos.x << ", " << pos.y << ", " << pos.z << std::endl;
}

// ----- Handover Trace Callback -----
void NotifyHandoverEndOkEnb (uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  std::ofstream out (s_handoverTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\tIMSI:" << imsi 
      << "\tCell:" << cellid 
      << "\tRNTI:" << rnti << std::endl;
  out.close();
}

// ----- RSRP/RSRQ/SINR Trace Callback -----
void LogRsrpRsrqSinr (Ptr<OutputStreamWrapper> stream, uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, uint8_t sinr)
{
  *stream->GetStream() << Simulator::Now().GetSeconds() << "\tRNTI:" << rnti
                       << "\tCell:" << cellId
                       << "\tRSRP:" << rsrp << " dBm"
                       << "\tRSRQ:" << rsrq << " dB"
                       << "\tSINR:" << static_cast<int>(sinr) << " dB" << std::endl;
}

// ----- Throughput Trace Callback -----
void ThroughputTrace (Ptr<const Packet> p, const Address & from, const Address & to)
{
  static std::ofstream out (s_throughputTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\t" << p->GetSize() << std::endl;
}

// ----- Energy Trace Callback (if using energy model) -----
void EnergyTrace (double remainingEnergy)
{
  static std::ofstream out (s_energyTraceFile, std::ios_base::app);
  out << Simulator::Now().GetSeconds() << "\t" << remainingEnergy << std::endl;
}

// ----- Combined Metrics Trace Callback (for RSRP/RSRQ and CQI) -----
void MetricsTrace (Ptr<const OranReport> report)
{
  std::ofstream out (s_metricsTraceFile, std::ios_base::app);
  double now = Simulator::Now().GetSeconds();
  if (report->GetInstanceTypeId() == TypeId::LookupByName("ns3::OranReportLteUeRsrpRsrq"))
  {
      Ptr<const OranReportLteUeRsrpRsrq> rsrpRpt = report->GetObject<const OranReportLteUeRsrpRsrq>();
      out << now << "\tRSRP_RSRQ\t" 
          << rsrpRpt->GetReporterE2NodeId() << "\t" 
          << rsrpRpt->GetRnti() << "\t" 
          << rsrpRpt->GetCellId() << "\t" 
          << rsrpRpt->GetRsrp() << "\t" 
          << rsrpRpt->GetRsrq() << "\t" 
          << rsrpRpt->GetIsServingCell() << "\t" 
          << rsrpRpt->GetComponentCarrierId() << std::endl;
  }
  //else if (report->GetInstanceTypeId() == TypeId::LookupByName("ns3::OranReportLteUeCqi"))
  //{
  //    Ptr<const OranReportLteUeCqi> cqiRpt = report->GetObject<const OranReportLteUeCqi>();
  //    out << now << "\tCQI\t" 
  //        << cqiRpt->GetReporterE2NodeId() << "\t" 
  //        << cqiRpt->GetRnti() << "\t" 
  //        << cqiRpt->GetCellId() << "\t" 
  //        << static_cast<uint32_t>(cqiRpt->GetCqi()) << "\t" 
  //        << cqiRpt->GetComponentCarrierId() << std::endl;
  //}
  out.close();
}

// ----- Reverse Velocity Callback (for UEs using ConstantVelocity) -----
void ReverseVelocity (NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mob = nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        if (mob)
        {
            mob->SetVelocity(Vector(-mob->GetVelocity().x, 0, 0));
        }
    }
    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

int main (int argc, char *argv[])
{
  // Scenario parameters
  uint16_t numberOfEnbs = 2;
  uint16_t numberOfUes = 10;
  Time simTime = Seconds (100);
  double distanceBetweenEnbs = 500; // meters
  double speedEmbb = 5.0;   // eMBB UEs: moderate speed
  // (speedUrllc and speedMmTc are defined but not used because we use RandomWaypoint models for URLLC/mMTC)
  Time lmQueryInterval = Seconds (5);
  std::string dbFileName = "oran-repository.db";
  std::string processingDelayRv = "ns3::NormalRandomVariable[Mean=0.005|Variance=0.000031]";
  std::string lateCommandPolicy = "DROP";

  CommandLine cmd;
  cmd.AddValue("db-file", "Database file name", dbFileName);
  cmd.Parse(argc, argv);

  LogComponentEnable("OranNearRtRic", LOG_LEVEL_WARN);
  Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

  // LTE/EPC configuration
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper(epcHelper);
  lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
  // We use a RSRP-based handover LM (NoOpHandoverAlgorithm disables automatic handover)
  lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");
  Ptr<Node> pgw = epcHelper->GetPgwNode();

  // Create remote host for traffic (only one)
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create(1);
  Ptr<Node> remoteHost = remoteHostContainer.Get(0);
  InternetStackHelper internet;
  internet.Install(remoteHostContainer);
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
  p2p.SetDeviceAttribute("Mtu", UintegerValue(65000));
  p2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
  NetDeviceContainer internetDevices = p2p.Install(pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase("1.1.0.0", "255.255.255.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
  Ipv4StaticRoutingHelper ipv4Routing;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
  remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

  // Create eNB and UE nodes
  NodeContainer enbNodes;
  enbNodes.Create(numberOfEnbs);
  NodeContainer ueNodes;
  ueNodes.Create(numberOfUes);

  // eNB mobility: place them along a line
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator>();
  for (uint16_t i = 0; i < numberOfEnbs; i++) {
      enbPositionAlloc->Add(Vector(i * distanceBetweenEnbs, 0, 30));
  }
  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator(enbPositionAlloc);
  enbMobility.Install(enbNodes);

  // UE initial positions: two clusters
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator>();
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  for (uint16_t i = 0; i < numberOfUes; i++) {
      if (i < numberOfUes/2) {
          double x = (distanceBetweenEnbs / 2) + uv->GetValue(-50,50);
          double y = 200 + uv->GetValue(-50,50);
          uePositionAlloc->Add(Vector(x, y, 1));
      } else {
          double x = (distanceBetweenEnbs / 2) + uv->GetValue(-50,50);
          double y = -200 + uv->GetValue(-50,50);
          uePositionAlloc->Add(Vector(x, y, 1));
      }
  }
  
  // Remove any default mobility model from UE nodes
  for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
    Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
    if (mob != nullptr) {
        // Instead of removing, update the parameters of the existing mobility model.
        // For example, you could change its velocity:
        Ptr<ConstantVelocityMobilityModel> cvMob = DynamicCast<ConstantVelocityMobilityModel>(mob);
        if (cvMob) {
            cvMob->SetVelocity(Vector(-cvMob->GetVelocity().x, 0, 0));
        }
    }
  }

  // For eMBB UEs (indices 0-2), use ConstantVelocityMobilityModel
  for (uint16_t i = 0; i < 3; i++) {
    MobilityHelper mobHelper;
    mobHelper.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobHelper.Install(ueNodes.Get(i));
    Ptr<ConstantVelocityMobilityModel> mob = ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(speedEmbb, 0, 0));
  }

  // For URLLC UEs (indices 3-7)
  for (uint16_t i = 3; i < 8; i++) {
      Ptr<ListPositionAllocator> urllcAlloc = CreateObject<ListPositionAllocator>();
      // Generate 10 random waypoints
      for (uint32_t j = 0; j < 10; j++) {
          double x = uv->GetValue(0, distanceBetweenEnbs);
          double y = uv->GetValue(-300, 300);
          urllcAlloc->Add(Vector(x, y, 1));
      }
      MobilityHelper urllcMobility;
      urllcMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                               "PositionAllocator", PointerValue(urllcAlloc),
                               "Speed", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=2.0]"),
                               "Pause", StringValue("ns3::ConstantRandomVariable[Constant=2]"));
      urllcMobility.Install(ueNodes.Get(i));

  }

  // For mMTC UEs (indices 7-9), use RandomWaypointMobilityModel with pre-generated waypoints
  for (uint16_t i = 8; i < 10; i++) {
    Ptr<ListPositionAllocator> mmtcAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t j = 0; j < 10; j++) {
        double x = uv->GetValue(0, distanceBetweenEnbs);
        double y = uv->GetValue(-300, 300);
        mmtcAlloc->Add(Vector(x, y, 1));
    }
    MobilityHelper mmtcMobility;
    mmtcMobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "PositionAllocator", PointerValue(mmtcAlloc),
                              "Speed", StringValue("ns3::UniformRandomVariable[Min=0.1|Max=0.5]"),
                              "Pause", StringValue("ns3::ConstantRandomVariable[Constant=5]"));
    mmtcMobility.Install(ueNodes.Get(i));
  }

  // Reverse velocity for eMBB UEs periodically
  Simulator::Schedule(Seconds(15), &ReverseVelocity, ueNodes, Seconds(15));

  // Install LTE devices
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

  // Install Internet stack on UEs
  internet.Install(ueNodes);

  // Use your own Ipv4AddressHelper for UEs with a unique network range.
  Ipv4AddressHelper ueIpv4;
  ueIpv4.SetBase("10.1.0.0", "255.255.255.0"); // note the different base from remote host
  Ipv4InterfaceContainer ueIpIfaces = ueIpv4.Assign(NetDeviceContainer(ueLteDevs));

  for (uint32_t u = 0; u < ueNodes.GetN(); u++) {
      Ptr<Node> ue = ueNodes.Get(u);
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4Routing.GetStaticRouting(ue->GetObject<Ipv4>());
      ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
  }

  // Attach each UE to the closest eNB (by Euclidean distance)
  for (uint32_t i = 0; i < ueNodes.GetN(); i++) {
      Ptr<Node> ue = ueNodes.Get(i);
      Vector uePos = ue->GetObject<MobilityModel>()->GetPosition();
      uint16_t bestEnb = 0;
      double bestDist = std::numeric_limits<double>::max();
      for (uint16_t j = 0; j < enbNodes.GetN(); j++) {
          Vector enbPos = enbNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
          double dist = sqrt(pow(uePos.x - enbPos.x,2) + pow(uePos.y - enbPos.y,2));
          if (dist < bestDist) {
              bestDist = dist;
              bestEnb = j;
          }
      }
      lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(bestEnb));
  }

  // Add X2 interface among eNBs
  lteHelper->AddX2Interface(enbNodes);

  // Create remote host for traffic
  NodeContainer remoteHostCont;
  remoteHostCont.Create(1);
  Ptr<Node> remoteHostApp = remoteHostCont.Get(0);
  internet.Install(remoteHostCont);
  NetDeviceContainer remoteDevices = p2p.Install(pgw, remoteHostApp);
  ipv4h.SetBase("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer remoteIfaces = ipv4h.Assign(remoteDevices);
  Ptr<Ipv4StaticRouting> remoteStaticRouting = ipv4Routing.GetStaticRouting(remoteHostApp->GetObject<Ipv4>());
  remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

  // Install traffic applications on UEs and remote host
  ApplicationContainer remoteApps;
  ApplicationContainer ueApps;
  uint16_t basePort = 1000;
  Ptr<UniformRandomVariable> onTimeRv = CreateObject<UniformRandomVariable>();
  onTimeRv->SetAttribute("Min", DoubleValue(1.0));
  onTimeRv->SetAttribute("Max", DoubleValue(5.0));
  Ptr<UniformRandomVariable> offTimeRv = CreateObject<UniformRandomVariable>();
  offTimeRv->SetAttribute("Min", DoubleValue(1.0));
  offTimeRv->SetAttribute("Max", DoubleValue(5.0));
  for (uint16_t i = 0; i < ueNodes.GetN(); i++) {
      uint16_t port = basePort * (i + 1);
      PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), port));
      ueApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(i)));
      ueApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
      Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
      remoteApps.Add(streamingServer);
      streamingServer->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIfaces.GetAddress(i), port)));
      // Set different traffic profiles based on UE category:
      if (i < 10) { // eMBB: high data rate, large packets
          streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("10Mbps")));
          streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
      } else if (i < 20) { // URLLC: moderate data rate, small packets
          streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
          streamingServer->SetAttribute("PacketSize", UintegerValue(200));
      } else { // mMTC: low data rate, very small packets
          streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("100Kbps")));
          streamingServer->SetAttribute("PacketSize", UintegerValue(100));
      }
      streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
      streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));
      remoteHostApp->AddApplication(streamingServer);
      streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
  }
  remoteApps.Start(Seconds(2));
  remoteApps.Stop(simTime + Seconds(10));
  ueApps.Start(Seconds(1));
  ueApps.Stop(simTime + Seconds(15));

  // ----- ORAN Setup -----
  Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
  oranHelper->SetAttribute("Verbose", BooleanValue(true));
  oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
  oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
  oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
  oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0.010)));
  oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue(lateCommandPolicy));

  if (!dbFileName.empty()) {
      std::remove(dbFileName.c_str());
  }
  oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                "DatabaseFile",
                                StringValue(dbFileName));
  oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover",
                                    "ProcessingDelayRv",
                                    StringValue(processingDelayRv));
  oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
  Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

  // Setup UE reporters (for location, cell info, RSRP/RSRQ, and CQI)
  for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++) {
      Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
      Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter = CreateObject<OranReporterLteUeCellInfo>();
      Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterLteUeRsrpRsrq>();
      //Ptr<OranReporterLteUeCqi> cqiReporter = CreateObject<OranReporterLteUeCqi>();
      Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator = CreateObject<OranE2NodeTerminatorLteUe>();

      locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
      lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
      rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
      //cqiReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

      for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++) {
          Ptr<LteUeNetDevice> lteUeDevice = ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
          if (lteUeDevice) {
              Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
              uePhy->TraceConnectWithoutContext("ReportUeMeasurements", MakeCallback(&OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
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
      //lteUeTerminator->AddReporter(cqiReporter);
      lteUeTerminator->Attach(ueNodes.Get(idx));
      Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
  }

  // Setup eNB reporters
  oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteEnb",
                                  "RegistrationIntervalRv",
                                  StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                  "SendIntervalRv",
                                  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
  oranHelper->AddReporter("ns3::OranReporterLocation",
                          "Trigger",
                          StringValue("ns3::OranReportTriggerPeriodic"));
  OranE2NodeTerminatorContainer e2NodeTerminatorsEnbs;
  e2NodeTerminatorsEnbs.Add(oranHelper->DeployTerminators(nearRtRic, enbNodes));

  // Optionally connect database logging (if needed)
  bool dbLog = false;
  if (dbLog) {
      // Uncomment the following line if QueryRcSink is defined:
      // nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
  }

  // Connect combined metrics trace callback
  nearRtRic->TraceConnectWithoutContext("ReportReceived", MakeCallback(&MetricsTrace));

  // Connect handover trace
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                                MakeCallback(&NotifyHandoverEndOkEnb));

  // Trace UE positions
  Ptr<OutputStreamWrapper> mobilityTrace = Create<OutputStreamWrapper>("MobilityTrace.tr", std::ios::out);
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i) {
      Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
      mob->TraceConnectWithoutContext("CourseChange", MakeBoundCallback(&LogPosition, mobilityTrace, ueNodes.Get(i)));
  }

  // Trace RSRP, RSRQ, SINR on UE PHY
  Ptr<OutputStreamWrapper> rsrpSinrTrace = Create<OutputStreamWrapper>("RsrpRsrqSinrTrace.tr", std::ios::out);
  for (NetDeviceContainer::Iterator it = ueLteDevs.Begin(); it != ueLteDevs.End(); ++it) {
      Ptr<NetDevice> device = *it;
      Ptr<LteUeNetDevice> lteUeDevice = device->GetObject<LteUeNetDevice>();
      if (lteUeDevice) {
          Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
          uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr", MakeBoundCallback(&LogRsrpRsrqSinr, rsrpSinrTrace));
      }
  }

  // Enable LTE traces
  lteHelper->EnablePhyTraces();
  lteHelper->EnableMacTraces();
  lteHelper->EnableRlcTraces();
  lteHelper->EnablePdcpTraces();

  Simulator::Stop(simTime);
  Simulator::Run();
  Simulator::Destroy();

  return 0;
}

