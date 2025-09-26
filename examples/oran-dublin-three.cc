/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * This program is written by Abdul Wadud from University College Dublin, Ireland. 
 * This implementation is a part of his Ph.D. study at UCD on O-RAN Conflict Mitigation
 */

 /* ---------------------------------------------------------------------------
 * Run Instructions (ORAN-connected: oran-dublin-three)
 * ---------------------------------------------------------------------------
 * Notes:
 *  • Use either --targetUes OR --uePerEnb (pick one).
 *  • Flags assume you added the traffic knobs: --trafficProfile, --usePaperMix,
 *    --embb*, --urllc*, --v2x*, --mmtc* as described in the code comments.
 *  • Example eNB file from your 0.3×0.5 km bbox:
 *      scratch/ns3_positions_Three_IE.txt
 *  • Typical urban settings shown: reuse-3, PF scheduler, LTE PHY error models off.
 * 
 * NS_LOG="LteInterference=level_info|prefix_time:LteSpectrumPhy=level_info|prefix_time" \./ns3 run "oran-dublin-three \
  --enbPosFile=scratch/ns3_positions_Three_IE.txt \
  --targetUes=120 --ueDiscR=120 \
  --usePaperMix=0 --trafficProfile=mixed \
  --sim-time=30 --txPower=30 --reuse=3 \
  --scheduler=pf --disablePhyErr=1 \
  --enableHoLm=1 --enableEnergyLm=1 --enableEnbEeReporter=1"
 *
 * A) Dublin city centre, ~360 UEs, paper traffic mix (bursty), reuse-3
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --usePaperMix=1 --trafficProfile=mixed \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * B) Same but explicit per-eNB UE count (skip targetUes)
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --uePerEnb=12 --ueDiscR=120 \
 *      --usePaperMix=1 --trafficProfile=mixed \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * C) Non-paper MIXED profile (4 apps per UE, all bursty EXP)
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=mixed \
 *      --embbBursty=1 --embbOnDist=exp --embbOffDist=exp --embbOnMean=0.5 --embbOffMean=2.0 --embbRate=10Mbps --embbPkt=1500 \
 *      --urllcBursty=1 --urllcOnDist=exp --urllcOffDist=exp --urllcOnMean=0.02 --urllcOffMean=0.02 --urllcRate=2Mbps --urllcPkt=256 \
 *      --v2xPeriodMs=100 --v2xPkt=300 \
 *      --mmtcOnDist=exp --mmtcOffDist=exp --mmtcOnMean=0.1 --mmtcOffMean=30.0 --mmtcRate=32kbps --mmtcPkt=100 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * D) eMBB only — heavy-tailed bursts (Pareto ON, EXP OFF)
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=embb \
 *      --embbBursty=1 --embbOnDist=pareto --embbOffDist=exp \
 *      --embbOnMean=0.3 --embbOffMean=1.5 --embbRate=10Mbps --embbPkt=1500 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * E) URLLC only — small packets, fast bursts
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=urllc \
 *      --urllcBursty=1 --urllcOnDist=exp --urllcOffDist=exp \
 *      --urllcOnMean=0.02 --urllcOffMean=0.02 --urllcRate=2Mbps --urllcPkt=256 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * F) V2X only — 10 Hz CAM-like UL
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=v2x \
 *      --v2xPeriodMs=100 --v2xPkt=300 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * G) mMTC only — sparse UL On/Off
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=mmtc \
 *      --mmtcOnDist=exp --mmtcOffDist=exp \
 *      --mmtcOnMean=0.1 --mmtcOffMean=30.0 --mmtcRate=32kbps --mmtcPkt=100 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * H) Constant traffic (no bursts), eMBB
 *    ./ns3 run "oran-dublin-three \
 *      --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *      --targetUes=360 --ueDiscR=120 \
 *      --trafficProfile=embb \
 *      --embbBursty=0 \
 *      --sim-time=120 \
 *      --txPower=46 \
 *      --reuse=3 \
 *      --scheduler=pf \
 *      --disablePhyErr=1"
 *
 * Tips for large (300–400 UE) runs:
 *  • Consider longer sims (e.g., --sim-time=120) for steadier bits/J.
 *  • NetAnim/traces can slow things down; increase poll interval or disable for long runs.
 *  • Keep NoOp handover algo so the RIC LM controls HOs.
 * --------------------------------------------------------------------------- */


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

 #include "ns3/oran-ru-energy-model.h" 

 #include "ns3/lte-spectrum-value-helper.h"
 
 // #include <ns3/test.h>
 
 //#include "oran-reporter-lte-energy-efficiency.h"
 
 #include <fstream>
 #include <sstream>
 #include <iostream>
 #include <string>
 #include <stdio.h>
 #include <regex>
 #include <limits>
 #include <cmath>
 #include <unordered_map>

 
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
 
 void ReverseVelocity (NodeContainer nodes, Time interval)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
  {
    Ptr<ConstantVelocityMobilityModel> cv =
        nodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
    if (!cv) continue;  // ← guard against null

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

 // Parse lines like: Vector( 123.4 , 567.8 , 20 )
 static std::vector<Vector>
 LoadEnbPositionsFromVectorFile (const std::string& path)
 {
    std::ifstream in(path.c_str());
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open enbPosFile: " << path);

    std::vector<Vector> pts;
    std::string line;
    std::regex rx(R"(Vector\s*\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))");
    std::smatch m;

    while (std::getline(in, line))
        if (std::regex_search(line, m, rx) && m.size() >= 4)
        pts.emplace_back(std::stod(m[1].str()), std::stod(m[2].str()), std::stod(m[3].str()));

    NS_ABORT_MSG_IF(pts.empty(), "No Vector(x,y,z) lines found in " << path);
    return pts;
 }

//  // Nearest eNB index by 2D distance
//  static uint32_t
//  NearestEnbIndex (const Vector& uePos, const NodeContainer& enbNodes)
//  {
//     double bestD2 = std::numeric_limits<double>::max();
//     uint32_t best = 0;
//     for (uint32_t j = 0; j < enbNodes.GetN(); ++j)
//         {
//         auto mm = enbNodes.Get(j)->GetObject<ConstantPositionMobilityModel>();
//         Vector ep = mm->GetPosition();
//         double dx = uePos.x - ep.x, dy = uePos.y - ep.y;
//         double d2 = dx*dx + dy*dy;
//         if (d2 < bestD2) { bestD2 = d2; best = j; }
//         }
//     return best;
//  }

// ---------- helpers (file scope; no captures) ----------
static std::string RvStr(const std::string& kind, double meanSec)
{
  std::ostringstream os;
  if (kind == "exp") {
    os << "ns3::ExponentialRandomVariable[Mean=" << meanSec << "]";
  } else {
    double a = 1.5;                           // Pareto shape
    double scale = meanSec*(a-1.0)/a;         // so E[X]=meanSec
    os << "ns3::ParetoRandomVariable[Shape=" << a << "|Scale=" << scale << "]";
  }
  return os.str();
}

static void AddDlOnOff(ApplicationContainer& ueApps,
                       ApplicationContainer& remoteApps,
                       Ptr<Node> remoteHost,
                       Ipv4Address dst, Ptr<Node> ueNode, uint16_t port,
                       const std::string& rateStr, uint32_t pkt,
                       bool burst, const std::string& onK, const std::string& offK,
                       double onM, double offM,
                       Ptr<Application>& ueSinkOut,            // ← new
                       Ptr<Application>& remoteOnOffOut)        // ← new
{
  PacketSinkHelper sink("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), port));
  auto s = sink.Install(ueNode);
  Ptr<Application> sinkApp = s.Get(0);
  sinkApp->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
  ueApps.Add(s);
  ueSinkOut = sinkApp;

  Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication>();
  onoff->SetAttribute("Remote",     AddressValue(InetSocketAddress(dst, port)));
  onoff->SetAttribute("DataRate",   DataRateValue(DataRate(rateStr)));
  onoff->SetAttribute("PacketSize", UintegerValue(pkt));
  if (burst) {
    onoff->SetAttribute("OnTime",  StringValue(RvStr(onK, onM)));
    onoff->SetAttribute("OffTime", StringValue(RvStr(offK, offM)));
  } else {
    onoff->SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
  }
  remoteHost->AddApplication(onoff);
  onoff->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
  remoteApps.Add(onoff);
  remoteOnOffOut = onoff;
}

static void AddUlUdpClient(ApplicationContainer& ueApps,
                           ApplicationContainer& remoteApps,
                           Ptr<Node> remoteHost,
                           Ptr<Node> ueNode, Ipv4Address dst, uint16_t port,
                           uint32_t pkt, double periodMs,
                           Ptr<Application>& ueClientOut,        // ← new
                           Ptr<Application>& remoteSinkOut)       // ← new
{
  PacketSinkHelper sink("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), port));
  auto rs = sink.Install(remoteHost);
  Ptr<Application> sinkApp = rs.Get(0);
  sinkApp->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
  remoteApps.Add(rs);
  remoteSinkOut = sinkApp;

  UdpClientHelper client(dst, port);
  client.SetAttribute("MaxPackets", UintegerValue(0));
  client.SetAttribute("Interval",   TimeValue(MilliSeconds(periodMs)));
  client.SetAttribute("PacketSize", UintegerValue(pkt));
  auto c = client.Install(ueNode);
  Ptr<Application> cliApp = c.Get(0);
  ueApps.Add(c);
  ueClientOut = cliApp;
}

static void AddUlOnOff(ApplicationContainer& ueApps,
                       ApplicationContainer& remoteApps,
                       Ptr<Node> remoteHost,
                       Ptr<Node> ueNode, Ipv4Address dst, uint16_t port,
                       const std::string& rateStr, uint32_t pkt,
                       const std::string& onK, const std::string& offK,
                       double onM, double offM,
                       Ptr<Application>& ueOnOffOut,            // ← new
                       Ptr<Application>& remoteSinkOut)          // ← new
{
  PacketSinkHelper sink("ns3::UdpSocketFactory",
                        InetSocketAddress(Ipv4Address::GetAny(), port));
  auto rs = sink.Install(remoteHost);
  Ptr<Application> sinkApp = rs.Get(0);
  sinkApp->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
  remoteApps.Add(rs);
  remoteSinkOut = sinkApp;

  Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication>();
  onoff->SetAttribute("Remote",     AddressValue(InetSocketAddress(dst, port)));
  onoff->SetAttribute("DataRate",   DataRateValue(DataRate(rateStr)));
  onoff->SetAttribute("PacketSize", UintegerValue(pkt));
  onoff->SetAttribute("OnTime",     StringValue(RvStr(onK, onM)));
  onoff->SetAttribute("OffTime",    StringValue(RvStr(offK, offM)));
  ueNode->AddApplication(onoff);
  ueApps.Add(onoff);
  ueOnOffOut = onoff;
}

// Second-best eNB as well (file-scope helper, no captures)
static std::pair<uint32_t,uint32_t>
BestTwoEnbIdx(const Vector& up, const NodeContainer& enbNodes)
{
  double bestD2 = std::numeric_limits<double>::max();
  double secondD2 = bestD2;
  uint32_t best = 0, second = 0;

  for (uint32_t j = 0; j < enbNodes.GetN(); ++j) {
    auto mm = enbNodes.Get(j)->GetObject<ConstantPositionMobilityModel>();
    Vector ep = mm->GetPosition();
    double dx = up.x - ep.x, dy = up.y - ep.y;
    double d2 = dx*dx + dy*dy;
    if (d2 < bestD2) { secondD2 = bestD2; second = best; bestD2 = d2; best = j; }
    else if (d2 < secondD2) { secondD2 = d2; second = j; }
  }
  return {best, second};
}

// Map an LTE CellId to our local eNB index; aborts if the CellId is unknown.
static inline uint32_t
IndexFromCellId(uint16_t cellId,
                const std::unordered_map<uint16_t, uint32_t>& cellId2EnbIdx)
{
  auto it = cellId2EnbIdx.find(cellId);
  NS_ABORT_MSG_IF(it == cellId2EnbIdx.end(),
                  "IndexFromCellId: unknown cellId=" << cellId);
  return it->second;
}

 int
 main(int argc, char* argv[])
 {
    CommandLine cmd(__FILE__);
    double txPower = 30.0; // default TxPower in dBm
    cmd.AddValue("txPower", "eNB TxPower in dBm", txPower);

    // flags for LMs for sanity check
    bool enableHoLm = false;          // OFF by default for stability
    bool enableEnergyLm = false;      // you already pass this from CLI
    bool enableEnbEeReporter = false; // optional
    cmd.AddValue("enableHoLm", "Enable RSRP-based HO LM (0/1)", enableHoLm);
    cmd.AddValue("enableEnergyLm", "Enable Energy-Saving LM (0/1)", enableEnergyLm);
    cmd.AddValue("enableEnbEeReporter", "Enable eNB Energy-Efficiency reporter (0/1)", enableEnbEeReporter);


    uint32_t reuse = 1;              // 1 or 3
    std::string scheduler = "pf";    // "pf" or "rr"
    bool disablePhyErr = false;      // 1 to disable LTE phy error models

    std::vector<Vector> siteCenters;

     // --- Real-topology controls ---
     std::string enbPosFile = "";   // path to a text file with lines like: Vector(x,y,z)
     uint32_t    uePerEnb   = 8;    // how many UEs per eNB (total UEs = uePerEnb * numberOfEnbs)
     double      ueDiscR    = 120;  // UE placement radius (m) around each eNB

     uint32_t targetUes = 0; // if >0, auto-compute uePerEnb to hit ~this many UEs
     cmd.AddValue("targetUes", "Target total UEs across all eNBs (overrides uePerEnb)", targetUes);


     // Default Control
     uint16_t numberOfUes = 4;
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


    // Traffic selector: "embb" | "urllc" | "v2x" | "mmtc" | "mixed"
    std::string trafficProfile = "mixed";
    bool usePaperMix = false;  // if true, overrides trafficProfile with paper’s mix

    // eMBB (DL OnOff)
    bool embbBursty = true;
    std::string embbOnDist = "exp";   // "exp" or "pareto"
    std::string embbOffDist = "exp";
    double embbOnMean = 0.5;          // s
    double embbOffMean = 2.0;         // s
    std::string embbRate = "10Mbps";
    uint32_t embbPkt = 1500;

    // URLLC (DL OnOff)
    bool urllcBursty = true;
    std::string urllcOnDist = "exp";
    std::string urllcOffDist = "exp";
    double urllcOnMean = 0.02;        // s
    double urllcOffMean = 0.02;       // s
    std::string urllcRate = "2Mbps";
    uint32_t urllcPkt = 256;

    // V2X (UL periodic UDP client)
    uint32_t v2xPkt = 300;
    double v2xPeriodMs = 100.0;       // 10 Hz

    // mMTC (UL sparse OnOff)
    std::string mmtcRate = "32kbps";
    uint32_t mmtcPkt = 100;
    std::string mmtcOnDist = "exp";
    std::string mmtcOffDist = "exp";
    double mmtcOnMean = 0.1;          // s
    double mmtcOffMean = 30.0;        // s long OFF

    cmd.AddValue("trafficProfile", "embb|urllc|v2x|mmtc|mixed", trafficProfile);
    cmd.AddValue("usePaperMix", "Override trafficProfile with paper's mix", usePaperMix);

    // eMBB
    cmd.AddValue("embbBursty",  "eMBB bursty ON/OFF (0/1)", embbBursty);
    cmd.AddValue("embbOnDist",  "eMBB ON dist: exp|pareto", embbOnDist);
    cmd.AddValue("embbOffDist", "eMBB OFF dist: exp|pareto", embbOffDist);
    cmd.AddValue("embbOnMean",  "eMBB mean ON (s)", embbOnMean);
    cmd.AddValue("embbOffMean", "eMBB mean OFF (s)", embbOffMean);
    cmd.AddValue("embbRate",    "eMBB ON data rate", embbRate);
    cmd.AddValue("embbPkt",     "eMBB packet size (B)", embbPkt);

    // URLLC
    cmd.AddValue("urllcBursty",  "URLLC bursty ON/OFF (0/1)", urllcBursty);
    cmd.AddValue("urllcOnDist",  "URLLC ON dist: exp|pareto", urllcOnDist);
    cmd.AddValue("urllcOffDist", "URLLC OFF dist: exp|pareto", urllcOffDist);
    cmd.AddValue("urllcOnMean",  "URLLC mean ON (s)", urllcOnMean);
    cmd.AddValue("urllcOffMean", "URLLC mean OFF (s)", urllcOffMean);
    cmd.AddValue("urllcRate",    "URLLC ON data rate", urllcRate);
    cmd.AddValue("urllcPkt",     "URLLC packet size (B)", urllcPkt);

    // V2X
    cmd.AddValue("v2xPkt",      "V2X payload (B)", v2xPkt);
    cmd.AddValue("v2xPeriodMs", "V2X period (ms)", v2xPeriodMs);

    // mMTC
    cmd.AddValue("mmtcRate",    "mMTC ON data rate", mmtcRate);
    cmd.AddValue("mmtcPkt",     "mMTC packet size (B)", mmtcPkt);
    cmd.AddValue("mmtcOnDist",  "mMTC ON dist: exp|pareto", mmtcOnDist);
    cmd.AddValue("mmtcOffDist", "mMTC OFF dist: exp|pareto", mmtcOffDist);
    cmd.AddValue("mmtcOnMean",  "mMTC mean ON (s)", mmtcOnMean);
    cmd.AddValue("mmtcOffMean", "mMTC mean OFF (s)", mmtcOffMean);
 
     // Command line arguments
     //CommandLine cmd(__FILE__);
    cmd.AddValue("reuse",       "Carrier reuse pattern: 1 or 3", reuse);
    cmd.AddValue("scheduler",   "pf|rr (default pf)", scheduler);
    cmd.AddValue("disablePhyErr","Disable LteSpectrumPhy error models", disablePhyErr);

     cmd.AddValue("enbPosFile", "File with eNB positions (lines like Vector(x,y,z))", enbPosFile);
     cmd.AddValue("uePerEnb",   "UEs per eNB when using enbPosFile", uePerEnb);
     cmd.AddValue("ueDiscR",    "UE placement disc radius (m) around each eNB", ueDiscR);

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
     if (disablePhyErr) {
        Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
        Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
      }
      
    
     /*--- lte and epc helper ---*/
     Ptr<LteHelper> lteHelper = CreateObject<LteHelper>(); // create lteHelper
     Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>(); // create epcHelper
     lteHelper->SetEpcHelper(epcHelper); // connect lte to the evolved packet core, which is the core network
    
     // 20 MHz
    const uint8_t kDlRb = 100; // or 75 if you choose narrower carriers
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(kDlRb));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(kDlRb));
    lteHelper->SetSchedulerType( (scheduler=="rr") ? "ns3::RrFfMacScheduler"
                                                     : "ns3::PfFfMacScheduler" );
     lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm"); // disable automatic handover
 
     // Getting the PGW node; it acts as a gateway between LTE and external network, such as- internet.
     Ptr<Node> pgw = epcHelper->GetPgwNode(); // PGW: Packet Data Network Gateway
 
     
    // One-time: site centers live at function scope (declared once above)
    if (!enbPosFile.empty()) {
        siteCenters = LoadEnbPositionsFromVectorFile(enbPosFile);
        numberOfEnbs = static_cast<uint16_t>(siteCenters.size());
        numberOfUes  = (targetUes > 0)
                        ? static_cast<uint16_t>(targetUes)
                        : static_cast<uint16_t>(uePerEnb * numberOfEnbs);
    }
    if (siteCenters.empty()) {
        siteCenters.reserve(numberOfEnbs);
        for (uint16_t k = 0; k < numberOfEnbs; ++k) {
        siteCenters.emplace_back(distance * k, 0.0, 20.0);
        }
    }
    
    NodeContainer enbNodes, ueNodes;              // ← declared ONCE
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    const uint32_t nEnb = enbNodes.GetN();
    const uint32_t nUe  = ueNodes.GetN();

    // Early sanity – fail fast if something drifted
    NS_ABORT_MSG_UNLESS(numberOfEnbs == nEnb, "numberOfEnbs != enbNodes.GetN()");
    NS_ABORT_MSG_UNLESS(numberOfUes  == nUe,  "numberOfUes  != ueNodes.GetN()");

    // Per-UE app buckets (declare ONCE here; do NOT re-declare later)
    std::vector<std::vector<Ptr<Application>>> ueAppsByUe(nUe);
    std::vector<std::vector<Ptr<Application>>> remoteAppsByUe(nUe);

    // Handy maps: NodeId → UE index (can be used by callbacks if needed)
    std::unordered_map<uint32_t, uint32_t> nodeId2UeIdx;
    for (uint32_t i = 0; i < nUe; ++i) {
    nodeId2UeIdx[ueNodes.Get(i)->GetId()] = i;
    }

    // We’ll also keep a container of “vehicular” UEs
    NodeContainer vehicularUes;

    // If no file, build your fallback line of sites
    if (siteCenters.empty())
    {
    siteCenters.reserve(enbNodes.GetN());
    for (uint16_t k = 0; k < numberOfEnbs; ++k)
        siteCenters.emplace_back(distance * k, 0.0, 20.0);
    }

    // Place eNBs
    Ptr<ListPositionAllocator> enbPosAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& v : siteCenters) enbPosAlloc->Add(v);
    MobilityHelper mobEnb;
    mobEnb.SetPositionAllocator(enbPosAlloc);
    mobEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobEnb.Install(enbNodes);

    // Rectangle bounds for RandomWalk UEs (cover all sites + margin)
    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -xmin, ymin = xmin, ymax = -xmin;
    for (const auto& v : siteCenters) {
    xmin = std::min(xmin, v.x); xmax = std::max(xmax, v.x);
    ymin = std::min(ymin, v.y); ymax = std::max(ymax, v.y);
    }
    double margin = std::max(2.0*ueDiscR, 100.0);
    Rectangle rwBounds (xmin - margin, xmax + margin, ymin - margin, ymax + margin);

    // ---- Role table ----
    struct MobilityProfile { std::string name; double speed; double z; };
    static const std::vector<MobilityProfile> kProfiles = {
    {"Pedestrian", 1.5, 1.5}, {"Cyclist", 5.5, 1.5}, {"Car", 15.0, 1.5},
    {"Motorbike", 18.0, 1.5}, {"Bus", 15.0, 1.5}, {"Train", 25.0, 1.5}
    };

    // ---- Per-UE mobility (start positions are now per-site) ----
    Ptr<UniformRandomVariable> U01 = CreateObject<UniformRandomVariable>();
    const double TWO_PI = 6.283185307179586;

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
    Ptr<Node> ue = ueNodes.Get(i);
    const MobilityProfile& P = kProfiles[i % kProfiles.size()];

    const Vector c = siteCenters[i % siteCenters.size()]; // home site for this UE

    double r  = ueDiscR * std::sqrt(U01->GetValue());
    double th = TWO_PI * U01->GetValue();
    Vector start (c.x + r*std::cos(th), c.y + r*std::sin(th), P.z);

    std::string model;
    if (P.name == "Pedestrian")
    {
        model = "ns3::RandomWalk2dMobilityModel";
    }
    else
    {
        model = "ns3::ConstantVelocityMobilityModel";
        if (P.name == "Bus")   start = Vector (c.x + 25.0, c.y - 10.0, P.z);
        if (P.name == "Train") start = Vector (c.x - 10.0, c.y +  0.0, P.z);
        vehicularUes.Add(ue);
    }

    MobilityHelper mh;
    if (model == "ns3::RandomWalk2dMobilityModel")
    {
        mh.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                            "Bounds", RectangleValue(rwBounds),
                            "Speed",  StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));
    }
    else
    {
        mh.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    }

    mh.Install(ue);

    if (auto mm = ue->GetObject<MobilityModel>()) mm->SetPosition(start);
    if (auto cv = ue->GetObject<ConstantVelocityMobilityModel>())
        cv->SetVelocity(Vector(P.speed, 0.0, 0.0));
    }

    // Ensure every node has a MobilityModel
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
    Ptr<Node> n = *it;
    if (!n->GetObject<MobilityModel>())
        n->AggregateObject(CreateObject<ConstantPositionMobilityModel>());
    }

    // UE/eNB course-change traces (safe hooks)
    Ptr<OutputStreamWrapper> ueTrace  = Create<OutputStreamWrapper>("MobilityTrace-UE.tr",  std::ios::out);
    Ptr<OutputStreamWrapper> enbTrace = Create<OutputStreamWrapper>("MobilityTrace-eNB.tr", std::ios::out);

    /* hook UEs */
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
    Ptr<MobilityModel> mob = ueNodes.Get(i)->GetObject<MobilityModel>();
    if (mob)
        mob->TraceConnectWithoutContext("CourseChange",
        MakeBoundCallback(&LogPosition, ueTrace, ueNodes.Get(i)));
    else
        NS_LOG_ERROR("Node " << i << " missing MobilityModel—fix your mobility assignment!");
    }

    /* verify */
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
    if (!ueNodes.Get(i)->GetObject<MobilityModel>())
        NS_LOG_ERROR("MobilityModel missing on UE " << i);
    }

    /* hook eNBs */
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
    auto mm = enbNodes.Get(i)->GetObject<MobilityModel>();
    if (mm)
        mm->TraceConnectWithoutContext("CourseChange",
        MakeBoundCallback(&LogPosition, enbTrace, enbNodes.Get(i)));
    }

    /* Schedule the first direction switch (vehicular only) */
    Simulator::Schedule(interval, &ReverseVelocity, vehicularUes, interval);

    /* Quick probes: only if we actually have UE[4] and UE[5] */
    if (nUe > 5) {
    Simulator::Schedule(Seconds(0.1), &MobilityProbe, 4, ueNodes.Get(4)); // Bus
    Simulator::Schedule(Seconds(10),  &MobilityProbe, 4, ueNodes.Get(4));
    Simulator::Schedule(Seconds(0.1), &MobilityProbe, 5, ueNodes.Get(5)); // Train
    Simulator::Schedule(Seconds(10),  &MobilityProbe, 5, ueNodes.Get(5));
    }


    /* Install LTE Devices in eNB and UEs */
    // Keep track of the EARFCN you assign to each eNB
    std::vector<uint16_t> dlEarfcns(enbNodes.GetN());

    NetDeviceContainer enbLteDevs;
    if (reuse == 3) {
    // pick well-separated EARFCNs (safe spacing)
    const uint32_t dlA = 100, ulA = 18100;
    const uint32_t dlB = 400, ulB = 18400;
    const uint32_t dlC = 700, ulC = 18700;

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
        uint32_t g = i % 3;
        uint16_t dl = g == 0 ? dlA : (g == 1 ? dlB : dlC);
        uint16_t ul = g == 0 ? ulA : (g == 1 ? ulB : ulC);

        lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(dl));
        lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(ul));

        NodeContainer one; one.Add(enbNodes.Get(i));
        NetDeviceContainer d = lteHelper->InstallEnbDevice(one);
        enbLteDevs.Add(d);

        dlEarfcns[i] = dl;
    }
    } else {
    // reuse=1: single carrier for all eNBs; record whatever you set
    const uint16_t dl = 100;                 // change if you set a different one
    lteHelper->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(dl));
    lteHelper->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(18100));
    enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) dlEarfcns[i] = dl;
    }

    /* --- Build eNB ID mappings immediately after install --- */
    std::unordered_map<uint16_t, uint32_t> cellId2EnbIdx;  // CellId -> enb index
    std::unordered_map<uint32_t, uint32_t> nodeId2EnbIdx;  // NodeId -> enb index

    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i) {
    Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
    NS_ABORT_MSG_IF(!enb, "enbLteDevs[" << i << "] is not an LteEnbNetDevice");
    cellId2EnbIdx[enb->GetCellId()]   = i;
    nodeId2EnbIdx[enb->GetNode()->GetId()] = i;
    }

    /* ---- Band dump and overlap check (purely diagnostic) ---- */
    struct Band { double lo, hi; uint16_t cell; uint16_t earfcn; };
    std::vector<Band> dlBands; dlBands.reserve(enbLteDevs.GetN());

    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i) {
    auto enb  = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
    uint16_t cell = enb ? enb->GetCellId() : 0;

    uint16_t dl   = dlEarfcns[i];
    double   fcHz = LteSpectrumValueHelper::GetCarrierFrequency(dl);
    double   bwHz = double(kDlRb) * 180000.0;
    double   lo   = fcHz - 0.5 * bwHz;
    double   hi   = fcHz + 0.5 * bwHz;

    NS_LOG_UNCOND("eNB " << i << " cell=" << cell
                        << " EARFCN_DL=" << dl << " RB=" << unsigned(kDlRb)
                        << " band=[" << lo << "," << hi << "]");

    dlBands.push_back({lo, hi, cell, dl});
    }

    // warn if any overlap
    for (size_t i = 0; i < dlBands.size(); ++i)
    for (size_t j = i + 1; j < dlBands.size(); ++j)
        if (std::max(dlBands[i].lo, dlBands[j].lo) < std::min(dlBands[i].hi, dlBands[j].hi))
        NS_LOG_UNCOND("WARNING: DL band overlap between cell "
                        << dlBands[i].cell << " and " << dlBands[j].cell);

    // Install Ue devices
    NetDeviceContainer ueLteDevs  = lteHelper->InstallUeDevice(ueNodes);

    /* Set eNB Tx power */
    for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it)
    {
    Ptr<LteEnbNetDevice> enbLteDevice = (*it)->GetObject<LteEnbNetDevice>();
    if (enbLteDevice)
        enbLteDevice->GetPhy()->SetTxPower(txPower);
    }

     
    /** Energy Model **/
    /***************************************************************************/
    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(5000000.0)); // 5 MJ
    // Make energy updates more frequent so short LM intervals see non-zero deltas
    sourceHelper.Set("PeriodicEnergyUpdateInterval", TimeValue(MilliSeconds(100)));
    sourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    EnergySourceContainer enbSources = sourceHelper.Install(enbNodes);

    // Keep references for KPI
    std::vector< Ptr<OranRuDeviceEnergyModel> > enbEnergyModels;

    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
    {
    Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
    Ptr<LteEnbPhy> phy = enb->GetPhy();
    Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(enbSources.Get(i));
    NS_ABORT_MSG_IF(!src, "Missing BasicEnergySource for enbSources[" << i << "]");

    Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel>();
    dem->SetEnergySource(src);
    dem->SetLteEnbPhy(phy);

    Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
    ru->SetAttribute("NumTrx", UintegerValue(64));
    ru->SetAttribute("EtaPA", DoubleValue(0.30));
    ru->SetAttribute("FixedOverheadW", DoubleValue(1.25));
    ru->SetAttribute("DeltaAf", DoubleValue(0.5));
    ru->SetAttribute("DeltaDC", DoubleValue(0.07));
    ru->SetAttribute("DeltaMS", DoubleValue(0.09));
    ru->SetAttribute("DeltaCool", DoubleValue(0.10));
    ru->SetAttribute("Vdc", DoubleValue(48.0));
    ru->SetAttribute("SleepPowerW", DoubleValue(5.0));
    ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));

    src->AppendDeviceEnergyModel(dem);
    enbEnergyModels.push_back(dem);
    }

    /* Map: eNB NodeId -> energy source (for safe reporter binding) */
    std::unordered_map<uint32_t, Ptr<BasicEnergySource>> enbEnergyByNode;
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) {
    enbEnergyByNode[ enbNodes.Get(i)->GetId() ] =
        DynamicCast<BasicEnergySource>(enbSources.Get(i));
    }
 
     // Install the IP stack on the UEs
     InternetStackHelper internet;
     internet.Install(ueNodes);
     Ipv4InterfaceContainer ueIpIfaces;
     ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
 
 
    // // Attach each UE to its nearest eNodeB (initial serving cell)
    // for (uint16_t i = 0; i < ueNodes.GetN(); ++i)
    // {
    //     Vector up = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
    //     uint32_t j = (enbNodes.GetN() == 1) ? 0 : NearestEnbIndex(up, enbNodes);
    //     lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(j));
    // }

    // Attach: 2/3 to nearest, 1/3 to second-nearest → early HOs
    for (uint16_t i = 0; i < ueNodes.GetN(); ++i) {
        Vector up = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        auto two = BestTwoEnbIdx(up, enbNodes);          // works in C++11/14/17
        uint32_t best = two.first, second = two.second;  // avoid structured bindings if toolchain whines
        uint32_t j = (enbNodes.GetN() == 1) ? 0 : ((i % 3 == 0 && enbNodes.GetN() >= 2) ? second : best);
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(j));
    }
    
 
     // Add X2 interface
     lteHelper->AddX2Interface(enbNodes);
     
    // ====================== Traffic Application Setup ======================
    ApplicationContainer ueApps, remoteApps;
    uint16_t basePort = 10000;

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

    /* Mobility to PGW and Remote Host */
    NodeContainer infra;
    infra.Add (pgw);
    infra.Add (remoteHost);
    MobilityHelper mobInfra;
    mobInfra.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobInfra.Install (infra);
    pgw->GetObject<MobilityModel> ()->SetPosition (Vector (-20.0, 20.0, 0.0));
    remoteHost->GetObject<MobilityModel> ()->SetPosition (Vector (-25.0, 20.0, 0.0));
    /* Mobility to PGW and Remote Host*/


     /*netAnim Trace Starts here*/

     // NetAnim: place AFTER all nodes are created and assigned mobility
     AnimationInterface anim ("lte-oran-animation.xml");
     anim.SetMaxPktsPerTraceFile (5000000);
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


    // ====================== Traffic Application Setup ======================
    // Buckets to remember exactly which apps belong to which Ue
    // Build apps per UE, record exact handles in the per-UE buckets created earlier
    for (uint16_t i = 0; i < ueNodes.GetN(); ++i)
    {
    uint16_t port = basePort + i * 20; // reserve a small range per UE

    auto keep = [&](Ptr<Application> app, bool toUe) {
        if (!app) return;
        (toUe ? ueAppsByUe[i] : remoteAppsByUe[i]).push_back(app);
    };

    if (usePaperMix)
    {
        switch (i % 4)
        {
        case 0: // DL: high-rate bursty (Pareto ON, Exp OFF)
        {
            Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
            AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    "20Mbps", 1400, true, "pareto", "exp", 0.3, 1.5,
                    ueSink, rhOnOff);
            keep(ueSink, true);
            keep(rhOnOff, false);
            break;
        }
        case 1: // UL: high-rate bursty
        {
            Ptr<Application> ueOnOff = nullptr, rhSink = nullptr;
            AddUlOnOff(ueApps, remoteApps, remoteHost,
                    ueNodes.Get(i), remoteIfaces.GetAddress(1), port++,
                    "20Mbps", 1200, "pareto", "exp", 0.3, 1.5,
                    ueOnOff, rhSink);
            keep(ueOnOff, true);
            keep(rhSink, false);
            break;
        }
        case 2: // DL: medium-rate bursty
        {
            Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
            AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    "750kbps", 800, true, "exp", "exp", 0.6, 2.0,
                    ueSink, rhOnOff);
            keep(ueSink, true);
            keep(rhOnOff, false);
            break;
        }
        default: // DL: low-rate bursty
        {
            Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
            AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    "150kbps", 600, true, "exp", "exp", 1.0, 3.0,
                    ueSink, rhOnOff);
            keep(ueSink, true);
            keep(rhOnOff, false);
            break;
        }
        }
    }
    else
    {
        if (trafficProfile == "embb")
        {
        Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
        AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    embbRate, embbPkt, embbBursty,
                    embbOnDist, embbOffDist, embbOnMean, embbOffMean,
                    ueSink, rhOnOff);
        keep(ueSink, true);
        keep(rhOnOff, false);
        }
        else if (trafficProfile == "urllc")
        {
        Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
        AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    urllcRate, urllcPkt, urllcBursty,
                    urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean,
                    ueSink, rhOnOff);
        keep(ueSink, true);
        keep(rhOnOff, false);
        }
        else if (trafficProfile == "v2x")
        {
        Ptr<Application> ueCli = nullptr, rhSink = nullptr;
        AddUlUdpClient(ueApps, remoteApps, remoteHost,
                        ueNodes.Get(i), remoteIfaces.GetAddress(1), port++,
                        v2xPkt, v2xPeriodMs,
                        ueCli, rhSink);
        keep(ueCli, true);
        keep(rhSink, false);
        }
        else if (trafficProfile == "mmtc")
        {
        Ptr<Application> ueOnOff = nullptr, rhSink = nullptr;
        AddUlOnOff(ueApps, remoteApps, remoteHost,
                    ueNodes.Get(i), remoteIfaces.GetAddress(1), port++,
                    mmtcRate, mmtcPkt, mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean,
                    ueOnOff, rhSink);
        keep(ueOnOff, true);
        keep(rhSink, false);
        }
        else // "mixed": one of each per UE
        {
        // eMBB DL
        {
            Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
            AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    embbRate, embbPkt, embbBursty,
                    embbOnDist, embbOffDist, embbOnMean, embbOffMean,
                    ueSink, rhOnOff);
            keep(ueSink, true);
            keep(rhOnOff, false);
        }
        // URLLC DL
        {
            Ptr<Application> ueSink = nullptr, rhOnOff = nullptr;
            AddDlOnOff(ueApps, remoteApps, remoteHost,
                    ueIpIfaces.GetAddress(i), ueNodes.Get(i), port++,
                    urllcRate, urllcPkt, urllcBursty,
                    urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean,
                    ueSink, rhOnOff);
            keep(ueSink, true);
            keep(rhOnOff, false);
        }
        // V2X UL (UDP client)
        {
            Ptr<Application> ueCli = nullptr, rhSink = nullptr;
            AddUlUdpClient(ueApps, remoteApps, remoteHost,
                        ueNodes.Get(i), remoteIfaces.GetAddress(1), port++,
                        v2xPkt, v2xPeriodMs,
                        ueCli, rhSink);
            keep(ueCli, true);
            keep(rhSink, false);
        }
        // mMTC UL (OnOff)
        {
            Ptr<Application> ueOnOff = nullptr, rhSink2 = nullptr;
            AddUlOnOff(ueApps, remoteApps, remoteHost,
                    ueNodes.Get(i), remoteIfaces.GetAddress(1), port++,
                    mmtcRate, mmtcPkt, mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean,
                    ueOnOff, rhSink2);
            keep(ueOnOff, true);
            keep(rhSink2, false);
        }
        }
    }
    }

    // Start/stop (staggered a bit)
    ueApps.Start(Seconds(2.2));
    ueApps.Stop (simTime - Seconds(0.5));
    remoteApps.Start(Seconds(2.3));
    remoteApps.Stop (simTime - Seconds(0.1));
    // ==================== End Traffic Application Setup ====================
 
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
    //  oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover",
    //                                    "ProcessingDelayRv",
    //                                    StringValue(processingDelayRv));
     oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
     
    //  // --- also run our new energy‐saving LM
    //  oranHelper->AddLogicModule("ns3::OranLmLte2LteEnergySaving",
    //                             "TargetEfficiency", DoubleValue(1e3),  // adjust as you like
    //                             "StepSize",       DoubleValue(1.0));

    // RIC setup:
    if (enableHoLm) {
        oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover",
                                        "ProcessingDelayRv", StringValue(processingDelayRv));
    } // else: no default LM
    
    if (enableEnergyLm) {
        oranHelper->AddLogicModule("ns3::OranLmLte2LteEnergySaving",
                                "TargetEfficiency", DoubleValue(1e3),
                                "StepSize",        DoubleValue(1.0));
    }
                                  
 
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
        // Do not index into remoteApps/ueApps here (ordering/types vary and can be null)
        //  remoteApps.Get(idx)->TraceConnectWithoutContext(
        //      "Tx",
        //      MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
        //  ueApps.Get(idx)->TraceConnectWithoutContext(
        //      "Rx",
        //      MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

        for (auto const& app : remoteAppsByUe[idx]) {
            if (app) { app->TraceConnectWithoutContext("Tx",
               MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter)); }
          }
          for (auto const& app : ueAppsByUe[idx]) {
            if (app) { app->TraceConnectWithoutContext("Rx",
               MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter)); }
          }
          
 
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
        e2NodeTerminatorsUes.Add(lteUeTerminator);

        // Safe UE-side log (avoid undefined 'node' and OOB device indexing)
        Ptr<Node> ueNode = ueNodes.Get(idx);
        std::string firstDev = (ueNode->GetNDevices() > 0)
                                ? ueNode->GetDevice(0)->GetInstanceTypeId().GetName()
                                : std::string("<no devices>");
        NS_LOG_INFO("UE E2 terminator bound: nodeId=" << ueNode->GetId()
                    << " nDevs=" << ueNode->GetNDevices()
                    << " firstDev=" << firstDev);
     }
     
     // ENb Nodes setup
    // --- Deploy eNB E2 terminators ---
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteEnb",
        "RegistrationIntervalRv", StringValue("ns3::ConstantRandomVariable[Constant=1]"),
        "SendIntervalRv",         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    
    oranHelper->AddReporter("ns3::OranReporterLocation",
        "Trigger", StringValue("ns3::OranReportTriggerPeriodic"));
    
    e2NodeTerminatorsEnbs.Add(oranHelper->DeployTerminators(nearRtRic, enbNodes));
    
    // Build: nodeId -> eNB terminator  (we'll use this to bind reporters)
    std::unordered_map<uint32_t, Ptr<OranE2NodeTerminatorLteEnb>> enbTermByNode;
    for (uint32_t i = 0; i < e2NodeTerminatorsEnbs.GetN(); ++i) {
        auto t = DynamicCast<OranE2NodeTerminatorLteEnb>(e2NodeTerminatorsEnbs.Get(i));
        NS_ABORT_MSG_IF(!t, "TerminatorsEnbs[" << i << "] is not an LTE eNB terminator");
        enbTermByNode[t->GetNode()->GetId()] = t;
    }
    
    
    
    // --- Attach energy-efficiency reporters, matched by nodeId ---
    if (enableEnbEeReporter) {
        for (const auto& kv : enbTermByNode) {
        uint32_t nid = kv.first;
        Ptr<OranE2NodeTerminatorLteEnb> term = kv.second;
    
        auto itSrc = enbEnergyByNode.find(nid);
        if (itSrc == enbEnergyByNode.end()) {
            NS_LOG_ERROR("No BasicEnergySource for eNB nodeId=" << nid << "; skipping EE reporter.");
            continue;
        }
    
        Ptr<OranReporterLteEnergyEfficiency> rpt = CreateObject<OranReporterLteEnergyEfficiency>();
        rpt->SetAttribute("Terminator", PointerValue(term));
        rpt->SetEnergySource(itSrc->second);
        term->AddReporter(rpt);
    
        // KEEP a strong ref, so it can’t be collected if the terminator holds a weak Ptr internally.
        enbEeReporters.push_back(rpt);
    
        NS_LOG_INFO("Bound EE reporter to eNB nodeId=" << nid);
        }
        // Optional: assert we really attached one per eNB
        NS_ABORT_MSG_IF(enbEeReporters.size() != enbNodes.GetN(),
        "Energy reporters attached (" << enbEeReporters.size() << ") != number of eNBs ("
        << enbNodes.GetN() << ")");
    }

     // DB logging to the terminal
     if (dbLog)
     {
         nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
     }
 
    // RIC first
    Simulator::Schedule(Seconds(1.0),
    &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    // then E2 terminators
    Simulator::Schedule(Seconds(1.2),
    &OranHelper::ActivateE2NodeTerminators, oranHelper, e2NodeTerminatorsEnbs);
    Simulator::Schedule(Seconds(1.4),
    &OranHelper::ActivateE2NodeTerminators, oranHelper, e2NodeTerminatorsUes);

    // Build E2Id → local eNB index AFTER RIC is up and one registration cycle passed
    Simulator::Schedule(Seconds(2.6), [&](){
    e2Id2EnbIdx.clear();
    for (uint32_t i = 0; i < e2NodeTerminatorsEnbs.GetN(); ++i) {
        auto t = DynamicCast<OranE2NodeTerminatorLteEnb>(e2NodeTerminatorsEnbs.Get(i));
        NS_ABORT_MSG_IF(!t, "TerminatorsEnbs[i] not LTE eNB terminator");
        const uint64_t e2Id   = t->GetE2NodeId();        // now non-zero
        const uint32_t nodeId = t->GetNode()->GetId();
        auto it = nodeId2EnbIdx.find(nodeId);
        NS_ABORT_MSG_IF(it == nodeId2EnbIdx.end(), "nodeId not in nodeId2EnbIdx");
        e2Id2EnbIdx[e2Id] = it->second;
        NS_LOG_UNCOND("E2Id→index (post-activate): E2 " << e2Id
                    << " (nodeId " << nodeId << ") -> " << it->second);
    }
    });

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
    //  lteHelper->EnableRlcTraces();
    //  lteHelper->EnablePdcpTraces();

     /*NetAnim Packet Tracing*/
     anim.EnableIpv4L3ProtocolCounters(Seconds(0), simTime); // Track IPv4 packets
     //anim.EnablePacketTrackingForDevice(ueLteDevs); // Track packets for UE devices
     //anim.EnablePacketTrackingForDevice(enbLteDevs); // Track packets for eNodeB devices

 
     Simulator::Stop(simTime);
     
     
     // turn on INFO‐level logging for our energy‐saving LM
    LogComponentEnable("OranLmLte2LteEnergySaving", LOG_LEVEL_INFO);
    LogComponentEnable("OranNearRtRic", LOG_LEVEL_INFO);
    LogComponentEnable("OranLmLte2LteRsrpHandover", LOG_LEVEL_INFO);
    LogComponentEnable("OranLmLte2LteEnergySaving", LOG_LEVEL_INFO);

 
     // if you want timestamps on *all* log lines globally, do this once:
     // LogComponentEnable("Core", LOG_PREFIX_TIME);
     
     
     Simulator::Run();
     
     
           
    // Calculate Total Energy Consumption
    double totalEnergyJ = 0.0;
    for (auto& dem : enbEnergyModels) {
    totalEnergyJ += dem->GetTotalEnergyConsumption();
    }
    double throughputMbps = (g_totalBytesReceived * 8.0) / simTime.GetSeconds() / 1e6;
    double energyEfficiency = (g_totalBytesReceived * 8.0) / std::max(totalEnergyJ, 1e-12);

    std::cout << "RESULT: " << txPower << ","
            << throughputMbps << ","
            << g_successfulHandover << ","
            << g_unsuccessfulHandover << ","
            << totalEnergyJ << ","
            << energyEfficiency << std::endl;

     Simulator::Destroy();
     
     return 0;
 }
 