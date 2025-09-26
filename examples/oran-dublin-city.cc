/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- 
 *
 * oran-dublin-three.cc  (robust + safety toggles)
 *
 * Robust defaults:
 *  - 20 MHz (100 RB) DL/UL
 *  - PF scheduler (set --scheduler=rr to try RR)
 *  - Optional reuse-3 EARFCN plan (--reuse=3)
 *  - Higher default eNB TxPower 46 dBm (override with --txPower)
 *
 * Extra toggles to avoid long-run LTE PHY assertions in dense deployments:
 *  - --downlinkOnly=1   : skip all UL traffic even in 'mixed' profile
 *  - --disablePhyErr=1  : disable LteSpectrumPhy data/control error models
 *
 * Example:
 *   ./ns3 run "oran-dublin-city \
 *     --enbPosFile=scratch/ns3_positions_Three_IE.txt \
 *     --uePerEnb=8 --ueDiscR=120 \
 *     --simTime=90 --speed=1.5 --txPower=46 \
 *     --trafficProfile=mixed \
 *     --reuse=3 \
 *     --outFile=results_dublin_three.csv"
 */

 #include "ns3/core-module.h"
 #include "ns3/network-module.h"
 #include "ns3/mobility-module.h"
 #include "ns3/internet-module.h"
 #include "ns3/lte-module.h"
 #include "ns3/point-to-point-module.h"
 #include "ns3/applications-module.h"
 #include "ns3/energy-module.h"
 
 #include "ns3/oran-ru-energy-model.h"   // if integrated as a contrib module
 // #include "oran-ru-energy-model.h"    // use this line instead for quick test in scratch/
 
 #include <vector>
 #include <algorithm>
 #include <sstream>
 #include <fstream>
 #include <regex>
 #include <cmath>
 #include <limits>
 
 using namespace ns3;
 
 NS_LOG_COMPONENT_DEFINE("OranDublinThree");
 
 static uint32_t g_successHandover = 0;
 static uint32_t g_failHandover = 0;
 static uint64_t g_totalBytesRx = 0;
 
 void NotifyHandoverEndOkEnb (uint64_t imsi, uint16_t cellid, uint16_t rnti)
 { g_successHandover++; std::cout << Simulator::Now ().As (Time::S) << " HO OK IMSI " << imsi
     << " to Cell " << cellid << " (RNTI " << rnti << ")\n"; }
 
 void NotifyHandoverFailure (std::string, uint64_t imsi, uint16_t rnti, uint16_t targetCellId)
 { g_failHandover++; std::cout << Simulator::Now ().As (Time::S) << " HO FAIL IMSI " << imsi
     << " targetCell " << targetCellId << " (RNTI " << rnti << ")\n"; }
 
 void RxSinkTrace (Ptr<const Packet> p, const Address &)
 { g_totalBytesRx += p->GetSize (); }
 
 void ReverseVelocity (NodeContainer nodes, Time interval)
 {
   for (uint32_t i = 0; i < nodes.GetN (); ++i)
     {
       auto cv = nodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
       Vector v = cv->GetVelocity ();
       cv->SetVelocity (Vector (-v.x, v.y, v.z));
     }
   Simulator::Schedule (interval, &ReverseVelocity, nodes, interval);
 }
 
 static std::vector<Vector> LoadEnbPositionsFromVectorFile (const std::string& path)
 {
   std::ifstream in(path.c_str());
   NS_ABORT_MSG_IF(!in.is_open(), "Cannot open enbPosFile: " << path);
 
   std::vector<Vector> pts;
   std::string line;
   std::regex rx(R"(Vector\s*\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))");
   std::smatch m;
 
   while (std::getline(in, line))
     {
       if (std::regex_search(line, m, rx) && m.size() >= 4)
         {
           pts.emplace_back(std::stod(m[1].str()), std::stod(m[2].str()), std::stod(m[3].str()));
         }
     }
   NS_ABORT_MSG_IF(pts.empty(), "No Vector(x,y,z) lines found in " << path);
   return pts;
 }
 
//  static uint32_t NearestEnbIndex (const Vector& uePos, const NodeContainer& enbNodes)
//  {
//    double bestD2 = std::numeric_limits<double>::max();
//    uint32_t best = 0;
//    for (uint32_t j = 0; j < enbNodes.GetN(); ++j)
//      {
//        auto mp = enbNodes.Get(j)->GetObject<ConstantPositionMobilityModel>();
//        Vector ep = mp->GetPosition();
//        double dx = uePos.x - ep.x;
//        double dy = uePos.y - ep.y;
//        double d2 = dx*dx + dy*dy;
//        if (d2 < bestD2) { bestD2 = d2; best = j; }
//      }
//    return best;
//  }

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
 
 int main (int argc, char *argv[])
 {
   uint16_t nEnb = 2, nUe = 4;
   double distance = 50.0, speed = 1.5;
   Time interval = Seconds (15), simTime = Seconds (30);
   double txPowerDbm = 46.0;
   bool enableLogs = false;
 
   std::string enbPosFile = "";
   uint32_t uePerEnb = 6;
   double ueDiscR = 120.0;
   std::string outFile = "";
 
   uint32_t reuse = 1;
   uint32_t dlEarfcnA = 100, dlEarfcnB = 300, dlEarfcnC = 500;
   uint32_t ulEarfcnA = 18100, ulEarfcnB = 18300, ulEarfcnC = 18500;
 
   std::string trafficProfile = "embb";
   bool embbBursty = true; std::string embbOnDist = "exp", embbOffDist = "exp";
   double embbOnMean = 0.5, embbOffMean = 2.0; std::string embbRate = "10Mbps"; uint32_t embbPkt = 1500;
   bool urllcBursty = true; std::string urllcOnDist = "exp", urllcOffDist = "exp";
   double urllcOnMean = 0.02, urllcOffMean = 0.02; std::string urllcRate = "2Mbps"; uint32_t urllcPkt = 256;
   uint32_t v2xPkt = 300; double v2xPeriodMs = 100.0;
   std::string mmtcRate = "32kbps"; uint32_t mmtcPkt = 100;
   std::string mmtcOnDist = "exp", mmtcOffDist = "exp"; double mmtcOnMean = 0.1, mmtcOffMean = 30.0;
 
   // NEW toggles
   bool downlinkOnly = false;
   bool disablePhyErr = false;
   std::string scheduler = "pf"; // pf | rr
 
   CommandLine cmd (__FILE__);
   cmd.AddValue ("nEnb", "Number of eNBs (ignored if enbPosFile provided)", nEnb);
   cmd.AddValue ("nUe", "Number of UEs (overridden when enbPosFile is used)", nUe);
   cmd.AddValue ("distance", "Fallback distance between eNBs [m]", distance);
   cmd.AddValue ("speed", "UE speed [m/s]", speed);
   cmd.AddValue ("simTime", "Simulation time [s]", simTime);
   cmd.AddValue ("txPower", "eNB TxPower [dBm]", txPowerDbm);
   cmd.AddValue ("enableLogs", "Enable component logs", enableLogs);
 
   cmd.AddValue ("enbPosFile", "File with eNB positions in lines like Vector(x,y,z)", enbPosFile);
   cmd.AddValue ("uePerEnb",   "UEs per eNB when using enbPosFile", uePerEnb);
   cmd.AddValue ("ueDiscR",    "UE placement disc radius (m) around each eNB", ueDiscR);
   cmd.AddValue ("outFile",    "Optional CSV to append the RESULT line", outFile);
 
   cmd.AddValue ("reuse", "Carrier reuse pattern: 1 or 3", reuse);
   cmd.AddValue ("dlEarfcnA", "DL EARFCN for reuse set A", dlEarfcnA);
   cmd.AddValue ("dlEarfcnB", "DL EARFCN for reuse set B", dlEarfcnB);
   cmd.AddValue ("dlEarfcnC", "DL EARFCN for reuse set C", dlEarfcnC);
   cmd.AddValue ("ulEarfcnA", "UL EARFCN for reuse set A", ulEarfcnA);
   cmd.AddValue ("ulEarfcnB", "UL EARFCN for reuse set B", ulEarfcnB);
   cmd.AddValue ("ulEarfcnC", "UL EARFCN for reuse set C", ulEarfcnC);
 
   cmd.AddValue ("trafficProfile", "embb|urllc|v2x|mmtc|mixed", trafficProfile);
 
   cmd.AddValue ("embbBursty", "eMBB: bursty ON/OFF", embbBursty);
   cmd.AddValue ("embbOnDist", "eMBB ON dist: exp|pareto", embbOnDist);
   cmd.AddValue ("embbOffDist","eMBB OFF dist: exp|pareto", embbOffDist);
   cmd.AddValue ("embbOnMean", "eMBB mean ON (s)", embbOnMean);
   cmd.AddValue ("embbOffMean","eMBB mean OFF (s)", embbOffMean);
   cmd.AddValue ("embbRate",   "eMBB ON data rate", embbRate);
   cmd.AddValue ("embbPkt",    "eMBB packet size (B)", embbPkt);
 
   cmd.AddValue ("urllcBursty","URLLC: bursty ON/OFF", urllcBursty);
   cmd.AddValue ("urllcOnDist","URLLC ON dist: exp|pareto", urllcOnDist);
   cmd.AddValue ("urllcOffDist","URLLC OFF dist: exp|pareto", urllcOffDist);
   cmd.AddValue ("urllcOnMean","URLLC mean ON (s)", urllcOnMean);
   cmd.AddValue ("urllcOffMean","URLLC mean OFF (s)", urllcOffMean);
   cmd.AddValue ("urllcRate",  "URLLC ON data rate", urllcRate);
   cmd.AddValue ("urllcPkt",   "URLLC packet size (B)", urllcPkt);
 
   cmd.AddValue ("v2xPkt",     "V2X payload (B)", v2xPkt);
   cmd.AddValue ("v2xPeriodMs","V2X period (ms)", v2xPeriodMs);
 
   cmd.AddValue ("mmtcRate",   "mMTC ON data rate", mmtcRate);
   cmd.AddValue ("mmtcPkt",    "mMTC packet size (B)", mmtcPkt);
   cmd.AddValue ("mmtcOnDist", "mMTC ON dist: exp|pareto", mmtcOnDist);
   cmd.AddValue ("mmtcOffDist","mMTC OFF dist: exp|pareto", mmtcOffDist);
   cmd.AddValue ("mmtcOnMean", "mMTC mean ON (s)", mmtcOnMean);
   cmd.AddValue ("mmtcOffMean","mMTC mean OFF (s)", mmtcOffMean);
 
   cmd.AddValue ("downlinkOnly","Skip all UL traffic (even in mixed)", downlinkOnly);
   cmd.AddValue ("disablePhyErr","Disable LteSpectrumPhy error models (workaround)", disablePhyErr);
   cmd.AddValue ("scheduler","pf|rr (default pf)", scheduler);
 
   cmd.Parse (argc, argv);
 
   if (enableLogs)
     {
       LogComponentEnable ("LteHelper", LOG_LEVEL_INFO);
       LogComponentEnable ("OranDublinThree", LOG_LEVEL_INFO);
     }
 
   if (disablePhyErr)
     {
       Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
       Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
     }
 
   // LTE/EPC helpers
   Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
   Ptr<LteHelper> lte = CreateObject<LteHelper> ();
   Ptr<PointToPointEpcHelper> epc = CreateObject<PointToPointEpcHelper> ();
   lte->SetEpcHelper (epc);
 
   lte->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (100));
   lte->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (100));
   if (scheduler == "rr") lte->SetSchedulerType ("ns3::RrFfMacScheduler");
   else lte->SetSchedulerType ("ns3::PfFfMacScheduler");
 
   lte->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
 
   // PGW / remote host
   Ptr<Node> pgw = epc->GetPgwNode ();
   NodeContainer remoteHostContainer; remoteHostContainer.Create (1);
   Ptr<Node> remoteHost = remoteHostContainer.Get (0);
   InternetStackHelper internet; internet.Install (remoteHostContainer);
 
   PointToPointHelper p2p;
   p2p.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
   p2p.SetDeviceAttribute ("Mtu", UintegerValue (65000));
   p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (0)));
   NetDeviceContainer p2pDevs = p2p.Install (pgw, remoteHost);
 
   Ipv4AddressHelper ipv4h;
   ipv4h.SetBase ("1.1.0.0", "255.255.255.0");
   Ipv4InterfaceContainer ifaces = ipv4h.Assign (p2pDevs);
 
   Ipv4StaticRoutingHelper ipv4RoutingHelper;
   Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
       ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
   remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"),
                                               Ipv4Mask ("255.0.0.0"), 1);
 
   // Nodes & Mobility
   NodeContainer enbNodes, ueNodes;
   Ptr<ListPositionAllocator> enbPosAlloc = CreateObject<ListPositionAllocator>();
 
   if (!enbPosFile.empty())
     {
       std::vector<Vector> sites = LoadEnbPositionsFromVectorFile(enbPosFile);
       nEnb = static_cast<uint16_t>(sites.size());
       for (const auto& v : sites) { enbPosAlloc->Add(v); }
       enbNodes.Create(nEnb);
       MobilityHelper mEnb; mEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
       mEnb.SetPositionAllocator(enbPosAlloc); mEnb.Install(enbNodes);
 
       nUe = static_cast<uint16_t>(nEnb * uePerEnb);
       ueNodes.Create(nUe);
 
       Ptr<ListPositionAllocator> uePosAlloc = CreateObject<ListPositionAllocator>();
       Ptr<UniformRandomVariable> U01 = CreateObject<UniformRandomVariable>();
       const double TWO_PI = 6.283185307179586;
       uint32_t u = 0;
       for (uint32_t s = 0; s < nEnb && u < nUe; ++s)
         {
           Vector c = enbNodes.Get(s)->GetObject<ConstantPositionMobilityModel>()->GetPosition();
           for (uint32_t k = 0; k < uePerEnb && u < nUe; ++k, ++u)
             {
               double r = ueDiscR * std::sqrt(U01->GetValue());
               double th = TWO_PI * U01->GetValue();
               uePosAlloc->Add(Vector(c.x + r*std::cos(th), c.y + r*std::sin(th), 1.5));
             }
         }
       MobilityHelper mUe; mUe.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
       mUe.SetPositionAllocator(uePosAlloc); mUe.Install(ueNodes);
       for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
         { ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(speed, 0.0, 0.0)); }
     }
   else
     {
       enbNodes.Create (nEnb); ueNodes.Create (nUe);
       Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator> ();
       for (uint16_t i = 0; i < nEnb; ++i) pos->Add (Vector (distance * i, 0.0, 20.0));
       for (uint16_t i = 0; i < nUe;  ++i) pos->Add (Vector ((distance/2.0)-speed*(interval.GetSeconds()/2.0), 0.0, 1.5));
 
       MobilityHelper m; m.SetMobilityModel ("ns3::ConstantPositionMobilityModel"); m.SetPositionAllocator (pos); m.Install (enbNodes);
       m.SetMobilityModel ("ns3::ConstantVelocityMobilityModel"); m.Install (ueNodes);
       for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
         { ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (speed, 0, 0)); }
     }
 
   Simulator::Schedule (interval, &ReverseVelocity, ueNodes, interval);
 
   // Devices (with optional reuse-3)
   NetDeviceContainer enbDevs;
   if (reuse == 3 && enbNodes.GetN() > 0)
     {
       for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
         {
           uint32_t idx = i % 3;
           if (idx == 0) { lte->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(dlEarfcnA)); lte->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(ulEarfcnA)); }
           else if (idx == 1) { lte->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(dlEarfcnB)); lte->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(ulEarfcnB)); }
           else { lte->SetEnbDeviceAttribute("DlEarfcn", UintegerValue(dlEarfcnC)); lte->SetEnbDeviceAttribute("UlEarfcn", UintegerValue(ulEarfcnC)); }
           NodeContainer one; one.Add(enbNodes.Get(i));
           NetDeviceContainer d = lte->InstallEnbDevice(one); enbDevs.Add(d);
         }
     }
   else
     {
       enbDevs = lte->InstallEnbDevice (enbNodes);
     }
 
   NetDeviceContainer ueDevs = lte->InstallUeDevice (ueNodes);
 
   // eNB Tx power
   for (auto it = enbDevs.Begin (); it != enbDevs.End (); ++it)
     {
       Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice> (*it);
       Ptr<LteEnbPhy> phy = enb->GetPhy (); phy->SetTxPower (txPowerDbm);
     }
 
   // IP, attach, X2
   InternetStackHelper internetUe; internetUe.Install (ueNodes);
   Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address (NetDeviceContainer (ueDevs));
//    for (uint16_t i = 0; i < ueNodes.GetN(); ++i)
//      {
//        Vector up = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
//        uint32_t j = (enbNodes.GetN() == 1) ? 0 : NearestEnbIndex(up, enbNodes);
//        lte->Attach (ueDevs.Get (i), enbDevs.Get (j));
//      }

     // Attach: 2/3 to nearest, 1/3 to second-nearest → early HOs
    for (uint16_t i = 0; i < ueNodes.GetN(); ++i) {
        Vector up = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        auto two = BestTwoEnbIdx(up, enbNodes);          // works in C++11/14/17
        uint32_t best = two.first, second = two.second;  // avoid structured bindings if toolchain whines
        uint32_t j = (enbNodes.GetN() == 1) ? 0 : ((i % 3 == 0 && enbNodes.GetN() >= 2) ? second : best);
        lte->Attach(ueDevs.Get(i), enbDevs.Get(j));
    }

   if (enbNodes.GetN() > 1) { lte->AddX2Interface (enbNodes); }
 
   // Traffic
   ApplicationContainer ueApps, remoteApps;
   uint16_t basePort = 10000;
 
   auto rvStr = [](const std::string& kind, double meanSec)
   {
     std::ostringstream os;
     if (kind == "exp") { os << "ns3::ExponentialRandomVariable[Mean=" << meanSec << "]"; }
     else { double a = 1.5; double scale = meanSec * (a - 1.0) / a; os << "ns3::ParetoRandomVariable[Shape=" << a << "|Scale=" << scale << "]"; }
     return os.str();
   };
 
   auto addDlOnOff = [&](Ipv4Address dst, Ptr<Node> ueNode, uint16_t port,
                         const std::string& rateStr, uint32_t pkt, bool burst,
                         const std::string& onK, const std::string& offK, double onM, double offM)
   {
     PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
     ApplicationContainer sink = sinkHelper.Install (ueNode);
     sink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace)); ueApps.Add (sink);
 
     Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication> ();
     onoff->SetAttribute ("Remote", AddressValue (InetSocketAddress (dst, port)));
     onoff->SetAttribute ("DataRate", DataRateValue (DataRate (rateStr)));
     onoff->SetAttribute ("PacketSize", UintegerValue (pkt));
 
     if (burst) { onoff->SetAttribute ("OnTime",  StringValue (rvStr (onK,  onM)));
                  onoff->SetAttribute ("OffTime", StringValue (rvStr (offK, offM))); }
     else { onoff->SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
            onoff->SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]")); }
 
     remoteHost->AddApplication (onoff); remoteApps.Add (onoff);
   };
 
   auto addUlUdpClient = [&](Ptr<Node> ueNode, Ipv4Address dst, uint16_t port, uint32_t pkt, double periodMs)
   {
     PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
     ApplicationContainer rsink = sinkHelper.Install (remoteHost);
     rsink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace)); remoteApps.Add (rsink);
 
     UdpClientHelper client (dst, port);
     client.SetAttribute ("MaxPackets", UintegerValue (0));
     client.SetAttribute ("Interval", TimeValue (MilliSeconds (periodMs)));
     client.SetAttribute ("PacketSize", UintegerValue (pkt));
     ApplicationContainer c = client.Install (ueNode); ueApps.Add (c);
   };
 
   auto addUlOnOff = [&](Ptr<Node> ueNode, Ipv4Address dst, uint16_t port, const std::string& rateStr,
                         uint32_t pkt, const std::string& onK, const std::string& offK, double onM, double offM)
   {
     PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
     ApplicationContainer rsink = sinkHelper.Install (remoteHost);
     rsink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace)); remoteApps.Add (rsink);
 
     Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication> ();
     onoff->SetAttribute ("Remote", AddressValue (InetSocketAddress (dst, port)));
     onoff->SetAttribute ("DataRate", DataRateValue (DataRate (rateStr)));
     onoff->SetAttribute ("PacketSize", UintegerValue (pkt));
     onoff->SetAttribute ("OnTime",  StringValue (rvStr (onK,  onM)));
     onoff->SetAttribute ("OffTime", StringValue (rvStr (offK, offM)));
     ueNode->AddApplication (onoff); ueApps.Add (onoff);
   };
 
   for (uint16_t i = 0; i < nUe; ++i)
     {
       uint16_t port = basePort + i * 20;
 
       if (trafficProfile == "embb")
         {
           addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++, embbRate, embbPkt,
                       embbBursty, embbOnDist, embbOffDist, embbOnMean, embbOffMean);
         }
       else if (trafficProfile == "urllc")
         {
           addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++, urllcRate, urllcPkt,
                       urllcBursty, urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean);
         }
       else if (trafficProfile == "v2x" && !downlinkOnly)
         {
           addUlUdpClient (ueNodes.Get (i), ifaces.GetAddress (1), port++, v2xPkt, v2xPeriodMs);
         }
       else if (trafficProfile == "mmtc" && !downlinkOnly)
         {
           addUlOnOff (ueNodes.Get (i), ifaces.GetAddress (1), port++, mmtcRate, mmtcPkt,
                       mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean);
         }
       else // mixed
         {
           addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++, embbRate, embbPkt,
                       embbBursty, embbOnDist, embbOffDist, embbOnMean, embbOffMean);
           addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++, urllcRate, urllcPkt,
                       urllcBursty, urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean);
 
           if (!downlinkOnly)
             {
               addUlUdpClient (ueNodes.Get (i), ifaces.GetAddress (1), port++, v2xPkt, v2xPeriodMs);
               addUlOnOff (ueNodes.Get (i), ifaces.GetAddress (1), port++, mmtcRate, mmtcPkt,
                           mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean);
             }
         }
     }
 
   ueApps.Start (Seconds (1.0)); ueApps.Stop  (simTime - Seconds (0.5));
   remoteApps.Start (Seconds (2.0)); remoteApps.Stop  (simTime - Seconds (1.0));
 
   // Energy model
   BasicEnergySourceHelper sourceHelper;
   sourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (500000.0));
   sourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (48.0));
   EnergySourceContainer sources = sourceHelper.Install (enbNodes);
 
   std::vector< Ptr<OranRuDeviceEnergyModel> > enbEnergyModels;
   for (uint32_t i = 0; i < enbDevs.GetN (); ++i)
     {
       Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice> (enbDevs.Get (i));
       Ptr<LteEnbPhy> phy = enb->GetPhy ();
       Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource> (sources.Get (i));
 
       Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel> ();
       dem->SetEnergySource (src); dem->SetLteEnbPhy (phy);
 
       Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel ();
       ru->SetAttribute ("NumTrx", UintegerValue (64));
       ru->SetAttribute ("EtaPA", DoubleValue (0.30));
       ru->SetAttribute ("FixedOverheadW", DoubleValue (1.25));
       ru->SetAttribute ("DeltaAf", DoubleValue (0.5));
       ru->SetAttribute ("DeltaDC", DoubleValue (0.07));
       ru->SetAttribute ("DeltaMS", DoubleValue (0.09));
       ru->SetAttribute ("DeltaCool", DoubleValue (0.10));
       ru->SetAttribute ("Vdc", DoubleValue (48.0));
       ru->SetAttribute ("SleepPowerW", DoubleValue (5.0));
       ru->SetAttribute ("SleepThresholdDbm", DoubleValue (0.0));
 
       src->AppendDeviceEnergyModel (dem); enbEnergyModels.push_back (dem);
     }
 
   Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk", MakeCallback (&NotifyHandoverEndOkEnb));
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach", MakeCallback (&NotifyHandoverFailure));
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble", MakeCallback (&NotifyHandoverFailure));
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining", MakeCallback (&NotifyHandoverFailure));
   Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving", MakeCallback (&NotifyHandoverFailure));
 
   Simulator::Stop (simTime);
   Simulator::Run ();
 
   double totalEnergyJ = 0.0; for (const auto &dem : enbEnergyModels) totalEnergyJ += dem->GetTotalEnergyConsumption ();
   double throughputMbps = (g_totalBytesRx * 8.0) / simTime.GetSeconds () / 1e6;
   double energyEfficiency = (g_totalBytesRx * 8.0) / std::max (totalEnergyJ, 1e-12);
 
   std::cout << "RESULT: " << txPowerDbm << ","
             << throughputMbps << ","
             << g_successHandover << ","
             << g_failHandover << ","
             << totalEnergyJ << ","
             << energyEfficiency << std::endl;
 
   if (!outFile.empty())
     {
       std::ofstream ofs(outFile.c_str(), std::ios::app);
       ofs << txPowerDbm << "," << throughputMbps << "," << g_successHandover << ","
           << g_failHandover << "," << totalEnergyJ << "," << energyEfficiency << "\n";
     }
 
   Simulator::Destroy ();
   return 0;
 }
 