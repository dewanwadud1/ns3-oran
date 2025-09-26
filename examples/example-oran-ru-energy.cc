/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- 
 *
 * example-oran-ru-energy.cc
 *
 * Minimal LTE+EPC scenario using the Oran RU Energy Model to compute energy
 * consumption and an energy efficiency KPI (bits/Joule). It also counts
 * successful/unsuccessful handovers and prints a one-line RESULT summary.
 *
 * Build options:
 *  1) Preferred: add oran-ru-energy-model.cc/.h into a module (e.g., contrib/oran/model)
 *     and include <ns3/oran-ru-energy-model.h>.
 *  2) Quick test: place oran-ru-energy-model.cc/.h next to this file and keep
 *     the include as "oran-ru-energy-model.h".
 *
 * ---------------------------------------------------------------------------
 * Run Instructions (examples)
 * ---------------------------------------------------------------------------
 * Defaults: nEnb=2, nUe=4, simTime=30s, distance=50m, speed=1.5 m/s, txPower=30 dBm
 *           RU energy source: 0.5 MJ @ 48 V (set inside the example)
 *
 * A) eMBB (DL OnOff) — Poisson-like bursts (both exponential):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=embb \
 *         --embbBursty=1 --embbOnDist=exp --embbOffDist=exp \
 *         --embbOnMean=0.5 --embbOffMean=2.0 --embbRate=10Mbps --embbPkt=1500"
 *
 * B) eMBB — Heavy-tailed bursts (Pareto ON, exponential OFF):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=embb \
 *         --embbBursty=1 --embbOnDist=pareto --embbOffDist=exp \
 *         --embbOnMean=0.3 --embbOffMean=1.5 --embbRate=10Mbps --embbPkt=1500"
 *
 * C) URLLC (DL OnOff, small packets, fast bursts):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=urllc \
 *         --urllcBursty=1 --urllcOnDist=exp --urllcOffDist=exp \
 *         --urllcOnMean=0.02 --urllcOffMean=0.02 --urllcRate=2Mbps --urllcPkt=256"
 *
 * D) V2X (UL periodic UDP client, e.g., 10 Hz CAMs):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=v2x \
 *         --v2xPeriodMs=100 --v2xPkt=300"
 *
 * E) mMTC (UL sparse OnOff, long OFF, short ON):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=mmtc \
 *         --mmtcOnDist=exp --mmtcOffDist=exp \
 *         --mmtcOnMean=0.1 --mmtcOffMean=30.0 --mmtcRate=32kbps --mmtcPkt=100"
 *
 * F) Mixed (per-UE: eMBB DL + URLLC DL + V2X UL + mMTC UL):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=mixed \
 *         --embbBursty=1 --embbOnDist=exp --embbOffDist=exp \
 *         --urllcBursty=1 --urllcOnDist=exp --urllcOffDist=exp \
 *         --v2xPeriodMs=100 --mmtcOnDist=exp --mmtcOffDist=exp"
 *
 * G) Original constant traffic (no bursts, single DL flow like legacy setup):
 *    ./ns3 run "example-oran-ru-energy --trafficProfile=embb --embbBursty=0"
 *
 * Common general knobs (optional):
 *   --nUe=4 --simTime=30 --txPower=30 --distance=50 --speed=1.5 --enableLogs=1
 *
 * Notes:
 *  • For Pareto ON-times, the scale is chosen internally to match the requested mean.
 *  • RESULT line prints: txPower, throughput(Mbps), HO_success, HO_fail, energy(J), bits/J.
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ExampleOranRuEnergy");

/* ------------------- Globals for KPIs ------------------- */
static uint32_t g_successHandover = 0;
static uint32_t g_failHandover = 0;
static uint64_t g_totalBytesRx = 0;

/* ------------------- Helpers/Callbacks ------------------- */

void
NotifyHandoverEndOkEnb (uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
  g_successHandover++;
  std::cout << Simulator::Now ().As (Time::S) << " HO OK IMSI " << imsi
            << " to Cell " << cellid << " (RNTI " << rnti << ")\n";
}

void
NotifyHandoverFailure (std::string /*context*/, uint64_t imsi, uint16_t rnti, uint16_t targetCellId)
{
  g_failHandover++;
  std::cout << Simulator::Now ().As (Time::S) << " HO FAIL IMSI " << imsi
            << " targetCell " << targetCellId << " (RNTI " << rnti << ")\n";
}

void
RxSinkTrace (Ptr<const Packet> p, const Address &/*from*/)
{
  g_totalBytesRx += p->GetSize ();
}

/** Reverse UEs' x-velocity every 'interval' to induce handovers. */
void
ReverseVelocity (NodeContainer nodes, Time interval)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
    {
      auto cv = nodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
      Vector v = cv->GetVelocity ();
      cv->SetVelocity (Vector (-v.x, v.y, v.z));
    }
  Simulator::Schedule (interval, &ReverseVelocity, nodes, interval);
}

int
main (int argc, char *argv[])
{
  // --------------------- CLI ---------------------
  uint16_t nEnb = 2;
  uint16_t nUe = 4;
  double distance = 50.0;             // meters between eNBs
  double speed = 1.5;                 // m/s UE speed
  Time interval = Seconds (15);       // reverse direction every 15 s
  Time simTime = Seconds (30);
  double txPowerDbm = 30.0;           // eNB Tx power
  bool enableLogs = false;

  // Traffic profile: embb | urllc | v2x | mmtc | mixed
  std::string trafficProfile = "embb";

  // eMBB burst knobs (DL OnOff)
  bool embbBursty = true;
  std::string embbOnDist = "exp";   // exp|pareto
  std::string embbOffDist = "exp";  // exp|pareto
  double embbOnMean = 0.5;          // s
  double embbOffMean = 2.0;         // s
  std::string embbRate = "10Mbps";
  uint32_t embbPkt = 1500;

  // URLLC burst knobs (DL OnOff, short packets)
  bool urllcBursty = true;
  std::string urllcOnDist = "exp";
  std::string urllcOffDist = "exp";
  double urllcOnMean = 0.02;        // s
  double urllcOffMean = 0.02;       // s
  std::string urllcRate = "2Mbps";
  uint32_t urllcPkt = 256;

  // V2X (UL periodic UDP client)
  uint32_t v2xPkt = 300;            // bytes
  double v2xPeriodMs = 100.0;       // 10 Hz default

  // mMTC (UL sparse OnOff)
  std::string mmtcRate = "32kbps";
  uint32_t mmtcPkt = 100;
  std::string mmtcOnDist = "exp";
  std::string mmtcOffDist = "exp";
  double mmtcOnMean = 0.1;          // s
  double mmtcOffMean = 30.0;        // s (long OFF)

  CommandLine cmd (__FILE__);
  cmd.AddValue ("nEnb", "Number of eNBs", nEnb);
  cmd.AddValue ("nUe", "Number of UEs", nUe);
  cmd.AddValue ("distance", "Distance between eNBs [m]", distance);
  cmd.AddValue ("speed", "UE speed [m/s]", speed);
  cmd.AddValue ("simTime", "Simulation time [s]", simTime);
  cmd.AddValue ("txPower", "eNB TxPower [dBm]", txPowerDbm);
  cmd.AddValue ("enableLogs", "Enable component logs", enableLogs);

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
  
  // --- Paper scenario knobs ---
uint32_t ringSites = 7;         // 7 ring eNBs
bool includeCenter = true;       // keep the central eNB (serves 0 UEs here)
double isd = 1700.0;             // inter-site distance (m)
uint32_t uePerSite = 9;          // 9 UEs per ring-site => 63 total
uint32_t excludeSites = 0;       // ξ in {0..3}, number of ring-sites to exclude from UE placement
bool usePaperMix = false;        // enable the paper’s traffic mix
double areaMin = 0.0, areaMax = 4000.0; // simulation bounds (m)

  cmd.AddValue("ringSites", "Number of ring sites (7 to match paper)", ringSites);
  cmd.AddValue("includeCenter", "Place a central eNB (no UEs attached)", includeCenter);
  cmd.AddValue("isd", "Inter-site distance (m)", isd);
  cmd.AddValue("uePerSite", "UEs per ring site (9 to match paper)", uePerSite);
  cmd.AddValue("excludeSites", "Exclude this many ring sites from UE placement (0..3)", excludeSites);
  cmd.AddValue("usePaperMix", "Use paper's TCP/UDP traffic mix (overrides trafficProfile)", usePaperMix);
  cmd.AddValue("areaMin", "Scenario rectangle min (m)", areaMin);
  cmd.AddValue("areaMax", "Scenario rectangle max (m)", areaMax);


  cmd.Parse (argc, argv);

  if (enableLogs)
    {
      LogComponentEnable ("LteHelper", LOG_LEVEL_INFO);
      LogComponentEnable ("ExampleOranRuEnergy", LOG_LEVEL_INFO);
    }

  // --------------------- LTE/EPC helpers ---------------------
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));
  Ptr<LteHelper> lte = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epc = CreateObject<PointToPointEpcHelper> ();
  lte->SetEpcHelper (epc);

  lte->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm"); // natural HOs with motion
  lte->SetSchedulerType ("ns3::RrFfMacScheduler");                 // RR MAC

  // PGW / remote host
  Ptr<Node> pgw = epc->GetPgwNode ();
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

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
                                              Ipv4Mask ("255.0.0.0"),
                                              1);

  // --------------------- Nodes ---------------------
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (nEnb);
  ueNodes.Create (nUe);

  // Mobility: place eNBs along x-axis, UEs start mid and move back/forth
  Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < nEnb; ++i)
    {
      pos->Add (Vector (distance * i, 0.0, 20.0));
    }
  for (uint16_t i = 0; i < nUe; ++i)
    {
      pos->Add (Vector ((distance / 2.0) - speed * (interval.GetSeconds () / 2.0),
                        0.0, 1.5));
    }

  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.SetPositionAllocator (pos);
  mobility.Install (enbNodes);

  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (ueNodes);

  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<ConstantVelocityMobilityModel> m = ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
      m->SetVelocity (Vector (speed, 0, 0));
    }

  Simulator::Schedule (interval, &ReverseVelocity, ueNodes, interval);

  // --------------------- Devices ---------------------
  NetDeviceContainer enbDevs = lte->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs = lte->InstallUeDevice (ueNodes);

  // Set eNB Tx power
  for (auto it = enbDevs.Begin (); it != enbDevs.End (); ++it)
    {
      Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice> (*it);
      NS_ABORT_MSG_IF (enb == nullptr, "Expected LteEnbNetDevice");
      Ptr<LteEnbPhy> phy = enb->GetPhy ();
      phy->SetTxPower (txPowerDbm);
    }

  // IP stack for UEs + address assignment
  InternetStackHelper internetUe;
  internetUe.Install (ueNodes);
  Ipv4InterfaceContainer ueIfaces = epc->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  // Attach all UEs to eNB 0 initially
  for (uint16_t i = 0; i < nUe; ++i)
    {
      lte->Attach (ueDevs.Get (i), enbDevs.Get (0));
    }

  // X2 between eNBs
  lte->AddX2Interface (enbNodes);

  // --------------------- Traffic ---------------------
  ApplicationContainer ueApps, remoteApps; // sinks on both sides
  uint16_t basePort = 10000;

  auto rvStr = [](const std::string& kind, double meanSec)
  {
    std::ostringstream os;
    if (kind == "exp")
      {
        os << "ns3::ExponentialRandomVariable[Mean=" << meanSec << "]";
      }
    else // pareto (heavy tail) with shape 1.5, scale chosen so mean matches meanSec
      {
        double a = 1.5;
        double scale = meanSec * (a - 1.0) / a; // mean = scale * a/(a-1)
        os << "ns3::ParetoRandomVariable[Shape=" << a << "|Scale=" << scale << "]";
      }
    return os.str();
  };

  auto addDlOnOff = [&](Ipv4Address dst, Ptr<Node> ueNode, uint16_t port,
                        const std::string& rateStr, uint32_t pkt,
                        bool burst, const std::string& onK, const std::string& offK,
                        double onM, double offM)
  {
    // UE sink
    PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer sink = sinkHelper.Install (ueNode);
    sink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace));
    ueApps.Add (sink);

    // Remote host OnOff -> UE
    Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication> ();
    onoff->SetAttribute ("Remote", AddressValue (InetSocketAddress (dst, port)));
    onoff->SetAttribute ("DataRate", DataRateValue (DataRate (rateStr)));
    onoff->SetAttribute ("PacketSize", UintegerValue (pkt));

    if (burst)
      {
        onoff->SetAttribute ("OnTime",  StringValue (rvStr (onK,  onM)));
        onoff->SetAttribute ("OffTime", StringValue (rvStr (offK, offM)));
      }
    else
      {
        onoff->SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff->SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

      }

    remoteHost->AddApplication (onoff);
    remoteApps.Add (onoff);
  };

  auto addUlUdpClient = [&](Ptr<Node> ueNode, Ipv4Address dst, uint16_t port,
                            uint32_t pkt, double periodMs)
  {
    // Remote host sink
    PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer rsink = sinkHelper.Install (remoteHost);
    rsink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace));
    remoteApps.Add (rsink);

    // UE client -> remote host
    UdpClientHelper client (dst, port);
    client.SetAttribute ("MaxPackets", UintegerValue (0)); // unlimited
    client.SetAttribute ("Interval", TimeValue (MilliSeconds (periodMs)));
    client.SetAttribute ("PacketSize", UintegerValue (pkt));
    ApplicationContainer c = client.Install (ueNode);
    ueApps.Add (c);
  };

  auto addUlOnOff = [&](Ptr<Node> ueNode, Ipv4Address dst, uint16_t port,
                        const std::string& rateStr, uint32_t pkt,
                        const std::string& onK, const std::string& offK,
                        double onM, double offM)
  {
    // Remote host sink
    PacketSinkHelper sinkHelper ("ns3::UdpSocketFactory",
                                 InetSocketAddress (Ipv4Address::GetAny (), port));
    ApplicationContainer rsink = sinkHelper.Install (remoteHost);
    rsink.Get (0)->TraceConnectWithoutContext ("Rx", MakeCallback (&RxSinkTrace));
    remoteApps.Add (rsink);

    // UE OnOff -> remote host
    Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication> ();
    onoff->SetAttribute ("Remote", AddressValue (InetSocketAddress (dst, port)));
    onoff->SetAttribute ("DataRate", DataRateValue (DataRate (rateStr)));
    onoff->SetAttribute ("PacketSize", UintegerValue (pkt));
    onoff->SetAttribute ("OnTime",  StringValue (rvStr (onK,  onM)));
    onoff->SetAttribute ("OffTime", StringValue (rvStr (offK, offM)));
    ueNode->AddApplication (onoff);
    ueApps.Add (onoff);
  };

  for (uint16_t i = 0; i < nUe; ++i)
    {
      uint16_t port = basePort + i * 20; // block of ports per-UE

      if (trafficProfile == "embb")
        {
          addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++,
                      embbRate, embbPkt,
                      embbBursty, embbOnDist, embbOffDist, embbOnMean, embbOffMean);
        }
      else if (trafficProfile == "urllc")
        {
          addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++,
                      urllcRate, urllcPkt,
                      urllcBursty, urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean);
        }
      else if (trafficProfile == "v2x")
        {
          addUlUdpClient (ueNodes.Get (i), ifaces.GetAddress (1), port++,
                          v2xPkt, v2xPeriodMs);
        }
      else if (trafficProfile == "mmtc")
        {
          addUlOnOff (ueNodes.Get (i), ifaces.GetAddress (1), port++,
                      mmtcRate, mmtcPkt,
                      mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean);
        }
      else // mixed: eMBB DL + URLLC DL + V2X UL + mMTC UL
        {
          addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++,
                      embbRate, embbPkt,
                      embbBursty, embbOnDist, embbOffDist, embbOnMean, embbOffMean);

          addDlOnOff (ueIfaces.GetAddress (i), ueNodes.Get (i), port++,
                      urllcRate, urllcPkt,
                      urllcBursty, urllcOnDist, urllcOffDist, urllcOnMean, urllcOffMean);

          addUlUdpClient (ueNodes.Get (i), ifaces.GetAddress (1), port++,
                          v2xPkt, v2xPeriodMs);

          addUlOnOff (ueNodes.Get (i), ifaces.GetAddress (1), port++,
                      mmtcRate, mmtcPkt,
                      mmtcOnDist, mmtcOffDist, mmtcOnMean, mmtcOffMean);
        }
    }

  ueApps.Start (Seconds (1.0));
  ueApps.Stop  (simTime - Seconds (0.5));
  remoteApps.Start (Seconds (2.0));
  remoteApps.Stop  (simTime - Seconds (1.0));

  // --------------------- Energy Model wiring ---------------------
  BasicEnergySourceHelper sourceHelper;
  sourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (500000.0)); // 0.5 MJ headroom
  sourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (48.0)); // must match RU Vdc
  EnergySourceContainer sources = sourceHelper.Install (enbNodes);

  // Keep references so we can query per-RU consumption at the end
  std::vector< Ptr<OranRuDeviceEnergyModel> > enbEnergyModels;

  for (uint32_t i = 0; i < enbDevs.GetN (); ++i)
    {
      Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice> (enbDevs.Get (i));
      Ptr<LteEnbPhy> phy = enb->GetPhy ();

      Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource> (sources.Get (i));

      Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel> ();
      dem->SetEnergySource (src);
      dem->SetLteEnbPhy (phy);

      Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel ();
      ru->SetAttribute ("NumTrx", UintegerValue (64));
      ru->SetAttribute ("EtaPA", DoubleValue (0.30));
      ru->SetAttribute ("FixedOverheadW", DoubleValue (1.25)); // ≈80 W per RU across 64 TRX
      ru->SetAttribute ("DeltaAf", DoubleValue (0.5));         // ~3 dB feeder loss
      ru->SetAttribute ("DeltaDC", DoubleValue (0.07));
      ru->SetAttribute ("DeltaMS", DoubleValue (0.09));
      ru->SetAttribute ("DeltaCool", DoubleValue (0.10));
      ru->SetAttribute ("Vdc", DoubleValue (48.0));
      ru->SetAttribute ("SleepPowerW", DoubleValue (5.0));
      ru->SetAttribute ("SleepThresholdDbm", DoubleValue (0.0));

      src->AppendDeviceEnergyModel (dem);
      enbEnergyModels.push_back (dem);
    }

  // --------------------- Handover tracing ---------------------
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                                 MakeCallback (&NotifyHandoverEndOkEnb));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
                   MakeCallback (&NotifyHandoverFailure));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
                   MakeCallback (&NotifyHandoverFailure));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
                   MakeCallback (&NotifyHandoverFailure));
  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
                   MakeCallback (&NotifyHandoverFailure));

  // --------------------- Run ---------------------
  Simulator::Stop (simTime);
  Simulator::Run ();

  // --------------------- KPIs ---------------------
  // Robust total energy from the device models (no dependence on initial J or harvesters)
  double totalEnergyJ = 0.0;
  for (const auto &dem : enbEnergyModels)
    {
      totalEnergyJ += dem->GetTotalEnergyConsumption ();
    }

  double throughputMbps = (g_totalBytesRx * 8.0) / simTime.GetSeconds () / 1e6;
  double energyEfficiency = (g_totalBytesRx * 8.0) / std::max (totalEnergyJ, 1e-12);

  std::cout << "RESULT: " << txPowerDbm << ","
            << throughputMbps << ","
            << g_successHandover << ","
            << g_failHandover << ","
            << totalEnergyJ << ","
            << energyEfficiency << std::endl;

  Simulator::Destroy ();
  return 0;
}

