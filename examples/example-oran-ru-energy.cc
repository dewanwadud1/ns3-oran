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
 *           RU energy source: 0.5 MJ @ 48 V
 *           RandomWalk bounds: [areaMin, areaMax]^2 (defaults: 0..4000 m)
 *
 * ---------- Traffic Profiles (single-cell / simple geometry) ----------
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
 * ---------- Paper-style Geometry (ring of sites) ----------
 * (Central eNB + 7 ring eNBs at ISD=1700 m; 9 UEs per ring-site = 63 UEs.
 *  UEs move with RandomWalk2d, speed U(2,4) m/s.)
 *
 * H) Paper geometry + paper traffic mix (LTE-only, 10 s):
 *    ./ns3 run "example-oran-ru-energy \
 *         --ringSites=7 --includeCenter=1 --isd=1700 --uePerSite=9 \
 *         --excludeSites=0 --usePaperMix=1 --simTime=10 \
 *         --areaMin=-1400 --areaMax=5400"
 *    (areaMin/Max widened so initial UE drops are inside RandomWalk bounds.)
 *
 * I) Paper geometry + non-uniform allocation (exclude ξ sites from UE drops):
 *    ./ns3 run "example-oran-ru-energy \
 *         --ringSites=7 --includeCenter=1 --isd=1700 --uePerSite=9 \
 *         --excludeSites=2 --usePaperMix=1 --simTime=10 \
 *         --areaMin=-1400 --areaMax=5400"
 *
 * J) Paper geometry + mixed profiles (your eMBB/URLLC/V2X/mMTC):
 *    ./ns3 run "example-oran-ru-energy \
 *         --ringSites=7 --includeCenter=1 --isd=1700 --uePerSite=9 \
 *         --excludeSites=0 --usePaperMix=0 --trafficProfile=mixed --simTime=30 \
 *         --areaMin=-1400 --areaMax=5400"
 *
 * ---------- Triggering Handovers quickly (for demos) ----------
 *
 * K) Smaller cells and longer run (more HOs without xApp/BS toggling):
 *    ./ns3 run "example-oran-ru-energy \
 *         --ringSites=7 --includeCenter=1 --isd=300 --uePerSite=9 \
 *         --usePaperMix=1 --simTime=120 --areaMin=-800 --areaMax=800"
 *
 * L) Edge seeding (start UEs near site edges to encourage HOs):
 *    (In code: set rMin=0.8*isd and rMax=isd for the RandomDiscPositionAllocator.)
 *    Then run like (H) or (J).
 *
 * ---------- General knobs ----------
 *   --nUe=<int> --simTime=<s> --txPower=<dBm> --distance=<m> --speed=<m/s> --enableLogs=1
 *   --ringSites=<int> --includeCenter=1|0 --isd=<m> --uePerSite=<int> --excludeSites=<0..3>
 *   --areaMin=<m> --areaMax=<m>          (sets RandomWalk bounding square)
 *   --usePaperMix=1|0                     (1 = overrides trafficProfile with paper mix)
 *   --RngRun=<seed>                       (reproducibility)
 *
 * Notes:
 *  • With ISD=1700 m and simTime=10 s, UEs only move ~20–40 m → few/no handovers unless you
 *    toggle BSs via an xApp or shrink ISD / run longer / seed near edges.
 *  • For Pareto ON-times, the scale is chosen internally to match the requested mean.
 *  • RESULT line prints: txPower, throughput(Mbps), HO_success, HO_fail, energy(J), bits/J.
 *  • To match the paper’s 20 MHz LTE channel, ensure the code sets Dl/UlBandwidth to 100 RBs
 *    (see comments near LteHelper setup).
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
  uint16_t nEnb = 7;
  uint16_t nUe = 63;
  double distance = 1700.0;             // meters between eNBs
  double speed = 1.5;                 // m/s UE speed
  Time interval = Seconds (15);       // reverse direction every 15 s
  Time simTime = Seconds (60);
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

  // --- Paper scenario knobs ---
  uint32_t ringSites = 7;         // 7 ring eNBs
  bool includeCenter = true;       // keep the central eNB (serves 0 UEs here)
  double isd = 1700.0;             // inter-site distance (m)
  uint32_t uePerSite = 9;          // 9 UEs per ring-site => 63 total
  uint32_t excludeSites = 0;       // ξ in {0..3}, number of ring-sites to exclude from UE placement
  bool usePaperMix = false;        // enable the paper’s traffic mix
  double areaMin = 0.0, areaMax = 4000.0; // simulation bounds (m)

  // Mobility mix (fractions must sum to ~1.0). Vehicle classes use ConstantVelocity.
  std::string mobilityMode = "random"; // random | next-site
  double pedFrac   = 0.50;   // pedestrians
  double bikeFrac  = 0.10;   // bicycles/scooters
  double carFrac   = 0.25;   // cars
  double busFrac   = 0.10;   // buses
  double trainFrac = 0.05;   // trains / high-speed



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

  cmd.AddValue("ringSites", "Number of ring sites (7 to match paper)", ringSites);
  cmd.AddValue("includeCenter", "Place a central eNB (no UEs attached)", includeCenter);
  cmd.AddValue("isd", "Inter-site distance (m)", isd);
  cmd.AddValue("uePerSite", "UEs per ring site (9 to match paper)", uePerSite);
  cmd.AddValue("excludeSites", "Exclude this many ring sites from UE placement (0..3)", excludeSites);
  cmd.AddValue("usePaperMix", "Use paper's TCP/UDP traffic mix (overrides trafficProfile)", usePaperMix);
  cmd.AddValue("areaMin", "Scenario rectangle min (m)", areaMin);
  cmd.AddValue("areaMax", "Scenario rectangle max (m)", areaMax);

  cmd.AddValue("mobilityMode", "random|next-site", mobilityMode);
  cmd.AddValue("pedFrac",   "Fraction pedestrians", pedFrac);
  cmd.AddValue("bikeFrac",  "Fraction bikes",       bikeFrac);
  cmd.AddValue("carFrac",   "Fraction cars",        carFrac);
  cmd.AddValue("busFrac",   "Fraction buses",       busFrac);
  cmd.AddValue("trainFrac", "Fraction trains",      trainFrac);

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

  Config::SetDefault("ns3::A3RsrpHandoverAlgorithm::Hysteresis", DoubleValue(3.0));
  Config::SetDefault("ns3::A3RsrpHandoverAlgorithm::TimeToTrigger", TimeValue(MilliSeconds(256)));
  lte->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm"); // natural HOs with motion
  //lte->SetSchedulerType ("ns3::RrFfMacScheduler");                 // RR MAC
  // wider channel (paper uses 20 MHz => 100 RB)
  // ---- Single-carrier LTE, 20 MHz, consistent across *all* nodes ----
  const uint8_t  rb20MHz  = 100;      // 20 MHz
  const uint16_t dlEarfcn = 2450;     // pick any FR1-ish EARFCN; must be consistent!
  const uint16_t ulEarfcn = 20450;

  lte->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (rb20MHz));
  lte->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (rb20MHz));
  lte->SetEnbDeviceAttribute ("DlEarfcn",    UintegerValue (dlEarfcn));
  lte->SetEnbDeviceAttribute ("UlEarfcn",    UintegerValue (ulEarfcn));

  // IMPORTANT: UEs need matching EARFCNs (bandwidth is eNB-only)
  lte->SetUeDeviceAttribute  ("DlEarfcn",    UintegerValue (dlEarfcn));
  // lte->SetUeDeviceAttribute  ("UlEarfcn",    UintegerValue (ulEarfcn)); // this attribute is not valid in ue


  // (Recommended under load)
  lte->SetSchedulerType ("ns3::PfFfMacScheduler");



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

  // --------------------- Nodes & geometry ---------------------
  NodeContainer enbNodes;
  NodeContainer ueNodes;

  // central + ring sites
  uint32_t numSites = ringSites + (includeCenter ? 1u : 0u);
  enbNodes.Create (numSites);

  // choose ring angles (equally spaced)
  std::vector<Vector> sitePos;
  sitePos.reserve(numSites);
  const double cx = (areaMin + areaMax) * 0.5;
  const double cy = (areaMin + areaMax) * 0.5;

  // center first (index 0) if requested
  //uint32_t centerIdx = 0;
  if (includeCenter)
  {
    sitePos.push_back(Vector(cx, cy, 20.0));
  }

  // ring sites
  for (uint32_t k = 0; k < ringSites; ++k)
  {
    double theta = 2.0 * M_PI * (double)k / (double)ringSites;
    double x = cx + isd * std::cos(theta);
    double y = cy + isd * std::sin(theta);
    sitePos.push_back(Vector(x, y, 20.0));
  }

  // place eNBs
  Ptr<ListPositionAllocator> ePos = CreateObject<ListPositionAllocator>();
  for (auto const& v : sitePos) { ePos->Add(v); }
  MobilityHelper mEnb;
  mEnb.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mEnb.SetPositionAllocator(ePos);
  mEnb.Install(enbNodes);

  // --------------------- UE creation per-site ---------------------
  NS_ABORT_MSG_IF(excludeSites > ringSites, "excludeSites > ringSites");

  // random set of excluded ring-site indices
  std::set<uint32_t> excludedRing;
  if (excludeSites > 0)
  {
    Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
    while (excludedRing.size() < excludeSites)
    {
      uint32_t idx = (uint32_t)u->GetInteger(0, ringSites - 1);
      excludedRing.insert(idx);
    }
  }

  // we do NOT allocate UEs to the center (to match 63 UEs total)
  std::vector<NodeContainer> siteUes; // per-site UE containers
  siteUes.resize(numSites);

  for (uint32_t r = 0; r < ringSites; ++r)
  {
    if (excludedRing.count(r)) continue;
    // site index in enbNodes: center is 0 if present, ring start at (includeCenter?1:0)
    uint32_t siteIdx = (includeCenter ? 1u : 0u) + r;

    NodeContainer group;
    group.Create(uePerSite);
    siteUes[siteIdx] = group;
    ueNodes.Add(group);
  }

  // // UE mobility: RandomWalk2d with speed U(2,4), bounded in [areaMin, areaMax]^2
  // MobilityHelper mUe;
  // mUe.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
  //                     "Bounds", RectangleValue(Rectangle(areaMin, areaMax, areaMin, areaMax)),
  //                     "Speed", StringValue("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),
  //                     "Distance", DoubleValue(5.0)); // step length before direction change

  // // per-site UE position: uniform disc around serving eNB with radius = isd
  // for (uint32_t s = 0; s < numSites; ++s)
  // {
  //   if (siteUes[s].GetN() == 0) continue;

  //   double maxRho = std::min({ isd,
  //                             sitePos[s].x - areaMin,
  //                             areaMax - sitePos[s].x,
  //                             sitePos[s].y - areaMin,
  //                             areaMax - sitePos[s].y });

  //   Ptr<RandomDiscPositionAllocator> disc = CreateObject<RandomDiscPositionAllocator>();
  //   disc->SetAttribute("X", DoubleValue(sitePos[s].x));
  //   disc->SetAttribute("Y", DoubleValue(sitePos[s].y));
  //   disc->SetAttribute("Rho",   StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(maxRho) + "]"));
  //   disc->SetAttribute("Theta", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283185307179586]"));

  //   mUe.SetPositionAllocator(disc);
  //   mUe.Install(siteUes[s]);
  // }

  // ---- Per-UE ConstantVelocity mobility with class-dependent speeds ----
  // RNG
  Ptr<UniformRandomVariable> U = CreateObject<UniformRandomVariable>();

  // speed pickers (m/s)
  auto pickSpeed = [&](const std::string& cls) {
    if (cls == "ped")   return U->GetValue(0.8, 1.6);    // pedestrians
    if (cls == "bike")  return U->GetValue(4.0, 8.0);    // bikes/scooters
    if (cls == "car")   return U->GetValue(15.0, 28.0);  // cars
    if (cls == "bus")   return U->GetValue(8.0, 14.0);   // buses
    return U->GetValue(33.0, 55.0);                      // trains
  };

  // draw mobility class by fractions (CLI knobs you added)
  auto drawClass = [&](){
    double r = U->GetValue(0.0, 1.0);
    if (r < pedFrac)  return std::string("ped");
    r -= pedFrac;
    if (r < bikeFrac) return std::string("bike");
    r -= bikeFrac;
    if (r < carFrac)  return std::string("car");
    r -= carFrac;
    if (r < busFrac)  return std::string("bus");
    return std::string("train");
  };
  

  // 1) Drop UEs per site (uniform disc clipped to [areaMin,areaMax]) and install ConstantVelocity
  MobilityHelper mUe;
  for (uint32_t s = 0; s < enbNodes.GetN(); ++s)
  {
    if (siteUes[s].GetN() == 0) continue;

    double maxRho = std::min({ isd,
                              sitePos[s].x - areaMin,
                              areaMax - sitePos[s].x,
                              sitePos[s].y - areaMin,
                              areaMax - sitePos[s].y });

    Ptr<RandomDiscPositionAllocator> disc = CreateObject<RandomDiscPositionAllocator>();
    disc->SetAttribute("X", DoubleValue(sitePos[s].x));
    disc->SetAttribute("Y", DoubleValue(sitePos[s].y));
    disc->SetAttribute("Rho",   StringValue("ns3::UniformRandomVariable[Min=0.0|Max=" + std::to_string(maxRho) + "]"));
    disc->SetAttribute("Theta", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=6.283185307179586]"));

    mUe.SetPositionAllocator(disc);
    mUe.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mUe.Install(siteUes[s]);
  }

  // 2) Assign velocities per UE (random heading or toward the next site)
  for (uint32_t s = 0; s < enbNodes.GetN(); ++s)
  {
    if (siteUes[s].GetN() == 0) continue;

    uint32_t nextSite = (s + 1) % enbNodes.GetN();   // neighbor site index
    Vector nextPos = sitePos[nextSite];

    for (uint32_t j = 0; j < siteUes[s].GetN(); ++j)
    {
      Ptr<ConstantVelocityMobilityModel> cv =
        siteUes[s].Get(j)->GetObject<ConstantVelocityMobilityModel>();

      std::string cls = drawClass();
      double v = pickSpeed(cls);

      double theta;
      if (mobilityMode == "next-site")
      {
        Vector p = cv->GetPosition();
        theta = std::atan2(nextPos.y - p.y, nextPos.x - p.x); // head toward neighbor site
      }
      else
      {
        theta = U->GetValue(0.0, 2.0 * M_PI);                 // random heading
      }

      cv->SetVelocity(Vector(v * std::cos(theta), v * std::sin(theta), 0.0));
    }
  }

  // (Optional) simple boundary bounce so UEs stay in [areaMin,areaMax]^2
  auto BounceIfOut = [=](Ptr<ConstantVelocityMobilityModel> cv){
    Vector p = cv->GetPosition(), v = cv->GetVelocity();
    bool changed = false;
    if (p.x < areaMin || p.x > areaMax) { v.x = -v.x; changed = true; }
    if (p.y < areaMin || p.y > areaMax) { v.y = -v.y; changed = true; }
    if (changed) cv->SetVelocity(v);
  };
  for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
  {
    Ptr<ConstantVelocityMobilityModel> cv = ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
    Simulator::Schedule(Seconds(1.0), [cv, BounceIfOut](){
      // periodic check every second
      for (uint32_t k=0; k<100000; ++k) { /* no-op to appease capture */ }
      BounceIfOut(cv);
      Simulator::Schedule(Seconds(1.0), [cv, BounceIfOut](){ BounceIfOut(cv); });
    });
  }

  // --------------------- Devices ---------------------
  NetDeviceContainer enbDevs = lte->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs = lte->InstallUeDevice (ueNodes);

  // ---- Sanity check: all eNBs share same DlBandwidth (and optionally DlEarfcn) ----
  UintegerValue rb0, ear0;
  auto enb0 = DynamicCast<LteEnbNetDevice>(enbDevs.Get(0));
  enb0->GetAttribute("DlBandwidth", rb0);

  // Check if this build exposes DlEarfcn on LteEnbNetDevice
  bool haveEar = false;
  TypeId::AttributeInformation info;
  TypeId tid0 = enb0->GetInstanceTypeId();
  if (tid0.LookupAttributeByName("DlEarfcn", &info))
  {
    haveEar = true;
    enb0->GetAttribute("DlEarfcn", ear0);
  }

  for (uint32_t i = 1; i < enbDevs.GetN(); ++i)
  {
    auto enb = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
    UintegerValue rb, ear;

    enb->GetAttribute("DlBandwidth", rb);
    NS_ABORT_MSG_UNLESS(rb.Get() == rb0.Get(),
                        "eNB " << i << " RB=" << rb.Get() << " != RB0=" << rb0.Get());

    if (haveEar)
    {
      enb->GetAttribute("DlEarfcn", ear);
      NS_ABORT_MSG_UNLESS(ear.Get() == ear0.Get(),
                          "eNB " << i << " EARFCN=" << ear.Get() << " != EAR0=" << ear0.Get());
    }
  }

  // Sanity check again
  for (uint32_t i = 0; i < enbDevs.GetN(); ++i) {
    Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
    UintegerValue dlbw, ulbw, dlearf, ulearf;
    enb->GetAttribute("DlBandwidth", dlbw);
    enb->GetAttribute("UlBandwidth", ulbw);
    enb->GetAttribute("DlEarfcn",    dlearf);
    enb->GetAttribute("UlEarfcn",    ulearf);
    NS_LOG_UNCOND("eNB["<<i<<"] RBs DL/UL="<< (uint32_t)dlbw.Get()<<" / "<<(uint32_t)ulbw.Get()
                  <<" EARFCN DL/UL="<< (uint32_t)dlearf.Get()<<" / "<<(uint32_t)ulearf.Get());
  }
  
  for (uint32_t i = 0; i < ueDevs.GetN(); ++i) {
    Ptr<LteUeNetDevice> ue = DynamicCast<LteUeNetDevice>(ueDevs.Get(i));
    UintegerValue dlearf;
    ue->GetAttribute("DlEarfcn", dlearf);
    NS_LOG_UNCOND("UE["<<i<<"] DlEarfcn="<< (uint32_t)dlearf.Get());
  }
  


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

  // Attach UEs to their serving eNB (one site at a time)
  uint32_t ueDevOffset = 0;
  for (uint32_t s = 0; s < enbNodes.GetN(); ++s)
  {
    uint32_t nThis = siteUes[s].GetN();
    if (nThis == 0) continue;

    for (uint32_t j = 0; j < nThis; ++j)
    {
      // find the j-th UE device in global ueDevs: we rely on creation order == container order
      Ptr<NetDevice> ueNd = ueDevs.Get(ueDevOffset + j);
      lte->Attach(ueNd, enbDevs.Get(s));
    }
    ueDevOffset += nThis;
  }


  // X2 between eNBs
  lte->AddX2Interface (enbNodes);

  // --------------------- Traffic ---------------------
  if (usePaperMix)
  {
    ApplicationContainer ueApps, remoteApps;
    uint16_t portBase = 20000;
    uint32_t N = ueNodes.GetN();
    NS_ABORT_MSG_IF(N == 0, "No UEs to apply paper mix");

    // 25% TCP full buffer @ 20 Mbps (approx: BulkSend over TCP)
    uint32_t nTcpFull = N / 4;
    // 25% UDP bursty ~20 Mbps (use OnOff with duty=0.5 and rate=40 Mbps)
    uint32_t nUdpBurst = N / 4;
    // 25% TCP bursty ~750 kbps (duty=0.5, rate=1.5 Mbps)
    uint32_t nTcpBurst750 = N / 4;
    // remaining UEs TCP bursty ~150 kbps (duty=0.5, rate=300 kbps)
    // uint32_t nTcpBurst150 = N - nTcpFull - nUdpBurst - nTcpBurst750;

    uint32_t idx = 0;

    auto addTcpDownlink = [&](Ptr<Node> ue, Ipv4Address ueAddr, uint16_t port, bool bursty, std::string rate, double onMean, double offMean)
    {
      // UE sink (TCP)
      PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
      ApplicationContainer sink = sinkHelper.Install(ue);
      sink.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxSinkTrace));
      ueApps.Add(sink);

      if (!bursty)
      {
        // Full buffer: BulkSend on remote
        BulkSendHelper bulk("ns3::TcpSocketFactory", InetSocketAddress(ueAddr, port));
        bulk.SetAttribute("MaxBytes", UintegerValue(0));
        ApplicationContainer a = bulk.Install(remoteHost);
        remoteApps.Add(a);
      }
      else
      {
        // TCP bursty via OnOff over TCP socket
        Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication>();
        onoff->SetAttribute("Remote", AddressValue(InetSocketAddress(ueAddr, port)));
        onoff->SetAttribute("DataRate", DataRateValue(DataRate(rate)));
        onoff->SetAttribute("PacketSize", UintegerValue(1200)); // typical TCP payload
        onoff->SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=" + std::to_string(onMean)  + "]"));
        onoff->SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=" + std::to_string(offMean) + "]"));
        onoff->SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
        remoteHost->AddApplication(onoff);
        remoteApps.Add(onoff);
      }
    };

    auto addUdpDownlink = [&](Ptr<Node> ue, Ipv4Address ueAddr, uint16_t port, std::string onRate, uint32_t pkt, double onMean, double offMean)
    {
      // UE sink (UDP)
      PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
      ApplicationContainer sink = sinkHelper.Install(ue);
      sink.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxSinkTrace));
      ueApps.Add(sink);

      Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication>();
      onoff->SetAttribute("Remote", AddressValue(InetSocketAddress(ueAddr, port)));
      onoff->SetAttribute("DataRate", DataRateValue(DataRate(onRate)));
      onoff->SetAttribute("PacketSize", UintegerValue(pkt));
      onoff->SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=" + std::to_string(onMean)  + "]"));
      onoff->SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=" + std::to_string(offMean) + "]"));
      // default Protocol is UDP
      remoteHost->AddApplication(onoff);
      remoteApps.Add(onoff);
    };

    // Helper to get each UE's IP (same order as ueNodes)
    auto ueAddrOf = [&](uint32_t k) { return ueIfaces.GetAddress(k); };

    // 25% TCP full-buffer @ 20 Mb/s (we cap via TCP pacing with on/off off; simplest is BulkSend -> PHY/MAC will bound)
    for (uint32_t c = 0; c < nTcpFull; ++c, ++idx)
    {
      addTcpDownlink(ueNodes.Get(idx), ueAddrOf(idx), portBase + idx, /*bursty=*/false, "20Mbps", 0, 0);
    }

    // 25% UDP bursty ~20 Mb/s average: duty ~0.5, ON rate = 40 Mb/s, pkt=1200B
    for (uint32_t c = 0; c < nUdpBurst; ++c, ++idx)
    {
      addUdpDownlink(ueNodes.Get(idx), ueAddrOf(idx), portBase + idx, "40Mbps", 1200, /*on*/0.5, /*off*/0.5);
    }

    // 25% TCP bursty ~750 kb/s average: duty ~0.5, ON rate = 1.5 Mb/s, pkt ~ 800–1200B
    for (uint32_t c = 0; c < nTcpBurst750; ++c, ++idx)
    {
      addTcpDownlink(ueNodes.Get(idx), ueAddrOf(idx), portBase + idx, /*bursty=*/true, "1500kbps", /*on*/0.5, /*off*/0.5);
    }

    // remainder TCP bursty ~150 kb/s average: duty ~0.5, ON rate = 300 kb/s
    for (; idx < N; ++idx)
    {
      addTcpDownlink(ueNodes.Get(idx), ueAddrOf(idx), portBase + idx, /*bursty=*/true, "300kbps", /*on*/0.5, /*off*/0.5);
    }

    ueApps.Start (Seconds (1.0));
    ueApps.Stop  (simTime - Seconds (0.5));
    remoteApps.Start (Seconds (2.0));
    remoteApps.Stop  (simTime - Seconds (1.0));
  }
  else
  {
    // keep your existing traffic profiles (eMBB/URLLC/V2X/mMTC/mixed) block here
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

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
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
  }


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

