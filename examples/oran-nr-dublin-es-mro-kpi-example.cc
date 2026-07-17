/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-dublin-es-mro-kpi-example.cc.
 *
 * Uses the same OpenCellID-derived Dublin gNB positions, places UEs around
 * those sites, runs the RSRP handover MRO xApp and the Energy Saving xApp,
 * and lets OranCmmNr2NrEsMro mediate ES TxPower reductions -- on a sub-6GHz
 * NR band instead of LTE.
 *
 * CSV outputs are written to:
 *   /workspace/results/nr-dublin-es-mro/<method>/kpis.csv
 *   /workspace/results/nr-dublin-es-mro/<method>/enb-kpis.csv
 *   /workspace/results/nr-dublin-es-mro/<method>/positions.csv
 *
 * Note: the LTE original tracks avg SINR via LteUePhy's
 * "ReportCurrentCellRsrpSinr" trace. NR uses NrUePhy's "DlDataSinr"
 * trace for SINR and "ReportUeMeasurements" for RSRP/RSRQ.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-cmm-nr-2-nr-es-mro.h"
#include "ns3/oran-module.h"
#include "ns3/oran-nr-cell-control-state.h"
#include "ns3/oran-nr-ru-energy-model.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrDublinEsMroKpiExample");

static uint64_t g_totalRxBytes = 0;
static uint64_t g_lastRxBytes = 0;
static uint32_t g_handoverOk = 0;
static uint32_t g_handoverFailures = 0;
static uint32_t g_pingPongHandovers = 0;
static uint32_t g_radioLinkFailures = 0;
static uint32_t g_connectionTimeouts = 0;
static double g_rsrpSum = 0.0;
static double g_minRsrp = std::numeric_limits<double>::infinity();
static uint64_t g_rsrpSamples = 0;
static double g_sinrDbSum = 0.0;
static uint64_t g_sinrSamples = 0;
static std::ofstream g_kpiOut;
static std::ofstream g_enbOut;
static std::ofstream g_posOut;
static std::ofstream g_cellParamOut;
static std::ofstream g_eventOut;

struct LastHoTransition
{
    uint16_t sourceCellId = 0;
    uint16_t targetCellId = 0;
    double timeSec = 0.0;
};

static std::map<uint64_t, LastHoTransition> g_lastHoTransitionByImsi;
static double g_pingPongWindowSec = 5.0;

static void
EnsureDir(const std::string& path)
{
    if (::mkdir(path.c_str(), 0755) != 0 && errno != EEXIST)
    {
        NS_ABORT_MSG("Could not create directory " << path << " errno=" << errno);
    }
}

static std::vector<Vector>
LoadEnbPositionsFromVectorFile(const std::string& path)
{
    std::ifstream in(path.c_str());
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open gNB position file: " << path);

    std::vector<Vector> positions;
    std::string line;
    std::regex rx(
        R"(Vector\s*\(\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\s*,\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?))");
    std::smatch match;
    while (std::getline(in, line))
    {
        if (std::regex_search(line, match, rx) && match.size() >= 4)
        {
            positions.emplace_back(std::stod(match[1].str()),
                                   std::stod(match[2].str()),
                                   std::stod(match[3].str()));
        }
    }

    NS_ABORT_MSG_IF(positions.empty(), "No Vector(x,y,z) gNB positions found in " << path);
    return positions;
}

static std::pair<uint32_t, uint32_t>
BestTwoEnbIdx(const Vector& uePos, const NodeContainer& gnbNodes)
{
    double bestD2 = std::numeric_limits<double>::infinity();
    double secondD2 = std::numeric_limits<double>::infinity();
    uint32_t best = 0;
    uint32_t second = 0;

    for (uint32_t j = 0; j < gnbNodes.GetN(); ++j)
    {
        Vector gnbPos = gnbNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
        const double dx = uePos.x - gnbPos.x;
        const double dy = uePos.y - gnbPos.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < bestD2)
        {
            secondD2 = bestD2;
            second = best;
            bestD2 = d2;
            best = j;
        }
        else if (d2 < secondD2)
        {
            secondD2 = d2;
            second = j;
        }
    }

    return std::make_pair(best, second);
}

static void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (mob)
        {
            Vector v = mob->GetVelocity();
            mob->SetVelocity(Vector(-v.x, -v.y, 0.0));
        }
    }
    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

static void
RxTrace(Ptr<const Packet> packet, const Address& from, const Address& to)
{
    g_totalRxBytes += packet->GetSize();
}

// Track serving-cell RSRP via NrUePhy's "ReportUeMeasurements" trace
// (rnti, cellId, rsrp, rsrq, isServingCell, ccId) -- confirmed identical
// signature to LteUePhy's trace of the same name.
static void
ObserveRsrpForKpis(uint16_t rnti, uint16_t cellId, double rsrp, double rsrq, bool serving, uint8_t)
{
    (void)rnti;
    (void)cellId;
    (void)rsrq;
    if (!serving)
        return;
    g_rsrpSum += rsrp;
    g_minRsrp = std::min(g_minRsrp, rsrp);
    g_rsrpSamples++;
}

static void
ObserveSinrForKpis(uint16_t cellId, uint16_t rnti, double avgSinrLinear, uint16_t bwpId)
{
    (void)cellId;
    (void)rnti;
    (void)bwpId;
    if (!std::isfinite(avgSinrLinear) || avgSinrLinear <= 0.0)
    {
        return;
    }
    g_sinrDbSum += 10.0 * std::log10(avgSinrLinear);
    g_sinrSamples++;
}

static void
NotifyHandoverEndOk(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    g_handoverOk++;
    if (g_eventOut.is_open())
    {
        g_eventOut << Simulator::Now().GetSeconds() << ",handover_end_ok," << context << ","
                   << imsi << "," << cellId << "," << rnti << ",0,0\n";
    }
}

static void
NotifyHandoverFailure(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    g_handoverFailures++;
    if (g_eventOut.is_open())
    {
        g_eventOut << Simulator::Now().GetSeconds() << ",handover_failure," << context << ","
                   << imsi << "," << cellId << "," << rnti << ",0,0\n";
    }
}

static void
NotifyUeHandoverStart(std::string context,
                      uint64_t imsi,
                      uint16_t sourceCellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    const double now = Simulator::Now().GetSeconds();
    auto it = g_lastHoTransitionByImsi.find(imsi);
    if (it != g_lastHoTransitionByImsi.end() && it->second.sourceCellId == targetCellId &&
        it->second.targetCellId == sourceCellId && (now - it->second.timeSec) <= g_pingPongWindowSec)
    {
        g_pingPongHandovers++;
        if (g_eventOut.is_open())
        {
            g_eventOut << now << ",ping_pong," << context << "," << imsi << "," << sourceCellId
                       << "," << rnti << "," << targetCellId << "," << (now - it->second.timeSec)
                       << "\n";
        }
    }

    g_lastHoTransitionByImsi[imsi] = {sourceCellId, targetCellId, now};
    if (g_eventOut.is_open())
    {
        g_eventOut << now << ",handover_start," << context << "," << imsi << "," << sourceCellId
                   << "," << rnti << "," << targetCellId << ",0\n";
    }
}

static void
NotifyUeHandoverEndError(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    g_handoverFailures++;
    if (g_eventOut.is_open())
    {
        g_eventOut << Simulator::Now().GetSeconds() << ",handover_end_error," << context << ","
                   << imsi << "," << cellId << "," << rnti << ",0,0\n";
    }
}

static void
NotifyRadioLinkFailure(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    g_radioLinkFailures++;
    if (g_eventOut.is_open())
    {
        g_eventOut << Simulator::Now().GetSeconds() << ",radio_link_failure," << context << ","
                   << imsi << "," << cellId << "," << rnti << ",0,0\n";
    }
}

static void
NotifyConnectionTimeout(std::string context,
                        uint64_t imsi,
                        uint16_t cellId,
                        uint16_t rnti,
                        uint8_t numberOfAttempts)
{
    g_connectionTimeouts++;
    if (g_eventOut.is_open())
    {
        g_eventOut << Simulator::Now().GetSeconds() << ",connection_timeout," << context << ","
                   << imsi << "," << cellId << "," << rnti << ",0,"
                   << static_cast<uint32_t>(numberOfAttempts) << "\n";
    }
}

static void
WritePositionSnapshot(Time interval, Time stopTime, NodeContainer gnbNodes, NodeContainer ueNodes)
{
    const double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Vector p = gnbNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_posOut << now << ",gNB," << i << "," << p.x << "," << p.y << "," << p.z << "\n";
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Vector p = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_posOut << now << ",UE," << i << "," << p.x << "," << p.y << "," << p.z << "\n";
    }
    g_posOut.flush();

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &WritePositionSnapshot, interval, stopTime, gnbNodes, ueNodes);
    }
}

static void
SampleKpis(Time interval,
           Time stopTime,
           NetDeviceContainer gnbNrDevs,
           std::vector<Ptr<OranNrRuDeviceEnergyModel>> energyModels)
{
    const double now = Simulator::Now().GetSeconds();
    const uint64_t deltaBytes = g_totalRxBytes - g_lastRxBytes;
    g_lastRxBytes = g_totalRxBytes;

    double totalEnergyJ = 0.0;
    double avgTxPower = 0.0;
    for (uint32_t i = 0; i < gnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNrDevs.Get(i));
        const double txPower = gnb->GetPhy(0)->GetTxPower();
        const double energyJ = energyModels[i]->GetTotalEnergyConsumption();
        avgTxPower += txPower;
        totalEnergyJ += energyJ;
        g_enbOut << now << "," << i << "," << txPower << "," << energyJ << "\n";
    }
    avgTxPower /= std::max<uint32_t>(gnbNrDevs.GetN(), 1);

    const double intervalMbps = (deltaBytes * 8.0) / interval.GetSeconds() / 1e6;
    const double cumulativeMbps = (now > 0.0) ? (g_totalRxBytes * 8.0) / now / 1e6 : 0.0;
    const double energyEfficiency = (g_totalRxBytes * 8.0) / std::max(totalEnergyJ, 1e-12);
    const double avgRsrp =
        g_rsrpSamples > 0 ? g_rsrpSum / static_cast<double>(g_rsrpSamples)
                          : std::numeric_limits<double>::quiet_NaN();
    const double minRsrp =
        g_rsrpSamples > 0 ? g_minRsrp : std::numeric_limits<double>::quiet_NaN();
    const double avgSinrDb =
        g_sinrSamples > 0 ? g_sinrDbSum / static_cast<double>(g_sinrSamples)
                          : std::numeric_limits<double>::quiet_NaN();

    g_kpiOut << now << "," << intervalMbps << "," << cumulativeMbps << "," << g_totalRxBytes << ","
             << totalEnergyJ << "," << energyEfficiency << "," << avgTxPower << "," << avgRsrp
             << "," << minRsrp << "," << avgSinrDb << "," << g_rsrpSamples << "," << g_handoverOk
             << "," << g_handoverFailures << "," << g_pingPongHandovers << ","
             << g_radioLinkFailures << "," << g_connectionTimeouts << "\n";

    g_kpiOut.flush();
    g_enbOut.flush();

    g_rsrpSum = 0.0;
    g_minRsrp = std::numeric_limits<double>::infinity();
    g_rsrpSamples = 0;
    g_sinrDbSum = 0.0;
    g_sinrSamples = 0;

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &SampleKpis, interval, stopTime, gnbNrDevs, energyModels);
    }
}

static void
SampleCellControlParams(Time interval,
                        Time stopTime,
                        NetDeviceContainer gnbNrDevs,
                        OranE2NodeTerminatorContainer gnbTerminators)
{
    const double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < gnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNrDevs.Get(i));
        Ptr<OranE2NodeTerminator> terminator = gnbTerminators.Get(i);
        const uint64_t e2NodeId = terminator->GetE2NodeId();
        const OranNrCellControlParams params = GetNrCellControlParameters(e2NodeId);

        g_cellParamOut << now << "," << i << "," << e2NodeId << "," << gnb->GetCellId() << ","
                       << gnb->GetPhy(0)->GetTxPower() << "," << params.cioDb << ","
                       << params.tttSec << "," << params.hysDb << "," << params.retDeg << "\n";
    }
    g_cellParamOut.flush();

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval,
                            &SampleCellControlParams,
                            interval,
                            stopTime,
                            gnbNrDevs,
                            gnbTerminators);
    }
}

int
main(int argc, char* argv[])
{
    std::string enbPosFile = "/workspace/data/ns3_positions_Three_IE.txt";
    std::string resultRoot = "/workspace/results/nr-dublin-es-mro";
    std::string mitigationMethod = "qacm";
    uint32_t targetUes = 117;
    double ueDiscR = 120.0;
    double txPower = 30.0;
    double targetPowerW = 50.0;
    double stepSizeDb = 1.0;
    Time simTime = Seconds(45);
    Time sampleInterval = Seconds(1);
    Time positionSampleInterval = Seconds(5);
    Time reverseInterval = Seconds(15);
    Time ricStartTime = Seconds(10);
    Time lmQueryInterval = Seconds(5);
    bool enableCco = false;
    bool enableMlb = false;
    bool mlbControlTtt = false;
    double pingPongWindowSec = 5.0;
    double ccoLowRsrpThresholdDbm = -110.0;
    double ccoLowRsrpFractionThreshold = 0.10;
    double mlbLoadImbalanceThreshold = 0.20;
    double mlbCioStepDb = 1.0;
    std::string dbFileName = "oran-nr-dublin-es-mro.db";

    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("enbPosFile", "Dublin gNB position file", enbPosFile);
    cmd.AddValue("resultRoot", "Root directory for result subdirectories", resultRoot);
    cmd.AddValue("mitigation-method",
                 "CMM method: none, cancel, dampen, priority, nswf, eg, qacm",
                 mitigationMethod);
    cmd.AddValue("targetUes", "Total number of UEs distributed around Dublin sites", targetUes);
    cmd.AddValue("ueDiscR", "UE placement radius around each gNB", ueDiscR);
    cmd.AddValue("txPower", "Initial gNB TxPower in dBm", txPower);
    cmd.AddValue("target-power-w", "ES target power per gNB in W", targetPowerW);
    cmd.AddValue("step-size-db", "ES TxPower step per decision", stepSizeDb);
    cmd.AddValue("sim-time", "Simulation duration", simTime);
    cmd.AddValue("sample-interval", "KPI sampling interval", sampleInterval);
    cmd.AddValue("ric-start-time", "Time to activate the Near-RT RIC", ricStartTime);
    cmd.AddValue("pingPongWindowSec", "A-B-A handover window counted as ping-pong", pingPongWindowSec);
    cmd.AddValue("enableCco", "Enable coverage/capacity optimization xApp", enableCco);
    cmd.AddValue("enableMlb", "Enable mobility load balancing xApp", enableMlb);
    cmd.AddValue("ccoLowRsrpThresholdDbm", "CCO low-RSRP threshold in dBm", ccoLowRsrpThresholdDbm);
    cmd.AddValue("ccoLowRsrpFractionThreshold",
                 "CCO fraction of low-RSRP UEs needed before increasing TxPower",
                 ccoLowRsrpFractionThreshold);
    cmd.AddValue("mlbLoadImbalanceThreshold", "MLB hot/cold cell load imbalance threshold", mlbLoadImbalanceThreshold);
    cmd.AddValue("mlbCioStepDb", "MLB CIO adjustment step in dB", mlbCioStepDb);
    cmd.AddValue("mlbControlTtt", "Let MLB also tune TTT on hot cells", mlbControlTtt);
    cmd.Parse(argc, argv);

    g_pingPongWindowSec = pingPongWindowSec;
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    EnsureDir("/workspace/results");
    EnsureDir(resultRoot);
    const std::string resultDir = resultRoot + "/" + mitigationMethod;
    EnsureDir(resultDir);

    const std::string kpiPath = resultDir + "/kpis.csv";
    const std::string enbPath = resultDir + "/enb-kpis.csv";
    const std::string posPath = resultDir + "/positions.csv";
    const std::string cellParamPath = resultDir + "/cell-params.csv";
    const std::string eventPath = resultDir + "/events.csv";
    std::remove(kpiPath.c_str());
    std::remove(enbPath.c_str());
    std::remove(posPath.c_str());
    std::remove(cellParamPath.c_str());
    std::remove(eventPath.c_str());
    std::remove(dbFileName.c_str());

    g_kpiOut.open(kpiPath.c_str());
    g_enbOut.open(enbPath.c_str());
    g_posOut.open(posPath.c_str());
    g_cellParamOut.open(cellParamPath.c_str());
    g_eventOut.open(eventPath.c_str());
    NS_ABORT_MSG_IF(!g_kpiOut.is_open(), "Cannot open KPI output " << kpiPath);
    NS_ABORT_MSG_IF(!g_enbOut.is_open(), "Cannot open gNB KPI output " << enbPath);
    NS_ABORT_MSG_IF(!g_posOut.is_open(), "Cannot open position output " << posPath);
    NS_ABORT_MSG_IF(!g_cellParamOut.is_open(),
                    "Cannot open cell-parameter output " << cellParamPath);
    NS_ABORT_MSG_IF(!g_eventOut.is_open(), "Cannot open event output " << eventPath);

    g_kpiOut << "time_s,interval_throughput_mbps,cumulative_throughput_mbps,total_rx_bytes,"
             << "total_energy_j,energy_efficiency_bits_per_j,avg_txpower_dbm,avg_rsrp_dbm,"
             << "min_rsrp_dbm,avg_sinr_db,rsrp_samples,handover_ok,handover_failures,"
             << "ping_pong_handovers,radio_link_failures,connection_timeouts\n";
    g_enbOut << "time_s,gnb_index,txpower_dbm,energy_j\n";
    g_posOut << "time_s,node_type,node_index,x_m,y_m,z_m\n";
    g_cellParamOut << "time_s,gnb_index,e2_node_id,cell_id,txpower_dbm,cio_db,ttt_s,hys_db,"
                   << "ret_deg\n";
    g_eventOut << "time_s,event_type,context,imsi,cell_id,rnti,target_cell_id,value\n";

    std::vector<Vector> siteCenters = LoadEnbPositionsFromVectorFile(enbPosFile);
    const uint32_t numberOfGnbs = siteCenters.size();
    const uint32_t numberOfUes = targetUes;

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   1,
                                                   BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> gnbPosAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& p : siteCenters)
    {
        gnbPosAlloc->Add(p);
    }
    MobilityHelper gnbMobility;
    gnbMobility.SetPositionAllocator(gnbPosAlloc);
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);

    Ptr<UniformRandomVariable> uniform = CreateObject<UniformRandomVariable>();
    Ptr<ListPositionAllocator> uePosAlloc = CreateObject<ListPositionAllocator>();
    const double twoPi = 6.283185307179586;
    for (uint32_t i = 0; i < numberOfUes; ++i)
    {
        const Vector c = siteCenters[i % siteCenters.size()];
        const double r = ueDiscR * std::sqrt(uniform->GetValue());
        const double theta = twoPi * uniform->GetValue();
        uePosAlloc->Add(Vector(c.x + r * std::cos(theta), c.y + r * std::sin(theta), 1.5));
    }

    MobilityHelper ueMobility;
    ueMobility.SetPositionAllocator(uePosAlloc);
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        const double speed = (i % 5 == 0) ? 13.0 : ((i % 3 == 0) ? 5.5 : 1.5);
        const double theta = twoPi * uniform->GetValue();
        mob->SetVelocity(Vector(speed * std::cos(theta), speed * std::sin(theta), 0.0));
    }
    Simulator::Schedule(reverseInterval, &ReverseVelocity, ueNodes, reverseInterval);

    NetDeviceContainer gnbNrDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNrDevs, randomStream);

    for (uint32_t i = 0; i < gnbNrDevs.GetN(); i++)
    {
        nrHelper->GetGnbPhy(gnbNrDevs.Get(i), 0)->SetAttribute("Numerology",
                                                               UintegerValue(numerology));
        nrHelper->GetGnbPhy(gnbNrDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(txPower));
    }
    NetDeviceContainer ueNrDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    randomStream += nrHelper->AssignStreams(ueNrDevs, randomStream);

    nrHelper->UpdateDeviceConfigs(gnbNrDevs);
    nrHelper->UpdateDeviceConfigs(ueNrDevs);
    nrHelper->AddX2Interface(gnbNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNrDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Vector p = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        auto bestTwo = BestTwoEnbIdx(p, gnbNodes);
        const uint32_t serving = (i % 4 == 0 && numberOfGnbs > 1) ? bestTwo.second : bestTwo.first;
        nrHelper->AttachToGnb(ueNrDevs.Get(i), gnbNrDevs.Get(serving));
    }

    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(5000000.0));
    sourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    energy::EnergySourceContainer sources = sourceHelper.Install(gnbNodes);
    std::vector<Ptr<OranNrRuDeviceEnergyModel>> gnbEnergyModels;
    std::vector<Ptr<energy::BasicEnergySource>> gnbEnergySources;
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNrDevs.Get(i));
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(sources.Get(i));
        Ptr<OranNrRuDeviceEnergyModel> dem = CreateObject<OranNrRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetNrGnbPhy(gnb->GetPhy(0));

        Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
        ru->SetAttribute("NumTrx", UintegerValue(4));
        ru->SetAttribute("EtaPA", DoubleValue(0.30));
        ru->SetAttribute("FixedOverheadW", DoubleValue(5.0));
        ru->SetAttribute("DeltaAf", DoubleValue(0.5));
        ru->SetAttribute("DeltaDC", DoubleValue(0.07));
        ru->SetAttribute("DeltaMS", DoubleValue(0.09));
        ru->SetAttribute("DeltaCool", DoubleValue(0.10));
        ru->SetAttribute("Vdc", DoubleValue(48.0));
        ru->SetAttribute("SleepPowerW", DoubleValue(2.0));
        ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));

        src->AppendDeviceEnergyModel(dem);
        gnbEnergyModels.push_back(dem);
        gnbEnergySources.push_back(src);
    }

    uint16_t basePort = 10000;
    ApplicationContainer ueApps;
    ApplicationContainer remoteApps;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const uint16_t port = basePort + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sinkHelper.Install(ueNodes.Get(i));
        sinkApp.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));
        ueApps.Add(sinkApp);

        Ptr<OnOffApplication> onoff = CreateObject<OnOffApplication>();
        onoff->SetAttribute("Remote",
                            AddressValue(InetSocketAddress(ueIpIfaces.GetAddress(i), port)));
        onoff->SetAttribute("DataRate", DataRateValue(DataRate((i % 4 == 0) ? "2Mbps" : "1Mbps")));
        onoff->SetAttribute("PacketSize", UintegerValue(1200));
        onoff->SetAttribute("OnTime", StringValue("ns3::ExponentialRandomVariable[Mean=0.6]"));
        onoff->SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=1.0]"));
        remoteHost->AddApplication(onoff);
        remoteApps.Add(onoff);

        nrHelper->ActivateDedicatedEpsBearer(ueNrDevs.Get(i),
                                             NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
                                             Create<NrEpcTft>());
    }
    ueApps.Start(Seconds(1.0));
    ueApps.Stop(simTime);
    remoteApps.Start(Seconds(2.0));
    remoteApps.Stop(simTime - Seconds(0.5));

    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrRsrpHandover");
    oranHelper->AddLogicModule("ns3::OranLmNr2NrEnergySaving",
                              "TargetPowerW",
                              DoubleValue(targetPowerW),
                              "StepSize",
                              DoubleValue(stepSizeDb),
                              "LmIntervalSec",
                              DoubleValue(lmQueryInterval.GetSeconds()));
    if (enableCco)
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrCoverageCapacityOptimization",
                                  "LowRsrpThresholdDbm",
                                  DoubleValue(ccoLowRsrpThresholdDbm),
                                  "LowRsrpFractionThreshold",
                                  DoubleValue(ccoLowRsrpFractionThreshold),
                                  "StepSize",
                                  DoubleValue(stepSizeDb));
    }
    if (enableMlb)
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrMobilityLoadBalancing",
                                  "LoadImbalanceThreshold",
                                  DoubleValue(mlbLoadImbalanceThreshold),
                                  "CioStep",
                                  DoubleValue(mlbCioStepDb),
                                  "ControlTtt",
                                  BooleanValue(mlbControlTtt));
    }
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNr2NrEsMro",
                                            "MitigationMethod",
                                            StringValue(mitigationMethod),
                                            "EsPriority",
                                            DoubleValue(0.70),
                                            "MroPriority",
                                            DoubleValue(1.00));
    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    OranE2NodeTerminatorContainer ueTerminators;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> cellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorNrUe> ueTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        cellInfoReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(ueTerminator));

        Ptr<NrUeNetDevice> nrUeDevice = DynamicCast<NrUeNetDevice>(ueNrDevs.Get(i));
        Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(0);
        uePhy->TraceConnectWithoutContext(
            "ReportUeMeasurements",
            MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
        uePhy->TraceConnectWithoutContext("ReportUeMeasurements", MakeCallback(&ObserveRsrpForKpis));
        uePhy->TraceConnectWithoutContext("DlDataSinr", MakeCallback(&ObserveSinrForKpis));

        ueTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerminator->SetAttribute("RegistrationIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerminator->SetAttribute("SendIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerminator->AddReporter(locationReporter);
        ueTerminator->AddReporter(cellInfoReporter);
        ueTerminator->AddReporter(rsrpRsrqReporter);
        ueTerminator->Attach(ueNodes.Get(i));
        ueTerminators.Add(ueTerminator);
        Simulator::Schedule(ricStartTime + Seconds(1.0),
                            &OranE2NodeTerminatorNrUe::Activate,
                            ueTerminator);
    }

    OranE2NodeTerminatorContainer gnbTerminators;
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<OranReporterLocation> locReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrEnergyEfficiency> energyReporter =
            CreateObject<OranReporterNrEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorNrGnb> gnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

        locReporter->SetAttribute("Terminator", PointerValue(gnbTerminator));
        locReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        energyReporter->SetEnergySource(gnbEnergySources[i]);
        energyReporter->SetAttribute("Terminator", PointerValue(gnbTerminator));
        energyReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        gnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        gnbTerminator->SetAttribute("RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        gnbTerminator->SetAttribute("SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=2]"));
        gnbTerminator->AddReporter(locReporter);
        gnbTerminator->AddReporter(energyReporter);
        gnbTerminator->Attach(gnbNodes.Get(i));
        gnbTerminators.Add(gnbTerminator);
        Simulator::Schedule(ricStartTime + Seconds(0.5),
                            &OranE2NodeTerminatorNrGnb::Activate,
                            gnbTerminator);
    }

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOk));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureJoining",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureLeaving",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NotifyUeHandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndError",
                    MakeCallback(&NotifyUeHandoverEndError));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
                    MakeCallback(&NotifyRadioLinkFailure));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionTimeout",
                    MakeCallback(&NotifyConnectionTimeout));

    Simulator::Schedule(ricStartTime, &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(sampleInterval, &SampleKpis, sampleInterval, simTime, gnbNrDevs, gnbEnergyModels);
    Simulator::Schedule(sampleInterval,
                        &SampleCellControlParams,
                        sampleInterval,
                        simTime,
                        gnbNrDevs,
                        gnbTerminators);
    Simulator::Schedule(Seconds(0),
                        &WritePositionSnapshot,
                        positionSampleInterval,
                        simTime,
                        gnbNodes,
                        ueNodes);

    Simulator::Stop(simTime);
    Simulator::Run();

    double totalEnergyJ = 0.0;
    double avgTxPower = 0.0;
    for (uint32_t i = 0; i < gnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbNrDevs.Get(i));
        totalEnergyJ += gnbEnergyModels[i]->GetTotalEnergyConsumption();
        avgTxPower += gnb->GetPhy(0)->GetTxPower();
    }
    avgTxPower /= std::max<uint32_t>(gnbNrDevs.GetN(), 1);
    const double throughputMbps = (g_totalRxBytes * 8.0) / simTime.GetSeconds() / 1e6;
    const double energyEfficiency = (g_totalRxBytes * 8.0) / std::max(totalEnergyJ, 1e-12);

    std::cout << "RESULT: method=" << mitigationMethod << " enbs=" << numberOfGnbs
              << " ues=" << numberOfUes << " throughput_mbps=" << throughputMbps
              << " handover_ok=" << g_handoverOk << " handover_failures=" << g_handoverFailures
              << " ping_pong=" << g_pingPongHandovers
              << " radio_link_failures=" << g_radioLinkFailures
              << " connection_timeouts=" << g_connectionTimeouts
              << " total_energy_j=" << totalEnergyJ
              << " energy_efficiency_bits_per_j=" << energyEfficiency
              << " avg_final_txpower_dbm=" << avgTxPower << " result_dir=" << resultDir
              << std::endl;

    g_kpiOut.close();
    g_enbOut.close();
    g_posOut.close();
    g_cellParamOut.close();
    g_eventOut.close();
    Simulator::Destroy();
    return 0;
}
