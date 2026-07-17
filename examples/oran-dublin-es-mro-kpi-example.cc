/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Dublin-scale ES/MRO conflict mitigation experiment.
 *
 * This example uses the OpenCellID-derived Dublin eNB positions from
 * workspace/data/ns3_positions_Three_IE.txt, places UEs around those sites,
 * runs the RSRP handover MRO xApp and the Energy Saving xApp, and lets
 * OranCmmLte2LteEsMro mediate ES TxPower reductions.
 *
 * CSV outputs are written to:
 *   /workspace/results/dublin-es-mro/<method>/kpis.csv
 *   /workspace/results/dublin-es-mro/<method>/enb-kpis.csv
 *   /workspace/results/dublin-es-mro/<method>/positions.csv
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-lte-cell-control-state.h"
#include "ns3/oran-module.h"
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

NS_LOG_COMPONENT_DEFINE("OranDublinEsMroKpiExample");

static uint64_t g_totalRxBytes = 0;
static uint64_t g_lastRxBytes = 0;
static uint32_t g_handoverOk = 0;
static uint32_t g_handoverFailures = 0;
static uint32_t g_pingPongHandovers = 0;
static uint32_t g_radioLinkFailures = 0;
static uint32_t g_connectionTimeouts = 0;
static double g_rsrpSum = 0.0;
static double g_sinrSum = 0.0;
static double g_minRsrp = std::numeric_limits<double>::infinity();
static uint64_t g_rsrpSamples = 0;
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
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open eNB position file: " << path);

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

    NS_ABORT_MSG_IF(positions.empty(), "No Vector(x,y,z) eNB positions found in " << path);
    return positions;
}

static std::pair<uint32_t, uint32_t>
BestTwoEnbIdx(const Vector& uePos, const NodeContainer& enbNodes)
{
    double bestD2 = std::numeric_limits<double>::infinity();
    double secondD2 = std::numeric_limits<double>::infinity();
    uint32_t best = 0;
    uint32_t second = 0;

    for (uint32_t j = 0; j < enbNodes.GetN(); ++j)
    {
        Vector enbPos = enbNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
        const double dx = uePos.x - enbPos.x;
        const double dy = uePos.y - enbPos.y;
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

static void
CurrentCellRsrpSinrTrace(uint16_t cellId,
                         uint16_t rnti,
                         double rsrp,
                         double sinr,
                         uint8_t componentCarrierId)
{
    const double rsrpDbm = 10.0 * std::log10(std::max(rsrp, 1e-30) * 1000.0);
    g_rsrpSum += rsrpDbm;
    g_sinrSum += 10.0 * std::log10(std::max(sinr, 1e-12));
    g_minRsrp = std::min(g_minRsrp, rsrpDbm);
    g_rsrpSamples++;
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
    if (it != g_lastHoTransitionByImsi.end() &&
        it->second.sourceCellId == targetCellId &&
        it->second.targetCellId == sourceCellId &&
        (now - it->second.timeSec) <= g_pingPongWindowSec)
    {
        g_pingPongHandovers++;
        if (g_eventOut.is_open())
        {
            g_eventOut << now << ",ping_pong," << context << "," << imsi << ","
                       << sourceCellId << "," << rnti << "," << targetCellId << ","
                       << (now - it->second.timeSec) << "\n";
        }
    }

    g_lastHoTransitionByImsi[imsi] = {sourceCellId, targetCellId, now};
    if (g_eventOut.is_open())
    {
        g_eventOut << now << ",handover_start," << context << "," << imsi << ","
                   << sourceCellId << "," << rnti << "," << targetCellId << ",0\n";
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
WritePositionSnapshot(Time interval, Time stopTime, NodeContainer enbNodes, NodeContainer ueNodes)
{
    const double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Vector p = enbNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_posOut << now << ",eNB," << i << "," << p.x << "," << p.y << "," << p.z << "\n";
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Vector p = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        g_posOut << now << ",UE," << i << "," << p.x << "," << p.y << "," << p.z << "\n";
    }
    g_posOut.flush();

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &WritePositionSnapshot, interval, stopTime, enbNodes, ueNodes);
    }
}

static void
SampleKpis(Time interval,
           Time stopTime,
           NetDeviceContainer enbLteDevs,
           std::vector<Ptr<OranRuDeviceEnergyModel>> energyModels)
{
    const double now = Simulator::Now().GetSeconds();
    const uint64_t deltaBytes = g_totalRxBytes - g_lastRxBytes;
    g_lastRxBytes = g_totalRxBytes;

    double totalEnergyJ = 0.0;
    double avgTxPower = 0.0;
    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
    {
        Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        const double txPower = enb->GetPhy()->GetTxPower();
        const double energyJ = energyModels[i]->GetTotalEnergyConsumption();
        avgTxPower += txPower;
        totalEnergyJ += energyJ;
        g_enbOut << now << "," << i << "," << txPower << "," << energyJ << "\n";
    }
    avgTxPower /= std::max<uint32_t>(enbLteDevs.GetN(), 1);

    const double intervalMbps = (deltaBytes * 8.0) / interval.GetSeconds() / 1e6;
    const double cumulativeMbps = (now > 0.0) ? (g_totalRxBytes * 8.0) / now / 1e6 : 0.0;
    const double energyEfficiency = (g_totalRxBytes * 8.0) / std::max(totalEnergyJ, 1e-12);
    const double avgRsrp = g_rsrpSamples > 0
                               ? g_rsrpSum / static_cast<double>(g_rsrpSamples)
                               : std::numeric_limits<double>::quiet_NaN();
    const double minRsrp = g_rsrpSamples > 0 ? g_minRsrp
                                             : std::numeric_limits<double>::quiet_NaN();
    const double avgSinrDb = g_rsrpSamples > 0
                                 ? g_sinrSum / static_cast<double>(g_rsrpSamples)
                                 : std::numeric_limits<double>::quiet_NaN();

    g_kpiOut << now << "," << intervalMbps << "," << cumulativeMbps << "," << g_totalRxBytes
             << "," << totalEnergyJ << "," << energyEfficiency << "," << avgTxPower << ","
             << avgRsrp << "," << minRsrp << "," << avgSinrDb << "," << g_rsrpSamples << ","
             << g_handoverOk << "," << g_handoverFailures << "," << g_pingPongHandovers << ","
             << g_radioLinkFailures << "," << g_connectionTimeouts << "\n";

    g_kpiOut.flush();
    g_enbOut.flush();

    g_rsrpSum = 0.0;
    g_sinrSum = 0.0;
    g_minRsrp = std::numeric_limits<double>::infinity();
    g_rsrpSamples = 0;

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &SampleKpis, interval, stopTime, enbLteDevs, energyModels);
    }
}

static void
SampleCellControlParams(Time interval,
                        Time stopTime,
                        NetDeviceContainer enbLteDevs,
                        OranE2NodeTerminatorContainer enbTerminators)
{
    const double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
    {
        Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        Ptr<OranE2NodeTerminator> terminator = enbTerminators.Get(i);
        const uint64_t e2NodeId = terminator->GetE2NodeId();
        const OranLteCellControlParams params = GetLteCellControlParameters(e2NodeId);

        g_cellParamOut << now << "," << i << "," << e2NodeId << "," << enb->GetCellId() << ","
                       << enb->GetPhy()->GetTxPower() << "," << params.cioDb << ","
                       << params.tttSec << "," << params.hysDb << "," << params.retDeg << "\n";
    }
    g_cellParamOut.flush();

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval,
                            &SampleCellControlParams,
                            interval,
                            stopTime,
                            enbLteDevs,
                            enbTerminators);
    }
}

int
main(int argc, char* argv[])
{
    std::string enbPosFile = "/workspace/data/ns3_positions_Three_IE.txt";
    std::string resultRoot = "/workspace/results/dublin-es-mro";
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
    std::string scheduler = "pf";
    bool useIdealRrc = true;
    bool disablePhyErr = true;
    bool enableRlfDetection = false;
    bool enableCco = false;
    bool enableMlb = false;
    bool mlbControlTtt = false;
    double pingPongWindowSec = 5.0;
    double ccoLowRsrpThresholdDbm = -110.0;
    double ccoLowRsrpFractionThreshold = 0.10;
    double mlbLoadImbalanceThreshold = 0.20;
    double mlbCioStepDb = 1.0;
    std::string dbFileName = "oran-dublin-es-mro.db";

    CommandLine cmd(__FILE__);
    cmd.AddValue("enbPosFile", "Dublin eNB position file", enbPosFile);
    cmd.AddValue("resultRoot", "Root directory for result subdirectories", resultRoot);
    cmd.AddValue("mitigation-method",
                 "CMM method: none, cancel, dampen, priority, nswf, eg, qacm",
                 mitigationMethod);
    cmd.AddValue("targetUes", "Total number of UEs distributed around Dublin sites", targetUes);
    cmd.AddValue("ueDiscR", "UE placement radius around each eNB", ueDiscR);
    cmd.AddValue("txPower", "Initial eNB TxPower in dBm", txPower);
    cmd.AddValue("target-power-w", "ES target power per eNB in W", targetPowerW);
    cmd.AddValue("step-size-db", "ES TxPower step per decision", stepSizeDb);
    cmd.AddValue("sim-time", "Simulation duration", simTime);
    cmd.AddValue("sample-interval", "KPI sampling interval", sampleInterval);
    cmd.AddValue("ric-start-time", "Time to activate the Near-RT RIC", ricStartTime);
    cmd.AddValue("scheduler", "pf or rr", scheduler);
    cmd.AddValue("useIdealRrc", "Use LTE ideal RRC for stable RIC-driven handovers", useIdealRrc);
    cmd.AddValue("disablePhyErr", "Disable LTE PHY error models", disablePhyErr);
    cmd.AddValue("enableRlfDetection", "Enable LTE UE radio-link-failure detection", enableRlfDetection);
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
    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(useIdealRrc));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(enableRlfDetection));
    if (disablePhyErr)
    {
        Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
        Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
    }

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
    NS_ABORT_MSG_IF(!g_enbOut.is_open(), "Cannot open eNB KPI output " << enbPath);
    NS_ABORT_MSG_IF(!g_posOut.is_open(), "Cannot open position output " << posPath);
    NS_ABORT_MSG_IF(!g_cellParamOut.is_open(),
                    "Cannot open cell-parameter output " << cellParamPath);
    NS_ABORT_MSG_IF(!g_eventOut.is_open(), "Cannot open event output " << eventPath);

    g_kpiOut << "time_s,interval_throughput_mbps,cumulative_throughput_mbps,total_rx_bytes,"
             << "total_energy_j,energy_efficiency_bits_per_j,avg_txpower_dbm,avg_rsrp_dbm,"
             << "min_rsrp_dbm,avg_sinr_db,rsrp_samples,handover_ok,handover_failures,"
             << "ping_pong_handovers,radio_link_failures,connection_timeouts\n";
    g_enbOut << "time_s,enb_index,txpower_dbm,energy_j\n";
    g_posOut << "time_s,node_type,node_index,x_m,y_m,z_m\n";
    g_cellParamOut << "time_s,enb_index,e2_node_id,cell_id,txpower_dbm,cio_db,ttt_s,hys_db,"
                   << "ret_deg\n";
    g_eventOut << "time_s,event_type,context,imsi,cell_id,rnti,target_cell_id,value\n";

    std::vector<Vector> siteCenters = LoadEnbPositionsFromVectorFile(enbPosFile);
    const uint32_t numberOfEnbs = siteCenters.size();
    const uint32_t numberOfUes = targetUes;

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(50));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType((scheduler == "rr") ? "ns3::RrFfMacScheduler"
                                                     : "ns3::PfFfMacScheduler");
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"),
                                               1);

    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> enbPosAlloc = CreateObject<ListPositionAllocator>();
    for (const auto& p : siteCenters)
    {
        enbPosAlloc->Add(p);
    }
    MobilityHelper enbMobility;
    enbMobility.SetPositionAllocator(enbPosAlloc);
    enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);

    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& p : siteCenters)
    {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }

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

    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    for (NetDeviceContainer::Iterator it = enbLteDevs.Begin(); it != enbLteDevs.End(); ++it)
    {
        Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(*it);
        enb->GetPhy()->SetTxPower(txPower);
    }
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces =
        epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Vector p = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        auto bestTwo = BestTwoEnbIdx(p, enbNodes);
        const uint32_t serving = (i % 4 == 0 && numberOfEnbs > 1) ? bestTwo.second : bestTwo.first;
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(serving));
    }
    lteHelper->AddX2Interface(enbNodes);

    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(5000000.0));
    sourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    energy::EnergySourceContainer sources = sourceHelper.Install(enbNodes);
    std::vector<Ptr<OranRuDeviceEnergyModel>> enbEnergyModels;
    std::vector<Ptr<energy::BasicEnergySource>> enbEnergySources;
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(sources.Get(i));
        Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetLteEnbPhy(enb->GetPhy());

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
        enbEnergyModels.push_back(dem);
        enbEnergySources.push_back(src);
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
        onoff->SetAttribute("OnTime",
                            StringValue("ns3::ExponentialRandomVariable[Mean=0.6]"));
        onoff->SetAttribute("OffTime",
                            StringValue("ns3::ExponentialRandomVariable[Mean=1.0]"));
        remoteHost->AddApplication(onoff);
        remoteApps.Add(onoff);
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
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover");
    oranHelper->AddLogicModule("ns3::OranLmLte2LteEnergySaving",
                               "TargetPowerW",
                               DoubleValue(targetPowerW),
                               "StepSize",
                               DoubleValue(stepSizeDb),
                               "LmIntervalSec",
                               DoubleValue(lmQueryInterval.GetSeconds()));
    if (enableCco)
    {
        oranHelper->AddLogicModule("ns3::OranLmLte2LteCoverageCapacityOptimization",
                                   "LowRsrpThresholdDbm",
                                   DoubleValue(ccoLowRsrpThresholdDbm),
                                   "LowRsrpFractionThreshold",
                                   DoubleValue(ccoLowRsrpFractionThreshold),
                                   "StepSize",
                                   DoubleValue(stepSizeDb));
    }
    if (enableMlb)
    {
        oranHelper->AddLogicModule("ns3::OranLmLte2LteMobilityLoadBalancing",
                                   "LoadImbalanceThreshold",
                                   DoubleValue(mlbLoadImbalanceThreshold),
                                   "CioStep",
                                   DoubleValue(mlbCioStepDb),
                                   "ControlTtt",
                                   BooleanValue(mlbControlTtt));
    }
    oranHelper->SetConflictMitigationModule("ns3::OranCmmLte2LteEsMro",
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
        Ptr<OranReporterLteUeCellInfo> cellInfoReporter = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter =
            CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe> ueTerminator = CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        cellInfoReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(ueTerminator));

        Ptr<LteUeNetDevice> ueLteDevice = DynamicCast<LteUeNetDevice>(ueLteDevs.Get(i));
        Ptr<LteUePhy> uePhy = ueLteDevice->GetPhy();
        uePhy->TraceConnectWithoutContext(
            "ReportUeMeasurements",
            MakeCallback(&ns3::OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
        uePhy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr",
                                          MakeCallback(&CurrentCellRsrpSinrTrace));

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
                            &OranE2NodeTerminatorLteUe::Activate,
                            ueTerminator);
    }

    OranE2NodeTerminatorContainer enbTerminators;
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<OranReporterLocation> locReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteEnergyEfficiency> energyReporter =
            CreateObject<OranReporterLteEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorLteEnb> enbTerminator = CreateObject<OranE2NodeTerminatorLteEnb>();

        locReporter->SetAttribute("Terminator", PointerValue(enbTerminator));
        locReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        energyReporter->SetEnergySource(enbEnergySources[i]);
        energyReporter->SetAttribute("Terminator", PointerValue(enbTerminator));
        energyReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        enbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        enbTerminator->SetAttribute("RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        enbTerminator->SetAttribute("SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=2]"));
        enbTerminator->AddReporter(locReporter);
        enbTerminator->AddReporter(energyReporter);
        enbTerminator->Attach(enbNodes.Get(i));
        enbTerminators.Add(enbTerminator);
        Simulator::Schedule(ricStartTime + Seconds(0.5),
                            &OranE2NodeTerminatorLteEnb::Activate,
                            enbTerminator);
    }

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOk));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
                    MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&NotifyUeHandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&NotifyUeHandoverEndError));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RadioLinkFailure",
                    MakeCallback(&NotifyRadioLinkFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
                    MakeCallback(&NotifyConnectionTimeout));

    Simulator::Schedule(ricStartTime,
                        &OranHelper::ActivateAndStartNearRtRic,
                        oranHelper,
                        nearRtRic);
    Simulator::Schedule(sampleInterval,
                        &SampleKpis,
                        sampleInterval,
                        simTime,
                        enbLteDevs,
                        enbEnergyModels);
    Simulator::Schedule(sampleInterval,
                        &SampleCellControlParams,
                        sampleInterval,
                        simTime,
                        enbLteDevs,
                        enbTerminators);
    Simulator::Schedule(Seconds(0),
                        &WritePositionSnapshot,
                        positionSampleInterval,
                        simTime,
                        enbNodes,
                        ueNodes);

    Simulator::Stop(simTime);
    Simulator::Run();

    double totalEnergyJ = 0.0;
    double avgTxPower = 0.0;
    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
    {
        Ptr<LteEnbNetDevice> enb = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        totalEnergyJ += enbEnergyModels[i]->GetTotalEnergyConsumption();
        avgTxPower += enb->GetPhy()->GetTxPower();
    }
    avgTxPower /= std::max<uint32_t>(enbLteDevs.GetN(), 1);
    const double throughputMbps = (g_totalRxBytes * 8.0) / simTime.GetSeconds() / 1e6;
    const double energyEfficiency = (g_totalRxBytes * 8.0) / std::max(totalEnergyJ, 1e-12);

    std::cout << "RESULT: method=" << mitigationMethod << " enbs=" << numberOfEnbs
              << " ues=" << numberOfUes << " throughput_mbps=" << throughputMbps
              << " handover_ok=" << g_handoverOk
              << " handover_failures=" << g_handoverFailures
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
