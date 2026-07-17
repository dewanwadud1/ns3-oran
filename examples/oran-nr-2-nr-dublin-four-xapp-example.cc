/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-dublin-four-xapp-example.cc.
 *
 * Deploys all four O-RAN xApps (ES, CCO, MRO, MLB) on the real Dublin gNB
 * topology, mediated by OranCmmConflictTriageNr, on a sub-6GHz NR band.
 * Same conflict taxonomy, KPI thresholds, and combined per-cycle KPI CSV as
 * the LTE original.
 *
 * ONNX support (added after the initial rule-based-only port): --use-onnx-es/
 * -mro/-mlb/-cco swap in the DQN ONNX LM for that xApp (see
 * workspace/oran/model/oran-lm-nr-2-nr-onnx-*.{h,cc}), mirroring the LTE
 * capstone's flags. --ground-truth dumps real per-UE kinematics/RSRP/LOS-NLOS
 * (CollectGroundTruth) for offline MRO retraining, and --los-trace dumps a
 * per-sample LOS/NLOS RSRP trace -- both direct ports of the LTE original's
 * mechanism, sourced from the same NR ThreeGppPropagationLossModel's
 * ChannelConditionModel that drives the real downlink channel.
 *
 * NR-specific additions vs. the LTE CSV schema:
 *   - avg_sinr_db is now REAL data (not NaN): NrUePhy's "DlDataSinr" trace
 *     (cellId, rnti, avgSinrLinear, bwpId) reports true per-UE average DL
 *     data SINR every time a CQI is computed; converted to dB and averaged
 *     per cell, same as avg_rsrp_dbm.
 *   - Three NR-only radio-config columns are appended to every row:
 *     numerology, bandwidth_mhz, carrier_freq_ghz (constant for the whole
 *     run -- these have no LTE equivalent since LTE doesn't have
 *     numerology/flexible-BWP the way NR does).
 *
 * UE mobility-mode diversity (7 profiles), traffic-type diversity
 * (eMBB/URLLC/mMTC/V2X), bandwidth-capacity-aware MLB, and the full
 * conflict-event-per-CSV-row KPI pipeline are all preserved unchanged.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/channel-condition-model.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-cmm-conflict-triage-nr.h"
#include "ns3/oran-lm-nr-2-nr-onnx-cco.h"
#include "ns3/oran-lm-nr-2-nr-onnx-energy-saving.h"
#include "ns3/oran-lm-nr-2-nr-onnx-mlb.h"
#include "ns3/oran-lm-nr-2-nr-onnx-mro.h"
#include "ns3/oran-lm-nr-2-nr-txp-calibration.h"
#include "ns3/oran-module.h"
#include "ns3/oran-nr-cell-control-state.h"
#include "ns3/oran-nr-ru-energy-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrDublinFourXappExample");

static constexpr double kEeThresh = 1e6;
static constexpr double kRsrpThresh = -95.0;
static constexpr double kSinrThresh = 5.0;
static constexpr double kCdrThresh = 0.02;
static constexpr double kHsrThresh = 0.95;
static constexpr double kTlThresh = 0.80;

// NR radio config, fixed for the whole run (not CLI-configurable) --
// exposed as constant CSV columns since NR's numerology/BWP concept has no
// LTE equivalent.
static constexpr double kCentralFrequencyHz = 3.5e9;
static constexpr double kBandwidthHz = 20e6;
static constexpr uint16_t kNumerology = 1;

// Keyed by the globally-unique ns-3 UE node index, NOT rnti: RNTI is
// assigned per-cell (each of the 13 gNBs independently numbers its own
// attached UEs starting from 1), so with 117 UEs spread across 13 cells,
// many physically-different UEs share the same rnti value. Keying these
// maps by rnti alone let unrelated UEs on different cells silently
// overwrite each other's entries, which was a real and severe bug: most
// cells' UE data was being lost/misattributed, not just occasionally but
// on essentially every report.
static std::map<uint32_t, double> g_latestUeRsrp;
static std::map<uint32_t, uint16_t> g_latestUeCell;
static std::map<uint32_t, double> g_latestUeSinrDb;

static std::map<uint16_t, uint32_t> g_hoAttempts;
static std::map<uint16_t, uint32_t> g_hoOk;
static std::map<uint16_t, uint32_t> g_hoFail;
static std::map<uint16_t, uint32_t> g_pingPong;
static std::map<uint16_t, uint32_t> g_connTimeouts;

struct LastHoTransition
{
    uint16_t sourceCellId = 0;
    uint16_t targetCellId = 0;
    double timeSec = 0.0;
};
static std::map<uint64_t, LastHoTransition> g_lastHoTransitionByImsi;
static double g_pingPongWindowSec = 5.0;
static uint32_t g_pingPongTotal = 0;

static std::map<uint16_t, uint64_t> g_rxBytesByCellId;
static std::map<uint16_t, uint64_t> g_rxBytesByCellIdLastCycle;

static std::map<uint32_t, uint64_t> g_ueRxBytesTotal;
static std::map<uint32_t, uint64_t> g_ueRxBytesLastReport;

static std::ofstream g_kpiCsvFile;

static uint32_t g_hoOkTotal = 0;
static uint32_t g_hoFailTotal = 0;
static uint32_t g_rlfTotal = 0;
static uint32_t g_connTimeoutsTotal = 0;
static uint64_t g_rxBytesTotal = 0;
static uint64_t g_rxBytesLastCycle = 0;
static uint32_t g_rsrpViolCycles = 0;

static NetDeviceContainer g_gnbDevs;
static OranE2NodeTerminatorContainer g_e2GnbTerms;
static OranE2NodeTerminatorContainer g_e2UeTerms;
static std::vector<Ptr<OranNrRuDeviceEnergyModel>> g_gnbEnergyModels;
static std::vector<double> g_gnbInitialEnergyJ;

static uint32_t g_cycleCount = 0;

struct UeModeProfile
{
    std::string name;
    double vMin;
    double vMax;
    double headingNoiseStd;
    double speedNoiseFrac;
};

static const std::vector<UeModeProfile> g_ueModeProfiles = {
    {"PED", 0.8, 2.0, 0.35, 0.08},
    {"CYCLIST", 2.5, 5.0, 0.25, 0.06},
    {"DRONE", 5.5, 8.5, 0.30, 0.10},
    {"BUS", 9.0, 14.0, 0.05, 0.03},
    {"CAR", 15.0, 22.0, 0.12, 0.05},
    {"UAV", 23.0, 26.0, 0.20, 0.08},
    {"TRAIN", 27.0, 40.0, 0.02, 0.02},
};

struct KinematicSample
{
    double time;
    double speed;
    double heading;
};

struct UeMobilityState
{
    std::size_t profileIdx;
    double speed;
    double heading;
    std::deque<KinematicSample> history; //!< last few samples, for finite-differencing
};

static std::map<uint32_t, UeMobilityState> g_ueMobState;
static std::map<uint32_t, std::string> g_ueMode;

// gNB mobility + shared channel condition model instance actually driving the
// downlink path loss -- used by both CollectGroundTruth (real LOS/NLOS
// logging) and the LOS/NLOS RSRP trace further below, and wired into
// OranLmNr2NrOnnxMro when --use-onnx-mro is active.
static std::map<uint16_t, Ptr<MobilityModel>> g_cellIdToEnbMobility;
static Ptr<ChannelConditionModel> g_channelConditionModel;

struct TrafficTypeProfile
{
    std::string name;
    double weight;
    double minMbps;
    double maxMbps;
    uint32_t packetSize;
    std::string onTimeExpr;
    std::string offTimeExpr;
};

static const std::vector<TrafficTypeProfile> g_trafficProfiles = {
    {"eMBB", 0.50, 5.0, 15.0, 1200, "ns3::ExponentialRandomVariable[Mean=4.0]",
     "ns3::ExponentialRandomVariable[Mean=1.0]"},
    {"URLLC", 0.20, 0.5, 2.0, 150, "ns3::ExponentialRandomVariable[Mean=0.05]",
     "ns3::ExponentialRandomVariable[Mean=0.05]"},
    {"mMTC", 0.20, 0.01, 0.1, 100, "ns3::ExponentialRandomVariable[Mean=0.5]",
     "ns3::ExponentialRandomVariable[Mean=10.0]"},
    {"V2X", 0.10, 1.0, 5.0, 300, "ns3::ExponentialRandomVariable[Mean=0.2]",
     "ns3::ExponentialRandomVariable[Mean=0.3]"},
};

static std::map<uint32_t, std::size_t> g_ueTrafficType;

std::size_t
DrawTrafficTypeIndex(Ptr<UniformRandomVariable> uv)
{
    double r = uv->GetValue(0.0, 1.0);
    double cumulative = 0.0;
    for (std::size_t i = 0; i < g_trafficProfiles.size(); ++i)
    {
        cumulative += g_trafficProfiles[i].weight;
        if (r <= cumulative)
        {
            return i;
        }
    }
    return g_trafficProfiles.size() - 1;
}

void
UpdateUeKinematics(uint32_t ueIdx,
                  Ptr<ConstantVelocityMobilityModel> mob,
                  Ptr<NormalRandomVariable> speedNoiseRv,
                  Ptr<NormalRandomVariable> headingNoiseRv,
                  Time updateInterval)
{
    UeMobilityState& s = g_ueMobState[ueIdx];
    const UeModeProfile& p = g_ueModeProfiles[s.profileIdx];

    double speedStd = p.speedNoiseFrac * (p.vMax - p.vMin);
    s.speed = std::max(
        p.vMin, std::min(p.vMax, s.speed + speedNoiseRv->GetValue(0.0, speedStd * speedStd)));
    s.heading += headingNoiseRv->GetValue(0.0, p.headingNoiseStd * p.headingNoiseStd);

    mob->SetVelocity(Vector(s.speed * std::cos(s.heading), s.speed * std::sin(s.heading), 0.0));

    s.history.push_back({Simulator::Now().GetSeconds(), s.speed, s.heading});
    if (s.history.size() > 4)
    {
        s.history.pop_front();
    }

    Simulator::Schedule(updateInterval,
                        &UpdateUeKinematics,
                        ueIdx,
                        mob,
                        speedNoiseRv,
                        headingNoiseRv,
                        updateInterval);
}

// Ground-truth data collection for offline MRO retraining (workspace/ml/mro_dqn/):
// dumps exact kinematics (from the mobility-update state above, not finite-
// differenced from noisy position reports like OranLmNr2NrOnnxMro has to at
// inference time), true mode label, position, and real per-gNB RSRP (via the
// same OranDataRepository::GetNrUeRsrpRsrq accessor the ONNX LMs use, so this
// reflects the actual NR 3GPP channel model already running in this scenario).
void
CollectGroundTruth(Ptr<OranDataRepository> data,
                   NodeContainer ueNodes,
                   Time interval,
                   std::ofstream* out)
{
    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        auto it = g_ueMobState.find(u);
        if (it == g_ueMobState.end() || it->second.history.size() < 3)
        {
            continue; // cold start
        }
        const auto& h = it->second.history;
        const auto& h0 = h[h.size() - 3];
        const auto& h1 = h[h.size() - 2];
        const auto& h2 = h[h.size() - 1];

        double dt01 = std::max(h1.time - h0.time, 1e-3);
        double dt12 = std::max(h2.time - h1.time, 1e-3);
        double accelPrev = (h1.speed - h0.speed) / dt01;
        double accel = (h2.speed - h1.speed) / dt12;
        double jerk = (accel - accelPrev) / dt12;
        double headingDelta = h2.heading - h1.heading;
        while (headingDelta > M_PI)
            headingDelta -= 2 * M_PI;
        while (headingDelta < -M_PI)
            headingDelta += 2 * M_PI;
        double bearingRate = headingDelta / dt12;

        Ptr<MobilityModel> ueMob = ueNodes.Get(u)->GetObject<MobilityModel>();
        Vector pos = ueMob->GetPosition();
        uint64_t ueE2Id = g_e2UeTerms.Get(u)->GetE2NodeId();

        (*out) << Simulator::Now().GetSeconds() << "," << u << "," << g_ueMode[u] << ","
              << h2.speed << "," << accel << "," << jerk << "," << bearingRate << "," << pos.x
              << "," << pos.y << ",";
        bool first = true;
        for (const auto& tup : data->GetNrUeRsrpRsrq(ueE2Id))
        {
            if (!first)
            {
                (*out) << ";";
            }
            (*out) << std::get<1>(tup) << ":" << std::get<2>(tup);
            first = false;
        }

        // Real LOS/NLOS per (UE, gNB), from the same ChannelConditionModel that
        // actually drove each of the RSRP samples just written above.
        (*out) << ",";
        first = true;
        for (const auto& [cellId, enbMob] : g_cellIdToEnbMobility)
        {
            if (!first)
            {
                (*out) << ";";
            }
            bool isLos = g_channelConditionModel->GetChannelCondition(enbMob, ueMob)->IsLos();
            (*out) << cellId << ":" << (isLos ? 1 : 0);
            first = false;
        }
        (*out) << "\n";
    }
    out->flush();

    Simulator::Schedule(interval, &CollectGroundTruth, data, ueNodes, interval, out);
}

static std::ofstream g_losTraceFile;

static std::vector<Vector>
LoadPositions(const std::string& path, uint32_t maxGnbs)
{
    std::ifstream in(path);
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open gNB position file: " << path);
    std::vector<Vector> pos;
    std::string line;
    std::regex rx(R"(Vector\s*\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+))");
    std::smatch m;
    while (std::getline(in, line))
    {
        if (std::regex_search(line, m, rx))
            pos.emplace_back(std::stod(m[1]), std::stod(m[2]), std::stod(m[3]));
        if (pos.size() >= maxGnbs)
            break;
    }
    NS_ABORT_MSG_IF(pos.empty(), "No positions found in " << path);
    return pos;
}

// NrUePhy's "ReportUeMeasurements" trace: (rnti, cellId, rsrp, rsrq, isServing, ccId).
// ueIdx is bound per-UE at TraceConnect time (see UE setup loop) since rnti
// alone is not a safe map key across multiple cells -- see comment at the
// g_latestUeRsrp declaration.
void
ObserveRsrp(uint32_t ueIdx,
           uint16_t /*rnti*/,
           uint16_t cellId,
           double rsrp,
           double /*rsrq*/,
           bool serving,
           uint8_t)
{
    if (!serving)
        return;
    g_latestUeRsrp[ueIdx] = rsrp;
    g_latestUeCell[ueIdx] = cellId;
}

// NrUePhy's "DlDataSinr" trace: (cellId, rnti, avgSinrLinear, bwpId).
void
ObserveSinr(uint32_t ueIdx, uint16_t /*cellId*/, uint16_t /*rnti*/, double avgSinrLinear, uint16_t /*bwpId*/)
{
    if (!std::isfinite(avgSinrLinear) || avgSinrLinear <= 0.0)
        return;
    g_latestUeSinrDb[ueIdx] = 10.0 * std::log10(avgSinrLinear);
}

// Bound to (ueMobility, ueIdx) per UE at connection time (see the UE setup
// loop below), so we get the serving UE's own mobility model directly instead
// of reverse-mapping it from rnti. Queries the *same* ChannelConditionModel
// instance the NR ThreeGppPropagationLossModel uses to compute this RSRP
// sample's path loss, so the logged LOS/NLOS state is the one that was
// actually applied, not an independent re-roll. NrUePhy's
// "ReportUeMeasurements" trace already reports rsrp in dBm (unlike LTE's
// ReportCurrentCellRsrpSinr, which reports linear Watts), so no W->dBm
// conversion is needed here.
void
ObserveLosNlosRsrp(Ptr<MobilityModel> ueMob,
                  uint32_t ueIdx,
                  uint16_t /*rnti*/,
                  uint16_t cellId,
                  double rsrpDbm,
                  double /*rsrq*/,
                  bool serving,
                  uint8_t)
{
    if (!serving || !g_channelConditionModel)
    {
        return;
    }
    auto it = g_cellIdToEnbMobility.find(cellId);
    if (it == g_cellIdToEnbMobility.end())
    {
        return;
    }

    Ptr<ChannelCondition> cond = g_channelConditionModel->GetChannelCondition(it->second, ueMob);
    bool isLos = cond->IsLos();

    Vector enbPos = it->second->GetPosition();
    Vector uePos = ueMob->GetPosition();
    double dist2d = std::sqrt(std::pow(enbPos.x - uePos.x, 2) + std::pow(enbPos.y - uePos.y, 2));

    g_losTraceFile << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << ","
                  << ueIdx << "," << cellId << "," << (isLos ? "LOS" : "NLOS") << "," << rsrpDbm
                  << "," << dist2d << "\n";
}

void
RxBytes(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytesTotal += pkt->GetSize();
}

void
RxByCell(uint32_t ueIdx, Ptr<NrUeNetDevice> ueDev, Ptr<const Packet> pkt, const Address&)
{
    uint16_t cellId = ueDev->GetRrc()->GetCellId();
    uint32_t size = pkt->GetSize();
    g_rxBytesByCellId[cellId] += size;
    g_ueRxBytesTotal[ueIdx] += size;
}

void
ReportUeDemand(uint32_t ueIdx, Ptr<OranReporterNrUeAppDemand> reporter, Time interval)
{
    uint64_t total = g_ueRxBytesTotal.count(ueIdx) ? g_ueRxBytesTotal[ueIdx] : 0;
    uint64_t last = g_ueRxBytesLastReport.count(ueIdx) ? g_ueRxBytesLastReport[ueIdx] : 0;
    g_ueRxBytesLastReport[ueIdx] = total;

    double mbps = ((total - last) * 8.0) / (interval.GetSeconds() * 1e6);
    reporter->ReportDemand(mbps);

    Simulator::Schedule(interval, &ReportUeDemand, ueIdx, reporter, interval);
}

uint16_t
SourceCellForOutcome(uint64_t imsi, uint16_t fallbackCellId)
{
    auto it = g_lastHoTransitionByImsi.find(imsi);
    return (it != g_lastHoTransitionByImsi.end()) ? it->second.sourceCellId : fallbackCellId;
}

void
HandoverOk(std::string, uint64_t imsi, uint16_t targetCellId, uint16_t)
{
    g_hoOkTotal++;
    g_hoOk[SourceCellForOutcome(imsi, targetCellId)]++;
}

void
HandoverStart(std::string, uint64_t imsi, uint16_t srcCell, uint16_t, uint16_t targetCell)
{
    g_hoAttempts[srcCell]++;

    const double now = Simulator::Now().GetSeconds();
    auto it = g_lastHoTransitionByImsi.find(imsi);
    if (it != g_lastHoTransitionByImsi.end() && it->second.sourceCellId == targetCell &&
        it->second.targetCellId == srcCell && (now - it->second.timeSec) <= g_pingPongWindowSec)
    {
        g_pingPongTotal++;
        g_pingPong[srcCell]++;
    }
    g_lastHoTransitionByImsi[imsi] = {srcCell, targetCell, now};
}

void
HandoverFail(std::string, uint64_t imsi, uint16_t cellId, uint16_t)
{
    g_hoFailTotal++;
    g_hoFail[SourceCellForOutcome(imsi, cellId)]++;
}

// NrGnbRrc's four HandoverFailure* traces fire as (imsi, rnti, cellId) --
// confirmed identical (and identically-swapped) argument order to
// LteEnbRrc's traces of the same name, via direct source grep of nr-gnb-rrc.cc.
void
HandoverFailEnb(std::string, uint64_t imsi, uint16_t /*rnti*/, uint16_t cellId)
{
    g_hoFailTotal++;
    g_hoFail[SourceCellForOutcome(imsi, cellId)]++;
}

void
ConnectionTimeout(std::string, uint64_t, uint16_t cellId, uint16_t, uint8_t)
{
    g_connTimeouts[cellId]++;
    g_connTimeoutsTotal++;
}

void
RadioLinkFailure(std::string, uint64_t, uint16_t, uint16_t)
{
    g_rlfTotal++;
}

void
LogCellState(Time interval, double lmIntervalSec, Ptr<OranCmmConflictTriageNr> triage,
            double enbCapacityMbps)
{
    double now = Simulator::Now().GetSeconds();
    g_cycleCount++;

    double worstRsrp = 0.0;
    uint32_t totalUes = 0;
    for (const auto& kv : g_latestUeRsrp)
    {
        if (worstRsrp == 0.0 || kv.second < worstRsrp)
            worstRsrp = kv.second;
        totalUes++;
    }

    uint32_t totalHoAttempts = 0, totalHoOk = 0, totalHoFail = 0;
    for (auto& kv : g_hoAttempts)
        totalHoAttempts += kv.second;
    for (auto& kv : g_hoOk)
        totalHoOk += kv.second;
    for (auto& kv : g_hoFail)
        totalHoFail += kv.second;

    double netCdr = (totalHoAttempts > 0) ? static_cast<double>(totalHoFail) / totalHoAttempts : 0.0;
    double netHsr = (totalHoAttempts > 0) ? static_cast<double>(totalHoOk) / totalHoAttempts : 1.0;

    double sumAllRsrp = 0.0;
    uint32_t rsrpN = 0;
    for (const auto& kv : g_latestUeRsrp)
    {
        sumAllRsrp += kv.second;
        rsrpN++;
    }
    double avgRsrp = (rsrpN > 0) ? sumAllRsrp / rsrpN : 0.0;

    uint64_t deltaBytes = g_rxBytesTotal - g_rxBytesLastCycle;
    g_rxBytesLastCycle = g_rxBytesTotal;
    double cycleThrMbps = (deltaBytes * 8.0) / (lmIntervalSec * 1e6);

    bool rsrpViol = (totalUes > 0 && worstRsrp < kRsrpThresh);
    bool cdrViol = (netCdr > kCdrThresh);
    bool hsrViol = (totalHoAttempts > 0 && netHsr < kHsrThresh);
    if (rsrpViol)
        g_rsrpViolCycles++;
    std::vector<std::string> violationCauses;
    if (rsrpViol)
        violationCauses.push_back("RSRP");
    if (cdrViol)
        violationCauses.push_back("CDR");
    if (hsrViol)
        violationCauses.push_back("HSR");
    const std::string status = violationCauses.empty() ? "OK" : "VIOLATION";

    std::cout << "[PMON] t=" << std::fixed << std::setprecision(2) << now << "s"
              << " worst_RSRP=" << (totalUes > 0 ? worstRsrp : 0.0) << "dBm"
              << " avg_RSRP=" << std::setprecision(2) << (rsrpN > 0 ? avgRsrp : 0.0) << "dBm"
              << " throughput_mbps=" << std::setprecision(3) << cycleThrMbps
              << " QoS_threshold=" << kRsrpThresh << "dBm"
              << " net_CDR=" << std::setprecision(4) << netCdr << " net_HSR=" << netHsr
              << " status=" << status;
    if (!violationCauses.empty())
    {
        std::cout << " causes=[";
        for (std::size_t i = 0; i < violationCauses.size(); ++i)
        {
            std::cout << (i == 0 ? "" : ",") << violationCauses[i];
        }
        std::cout << "]";
    }
    std::cout << "\n";

    for (uint32_t i = 0; i < g_gnbDevs.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(g_gnbDevs.Get(i));
        uint64_t e2id = g_e2GnbTerms.Get(i)->GetE2NodeId();
        uint16_t cellId = dev->GetCellId();
        OranNrCellControlParams cp = GetNrCellControlParameters(e2id);

        double minRsrp = 0.0;
        uint32_t nUes = 0;
        double sumSinr = 0.0;
        uint32_t nSinrSamples = 0;
        for (const auto& kv : g_latestUeRsrp)
        {
            auto cit = g_latestUeCell.find(kv.first);
            if (cit == g_latestUeCell.end() || cit->second != cellId)
                continue;
            if (nUes == 0 || kv.second < minRsrp)
                minRsrp = kv.second;
            nUes++;
            auto sit = g_latestUeSinrDb.find(kv.first);
            if (sit != g_latestUeSinrDb.end() && std::isfinite(sit->second))
            {
                sumSinr += sit->second;
                nSinrSamples++;
            }
        }
        double avgSinr = (nSinrSamples > 0) ? (sumSinr / nSinrSamples)
                                            : std::numeric_limits<double>::quiet_NaN();

        uint32_t att = g_hoAttempts.count(cellId) ? g_hoAttempts[cellId] : 0;
        uint32_t ok = g_hoOk.count(cellId) ? g_hoOk[cellId] : 0;
        uint32_t fail = g_hoFail.count(cellId) ? g_hoFail[cellId] : 0;
        double cdr = (att > 0) ? static_cast<double>(fail) / att : 0.0;
        double hsr = (att > 0) ? static_cast<double>(ok) / att : 1.0;

        double tl = (totalUes > 0) ? static_cast<double>(nUes) / totalUes : 0.0;

        uint64_t cellBytes = g_rxBytesByCellId.count(cellId) ? g_rxBytesByCellId[cellId] : 0;
        double energyConsumedJ = std::max(g_gnbEnergyModels[i]->GetTotalEnergyConsumption(), 1e-9);
        double ee = (cellBytes * 8.0) / energyConsumedJ;

        uint64_t cellBytesLast =
            g_rxBytesByCellIdLastCycle.count(cellId) ? g_rxBytesByCellIdLastCycle[cellId] : 0;
        g_rxBytesByCellIdLastCycle[cellId] = cellBytes;
        double cellThrMbps = ((cellBytes - cellBytesLast) * 8.0) / (lmIntervalSec * 1e6);

        uint32_t pingPong = g_pingPong.count(cellId) ? g_pingPong[cellId] : 0;
        uint32_t connTimeout = g_connTimeouts.count(cellId) ? g_connTimeouts[cellId] : 0;

        double txpFreq = 0.0, cioFreq = 0.0, retFreq = 0.0, tttFreq = 0.0;
        if (triage)
        {
            std::map<std::string, uint32_t> icpCounts = triage->GetAndResetIcpCounts(e2id);
            auto getCount = [&icpCounts](const char* name) {
                auto it = icpCounts.find(name);
                return it != icpCounts.end() ? static_cast<double>(it->second) : 0.0;
            };
            txpFreq = getCount("TxPower");
            cioFreq = getCount("CIO");
            retFreq = getCount("RET");
            tttFreq = getCount("TTT");
        }

        auto joinRoles = [](const std::set<std::string>& roles) {
            if (roles.empty())
            {
                return std::string("-");
            }
            std::ostringstream oss;
            bool first = true;
            for (const auto& r : roles)
            {
                oss << (first ? "" : ";") << r;
                first = false;
            }
            return oss.str();
        };
        std::string txpBy = "-", cioBy = "-", retBy = "-", tttBy = "-";
        if (triage)
        {
            std::map<std::string, std::set<std::string>> icpActors = triage->GetAndResetIcpActors(e2id);
            auto getActors = [&icpActors](const char* name) {
                auto it = icpActors.find(name);
                return it != icpActors.end() ? it->second : std::set<std::string>{};
            };
            txpBy = joinRoles(getActors("TxPower"));
            cioBy = joinRoles(getActors("CIO"));
            retBy = joinRoles(getActors("RET"));
            tttBy = joinRoles(getActors("TTT"));
        }

        std::vector<OranCmmConflictTriageNr::ConflictEventRecord> conflictEvents;
        if (triage)
        {
            conflictEvents = triage->GetAndResetConflictEvents(e2id);
        }

        bool rsrpViolCell = (nUes > 0 && minRsrp < kRsrpThresh);
        bool sinrViolCell = (nSinrSamples > 0 && avgSinr < kSinrThresh);
        bool cdrViolCell = (att > 0 && cdr > kCdrThresh);
        bool hsrViolCell = (att > 0 && hsr < kHsrThresh);
        bool tlViolCell = (tl > kTlThresh);
        bool eeViolCell = (nUes > 0 && ee < kEeThresh);

        std::cout << "[STATE] t=" << std::fixed << std::setprecision(2) << now << "s"
                  << " e2=" << e2id << " cell=" << cellId << " TXP=" << dev->GetPhy(0)->GetTxPower()
                  << "dBm"
                  << " CIO=" << cp.cioDb << "dB"
                  << " TTT=" << cp.tttSec * 1000.0 << "ms"
                  << " HYS=" << cp.hysDb << "dB"
                  << " RET=" << cp.retDeg << "deg"
                  << " nUEs=" << nUes << " minRSRP=" << (nUes > 0 ? minRsrp : 0.0) << "dBm"
                  << " avgSINR=" << avgSinr << "dB"
                  << " CDR=" << std::setprecision(4) << cdr << " HSR=" << hsr << " TL=" << tl
                  << " EE=" << std::setprecision(2) << ee << " hoAttempts=" << att
                  << " puf_TXP=" << txpFreq << " puf_CIO=" << cioFreq << " puf_RET=" << retFreq
                  << " puf_TTT=" << tttFreq << " thr_RSRP=" << kRsrpThresh
                  << " thr_SINR=" << kSinrThresh << " thr_CDR=" << kCdrThresh
                  << " thr_HSR=" << kHsrThresh << " thr_TL=" << kTlThresh << " thr_EE=" << kEeThresh
                  << "\n";

        auto writeCsvRow = [&](const std::string& conflictType,
                               const std::string& conflictIcp,
                               const std::string& conflictingXapps,
                               const std::string& affectedXapps,
                               const std::string& winnerXapp) {
            if (!g_kpiCsvFile.is_open())
            {
                return;
            }
            g_kpiCsvFile << now << "," << e2id << "," << cellId << "," << dev->GetPhy(0)->GetTxPower()
                        << "," << cp.cioDb << "," << cp.tttSec * 1000.0 << "," << cp.hysDb << ","
                        << cp.retDeg << "," << nUes << "," << (nUes > 0 ? minRsrp : 0.0) << ","
                        << avgSinr << "," << cdr << "," << hsr << "," << tl << "," << ee << ","
                        << cellThrMbps << "," << cellThrMbps << "," << enbCapacityMbps << ","
                        << (cellThrMbps / enbCapacityMbps) << "," << att << "," << ok << "," << fail
                        << "," << pingPong << "," << connTimeout << "," << txpFreq << "," << cioFreq
                        << "," << retFreq << "," << tttFreq << "," << txpBy << "," << cioBy << ","
                        << retBy << "," << tttBy << "," << conflictType << "," << conflictIcp << ","
                        << conflictingXapps << "," << affectedXapps << "," << winnerXapp << ","
                        << cycleThrMbps << "," << (totalUes > 0 ? worstRsrp : 0.0) << ","
                        << (rsrpN > 0 ? avgRsrp : 0.0) << "," << netCdr << "," << netHsr << ","
                        << status << "," << kRsrpThresh << "," << kSinrThresh << "," << kCdrThresh
                        << "," << kHsrThresh << "," << kTlThresh << "," << kEeThresh << ","
                        << rsrpViolCell << "," << sinrViolCell << "," << cdrViolCell << ","
                        << hsrViolCell << "," << tlViolCell << "," << eeViolCell << ","
                        << kNumerology << "," << (kBandwidthHz / 1e6) << ","
                        << (kCentralFrequencyHz / 1e9) << "\n";
            g_kpiCsvFile.flush();
        };

        if (conflictEvents.empty())
        {
            writeCsvRow("None", "", "", "", "");
        }
        else
        {
            for (const auto& ev : conflictEvents)
            {
                std::set<std::string> conflictingSet(ev.conflicting.begin(), ev.conflicting.end());
                std::set<std::string> affectedSet(ev.affected.begin(), ev.affected.end());
                writeCsvRow(ev.type, ev.icp, joinRoles(conflictingSet), joinRoles(affectedSet),
                           ev.winner);
            }
        }
    }

    g_hoAttempts.clear();
    g_hoOk.clear();
    g_hoFail.clear();
    g_pingPong.clear();
    g_connTimeouts.clear();

    Simulator::Schedule(interval, &LogCellState, interval, lmIntervalSec, triage, enbCapacityMbps);
}

int
main(int argc, char* argv[])
{
    std::string enbPosFile = "/workspace/data/ns3_positions_Three_IE.txt";
    std::string conflictLog = "";
    std::string kpiCsvPath = "";
    double pingPongWindowSec = 5.0;
    double enbCapacityMbps = 50.0;
    uint32_t maxEnbs = 13;
    uint32_t nUesPerEnb = 9;
    double simTime = 150.0;
    double lmInterval = 0.5;
    bool calibrateTxpLag = false; // diagnostic-only: replace all 4 xApps with a large
                                   // known-schedule TxPower square wave (see
                                   // OranLmNr2NrTxpCalibration) to empirically measure
                                   // the parameter-change-to-KPI-effect lag.
    double calibLowDbm = 10.0;
    double calibHighDbm = 45.0;
    uint32_t calibPeriodCycles = 6;
    double reportInterval = 0.5; // E2 measurement-report push cadence; independent of
                                  // lmInterval (the RIC's decision cadence) so that a large
                                  // --lm-interval (e.g. to hold power fixed for a PTX-sweep
                                  // calibration run) doesn't also starve the data repository
                                  // of RSRP/RSRQ reports.
    double txPower = 35.0; // matches LTE capstone's macro-cell starting power for this same
                           // real Dublin site topology -- 20dBm (this file's prior default)
                           // was far too low for real inter-site macro distances, causing
                           // most UEs' RSRP to sit at the coverage-limit floor; sub-6GHz vs
                           // mmWave does not imply a lower macro TxPower, so there was no
                           // real justification for deviating from LTE's value here.
    double esTargetW = 20.0;
    double esStepDb = 1.0;
    double ccoRsrpDbm = -95.0;
    double ccoCritDbm = -105.0;
    double ccoFracThr = 0.15;
    double ccoCritFrac = 0.30;
    double ccoRetStep = 1.0;
    double mlbThresh = 0.20;
    double ueSpeed = 3.0;
    std::string dbFile = "oran-nr-dublin-four-xapp.db";
    std::string mode = "reactive";
    double proactiveThresh = -90.0;
    double emaAlpha = 0.3;
    uint32_t predHorizon = 5;
    double proactiveRiskThreshold = 0.65;
    double proactiveRsrpGuardDb = 10.0;
    std::string groundTruthPath = ""; // if set, dump real MRO training data (workspace/ml/mro_dqn/)
    std::string losTracePath = "";    // LOS/NLOS RSRP trace CSV (empty = disabled)
    bool useOnnxEs = false;           // opt-in: DQN ONNX ES instead of rule-based
    bool useOnnxMro = false;          // opt-in: DQN ONNX MRO/handover instead of rule-based
    bool useOnnxMlb = false;          // opt-in: DQN ONNX MLB instead of rule-based
    bool useOnnxCco = false;          // opt-in: DQN ONNX CCO instead of rule-based

    const double centralFrequency = kCentralFrequencyHz;
    const double bandwidth = kBandwidthHz;
    const uint16_t numerology = kNumerology;

    CommandLine cmd(__FILE__);
    cmd.AddValue("enb-pos-file", "Dublin gNB position file", enbPosFile);
    cmd.AddValue("max-enbs", "Number of gNBs to load", maxEnbs);
    cmd.AddValue("n-ues-per-enb", "UEs per gNB", nUesPerEnb);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("lm-interval", "LM query interval (s)", lmInterval);
    cmd.AddValue("report-interval", "E2 measurement-report push interval (s)", reportInterval);
    cmd.AddValue("calibrate-txp-lag",
                 "Diagnostic-only: replace all 4 xApps with a large known-schedule "
                 "TxPower square wave (OranLmNr2NrTxpCalibration) to measure the "
                 "parameter-change-to-KPI-effect lag empirically",
                 calibrateTxpLag);
    cmd.AddValue("calib-low-dbm", "Calibration square-wave low extreme (dBm)", calibLowDbm);
    cmd.AddValue("calib-high-dbm", "Calibration square-wave high extreme (dBm)", calibHighDbm);
    cmd.AddValue("calib-period-cycles",
                 "Calibration: LM cycles spent at each extreme before toggling",
                 calibPeriodCycles);
    cmd.AddValue("tx-power", "Initial TxPower (dBm)", txPower);
    cmd.AddValue("es-target-w", "ES target power (W)", esTargetW);
    cmd.AddValue("es-step-db", "ES TxPower step (dB)", esStepDb);
    cmd.AddValue("cco-rsrp-dbm", "CCO/violation RSRP threshold (dBm)", ccoRsrpDbm);
    cmd.AddValue("cco-crit-dbm", "CCO critical-RSRP threshold", ccoCritDbm);
    cmd.AddValue("cco-frac-thr", "CCO low-RSRP fraction trigger", ccoFracThr);
    cmd.AddValue("cco-crit-frac", "CCO critical fraction trigger", ccoCritFrac);
    cmd.AddValue("cco-ret-step", "CCO RET step (deg)", ccoRetStep);
    cmd.AddValue("mlb-threshold", "MLB load imbalance threshold", mlbThresh);
    cmd.AddValue("ue-speed", "UE mobility speed (m/s)", ueSpeed);
    cmd.AddValue("conflict-log", "CDC CSV output path", conflictLog);
    cmd.AddValue("kpi-csv",
                 "Combined per-cycle, per-eNB parameter+KPI CSV output path (empty=disabled)",
                 kpiCsvPath);
    cmd.AddValue("ping-pong-window-sec", "A-B-A handover window counted as ping-pong",
                pingPongWindowSec);
    cmd.AddValue("enb-capacity-mbps",
                 "Fixed per-gNB bandwidth capacity MLB balances demand against",
                 enbCapacityMbps);
    cmd.AddValue("db-file", "SQLite data-repository output path", dbFile);
    cmd.AddValue("mode", "baseline|reactive|proactive", mode);
    cmd.AddValue("proactive-thresh", "KPI predictor RSRP threshold (dBm)", proactiveThresh);
    cmd.AddValue("ema-alpha", "EMA smoothing factor", emaAlpha);
    cmd.AddValue("pred-horizon", "Prediction horizon (cycles)", predHorizon);
    cmd.AddValue("proactive-risk-threshold",
                 "Noisy-OR conflict likelihood threshold for suppressing risky ES commands",
                 proactiveRiskThreshold);
    cmd.AddValue("proactive-rsrp-guard-db",
                 "RSRP guard band used by the proactive noisy-OR runtime-risk evidence",
                 proactiveRsrpGuardDb);
    cmd.AddValue("ground-truth",
                 "Real MRO training-data CSV output path (empty=disabled)",
                 groundTruthPath);
    cmd.AddValue("los-trace", "LOS/NLOS RSRP trace CSV output path (empty=disabled)", losTracePath);
    cmd.AddValue("use-onnx-es",
                 "Use the DQN ONNX Energy Saving LM instead of the rule-based one "
                 "(workspace/ml/es_dqn/)",
                 useOnnxEs);
    cmd.AddValue("use-onnx-mro",
                 "Use the DQN ONNX MRO/handover LM instead of the rule-based one "
                 "(workspace/ml/mro_dqn/)",
                 useOnnxMro);
    cmd.AddValue("use-onnx-mlb",
                 "Use the DQN ONNX MLB LM instead of the rule-based one "
                 "(workspace/ml/mlb_dqn/)",
                 useOnnxMlb);
    cmd.AddValue("use-onnx-cco",
                 "Use the DQN ONNX CCO LM instead of the rule-based one "
                 "(workspace/ml/cco_dqn/); ONNX CCO is TxPower-only, no RET",
                 useOnnxCco);
    cmd.Parse(argc, argv);
    g_pingPongWindowSec = pingPongWindowSec;

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));

    if (!kpiCsvPath.empty())
    {
        g_kpiCsvFile.open(kpiCsvPath);
        g_kpiCsvFile << "time_s,enb_e2id,cell_id,"
                       "txp_dbm,cio_db,ttt_ms,hys_db,ret_deg,"
                       "n_ues,min_rsrp_dbm,avg_sinr_db,cdr,hsr,tl,ee_bits_per_j,cell_throughput_mbps,"
                       "demand_mbps,capacity_mbps,demand_capacity_ratio,"
                       "ho_attempts,ho_ok,ho_fail,ping_pong,conn_timeouts,"
                       "puf_txp,puf_cio,puf_ret,puf_ttt,"
                       "txp_by,cio_by,ret_by,ttt_by,"
                       "conflict_type,conflict_icp,conflicting_xapps,affected_xapps,winner_xapp,"
                       "net_throughput_mbps,net_worst_rsrp_dbm,net_avg_rsrp_dbm,net_cdr,net_hsr,net_status,"
                       "thr_rsrp_dbm,thr_sinr_db,thr_cdr,thr_hsr,thr_tl,thr_ee,"
                       "rsrp_viol,sinr_viol,cdr_viol,hsr_viol,tl_viol,ee_viol,"
                       "numerology,bandwidth_mhz,carrier_freq_ghz\n";
    }

    LogComponentEnable("OranLmNr2NrCoverageCapacityOptimization",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranCmmConflictTriageNr", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    // ── Topology ──────────────────────────────────────────────────────────────
    std::vector<Vector> sites = LoadPositions(enbPosFile, maxEnbs);
    uint32_t nEnbs = sites.size();
    uint32_t nUes = nEnbs * nUesPerEnb;

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

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(Seconds(1.0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(Seconds(1.0)));
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    // InitializeOperationBand() is what actually creates each BWP's
    // ThreeGppPropagationLossModel (and its ThreeGppChannelConditionModel),
    // unlike LTE where the equivalent object is only created lazily on
    // InstallEnbDevice() -- so it's already safe to fetch here (see
    // NrHelper::InitializeOperationBand in workspace/nr/helper/nr-helper.cc).
    // Unconditional: used by CollectGroundTruth, --los-trace, and
    // OranLmNr2NrOnnxMro's real LOS/NLOS feature.
    {
        Ptr<ThreeGppPropagationLossModel> tgppPlm =
            DynamicCast<ThreeGppPropagationLossModel>(band.m_cc[0]->m_bwp[0]->m_propagation);
        NS_ABORT_MSG_IF(!tgppPlm, "Expected a ThreeGppPropagationLossModel on the NR BWP");
        g_channelConditionModel = tgppPlm->GetChannelConditionModel();
    }

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
    NodeContainer remoteHosts;
    remoteHosts.Create(1);
    Ptr<Node> remoteHost = remoteHosts.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHosts);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevs = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevs);
    Ipv4StaticRoutingHelper ipv4Routing;
    Ptr<Ipv4StaticRouting> rhRoute = ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    rhRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(nEnbs);
    ueNodes.Create(nUes);

    Ptr<ListPositionAllocator> gnbPos = CreateObject<ListPositionAllocator>();
    for (const auto& v : sites)
        gnbPos->Add(Vector(v.x, v.y, 20.0));
    MobilityHelper gnbMob;
    gnbMob.SetPositionAllocator(gnbPos);
    gnbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMob.Install(gnbNodes);

    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Stream", IntegerValue(42));
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    const double twoPi = 6.283185307179586;
    for (uint32_t i = 0; i < nUes; i++)
    {
        const Vector& c = sites[i % nEnbs];
        double r = 60.0 * std::sqrt(rng->GetValue());
        double theta = twoPi * rng->GetValue();
        uePos->Add(Vector(c.x + r * std::cos(theta), c.y + r * std::sin(theta), 1.5));
    }
    MobilityHelper ueMob;
    ueMob.SetPositionAllocator(uePos);
    ueMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMob.Install(ueNodes);

    const double ueSpeedScale = ueSpeed / 3.0;
    const Time kinematicUpdateInterval = Seconds(2.0);
    Ptr<NormalRandomVariable> speedNoiseRv = CreateObject<NormalRandomVariable>();
    Ptr<NormalRandomVariable> headingNoiseRv = CreateObject<NormalRandomVariable>();
    for (uint32_t i = 0; i < nUes; i++)
    {
        std::size_t profileIdx = i % g_ueModeProfiles.size();
        const UeModeProfile& p = g_ueModeProfiles[profileIdx];

        UeMobilityState state;
        state.profileIdx = profileIdx;
        state.speed = ueSpeedScale * (p.vMin + (p.vMax - p.vMin) * rng->GetValue());
        state.heading = twoPi * rng->GetValue();
        g_ueMobState[i] = state;
        g_ueMode[i] = p.name;
        g_ueTrafficType[i] = DrawTrafficTypeIndex(rng);

        Ptr<ConstantVelocityMobilityModel> mob =
            ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(
            Vector(state.speed * std::cos(state.heading), state.speed * std::sin(state.heading), 0.0));

        Simulator::Schedule(kinematicUpdateInterval,
                            &UpdateUeKinematics,
                            i,
                            mob,
                            speedNoiseRv,
                            headingNoiseRv,
                            kinematicUpdateInterval);
    }

    // NR devices
    g_gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(g_gnbDevs, randomStream);

    for (uint32_t i = 0; i < g_gnbDevs.GetN(); i++)
    {
        nrHelper->GetGnbPhy(g_gnbDevs.Get(i), 0)->SetAttribute("Numerology",
                                                               UintegerValue(numerology));
        nrHelper->GetGnbPhy(g_gnbDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(txPower));
    }
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    randomStream += nrHelper->AssignStreams(ueDevs, randomStream);

    nrHelper->UpdateDeviceConfigs(g_gnbDevs);
    nrHelper->UpdateDeviceConfigs(ueDevs);
    nrHelper->AddX2Interface(gnbNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIps = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        ipv4Routing.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>())
            ->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    for (uint32_t u = 0; u < nUes; u++)
        nrHelper->AttachToGnb(ueDevs.Get(u), g_gnbDevs.Get(u % nEnbs));

    // Energy models
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1e7));
    srcHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    energy::EnergySourceContainer eSources = srcHelper.Install(gnbNodes);
    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(g_gnbDevs.Get(i));
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(eSources.Get(i));
        Ptr<OranNrRuDeviceEnergyModel> dem = CreateObject<OranNrRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetNrGnbPhy(dev->GetPhy(0));
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
        g_gnbEnergyModels.push_back(dem);
        g_gnbInitialEnergyJ.push_back(src->GetInitialEnergy());
    }

    // UDP DL traffic -- per-UE eMBB/URLLC/mMTC/V2X profile
    for (uint32_t u = 0; u < nUes; u++)
    {
        const TrafficTypeProfile& tp = g_trafficProfiles[g_ueTrafficType[u]];
        double rateMbps = tp.minMbps + (tp.maxMbps - tp.minMbps) * rng->GetValue();

        uint16_t port = 9000 + u;
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sink.Install(ueNodes.Get(u));
        Ptr<NrUeNetDevice> sinkUeDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(u));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxBytes));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&RxByCell, u, sinkUeDev));
        sinkApp.Start(Seconds(1.0));
        sinkApp.Stop(Seconds(simTime));

        Ptr<OnOffApplication> src = CreateObject<OnOffApplication>();
        src->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIps.GetAddress(u), port)));
        src->SetAttribute("DataRate", DataRateValue(DataRate(static_cast<uint64_t>(rateMbps * 1e6))));
        src->SetAttribute("PacketSize", UintegerValue(tp.packetSize));
        src->SetAttribute("OnTime", StringValue(tp.onTimeExpr));
        src->SetAttribute("OffTime", StringValue(tp.offTimeExpr));
        remoteHost->AddApplication(src);
        src->SetStartTime(Seconds(2.0));
        src->SetStopTime(Seconds(simTime - 1.0));

        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(u),
                                             NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
                                             Create<NrEpcTft>());
    }

    // ── Near-RT RIC with all four xApps (rule-based only, per ONNX-deferral) ──
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose", BooleanValue(false));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(Seconds(lmInterval)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));
    std::remove(dbFile.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite", "DatabaseFile", StringValue(dbFile));

    if (calibrateTxpLag)
    {
        // Diagnostic-only path: no real xApps, no mitigation -- just a known,
        // large-amplitude TxPower square wave on every gNB, so the
        // parameter-change-to-KPI-effect lag can be measured directly from
        // the resulting KPI CSV without any confounding from real xApp
        // decision logic or CMM intervention.
        oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrTxpCalibration",
                                          "LowDbm", DoubleValue(calibLowDbm),
                                          "HighDbm", DoubleValue(calibHighDbm),
                                          "PeriodCycles", UintegerValue(calibPeriodCycles));
        oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr",
                                                 "Method", StringValue("noop"));
    }
    else
    {
    // Default LM: MRO (handover-based). --use-onnx-mro swaps in the DQN ONNX
    // policy (workspace/ml/mro_dqn/) instead of the rule-based RSRP handover.
    if (useOnnxMro)
    {
        oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrOnnxMro");
    }
    else
    {
        oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrRsrpHandover",
                                          "HandoverHoldoffSec",
                                          DoubleValue(1.5),
                                          "RsrpHysteresisDb",
                                          DoubleValue(2.0),
                                          "EnableCellControlBias",
                                          BooleanValue(true));
    }

    // ES: --use-onnx-es swaps in the DQN ONNX policy (workspace/ml/es_dqn/).
    if (useOnnxEs)
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrOnnxEnergySaving",
                                  "OnnxModelPath",
                                  StringValue("es_dqn_nr.onnx"),
                                  "NominalTxPowerDbm",
                                  DoubleValue(txPower));
    }
    else
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrEnergySaving",
                                  "TargetPowerW",
                                  DoubleValue(esTargetW),
                                  "StepSize",
                                  DoubleValue(esStepDb),
                                  "LmIntervalSec",
                                  DoubleValue(lmInterval));
    }

    // MLB: --use-onnx-mlb swaps in the DQN ONNX policy (workspace/ml/mlb_dqn/);
    // it still issues CIO (not handovers) so CDC keeps recognizing it as MLB.
    if (useOnnxMlb)
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrOnnxMlb",
                                  "DqnPath",
                                  StringValue("mlb_dqn_nr.onnx"),
                                  "CioStep",
                                  DoubleValue(1.0),
                                  "MaxAbsCio",
                                  DoubleValue(6.0));
    }
    else
    {
        oranHelper->AddLogicModule("ns3::OranLmNr2NrMobilityLoadBalancing",
                                  "LoadImbalanceThreshold",
                                  DoubleValue(mlbThresh),
                                  "CioStep",
                                  DoubleValue(1.0),
                                  "MaxAbsCio",
                                  DoubleValue(6.0),
                                  "EnbCapacityMbps",
                                  DoubleValue(enbCapacityMbps));
    }

    if (mode == "baseline")
    {
        // --use-onnx-cco swaps in the DQN ONNX policy (workspace/ml/cco_dqn/)
        // instead; it is TxPower-only (no RET), which removes the Indirect
        // RET->MRO conflict path in this mode too.
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrOnnxCco",
                                      "DqnPath",
                                      StringValue("cco_dqn_nr.onnx"),
                                      "NominalTxPowerDbm",
                                      DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrCoverageCapacityOptimization",
                                      "LowRsrpThresholdDbm",
                                      DoubleValue(ccoRsrpDbm),
                                      "LowRsrpFractionThreshold",
                                      DoubleValue(ccoFracThr),
                                      "StepSize",
                                      DoubleValue(1.0),
                                      "MinSamplesPerCell",
                                      UintegerValue(1),
                                      "CriticalRsrpThresholdDbm",
                                      DoubleValue(ccoCritDbm),
                                      "CriticalFractionThreshold",
                                      DoubleValue(ccoCritFrac),
                                      "RetStepDeg",
                                      DoubleValue(ccoRetStep));
        }
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("noop"));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("noop"), "ConflictLogFile",
                                                     StringValue(conflictLog));
    }
    else if (mode == "reactive")
    {
        // --use-onnx-cco swaps in the DQN ONNX policy instead (TxPower-only,
        // no RET -> no Indirect RET->MRO conflict in this mode either).
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrOnnxCco",
                                      "DqnPath",
                                      StringValue("cco_dqn_nr.onnx"),
                                      "NominalTxPowerDbm",
                                      DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrCoverageCapacityOptimization",
                                      "LowRsrpThresholdDbm",
                                      DoubleValue(ccoRsrpDbm),
                                      "LowRsrpFractionThreshold",
                                      DoubleValue(ccoFracThr),
                                      "StepSize",
                                      DoubleValue(1.0),
                                      "MinSamplesPerCell",
                                      UintegerValue(1),
                                      "CriticalRsrpThresholdDbm",
                                      DoubleValue(ccoCritDbm),
                                      "CriticalFractionThreshold",
                                      DoubleValue(ccoCritFrac),
                                      "RetStepDeg",
                                      DoubleValue(ccoRetStep));
        }
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("qacm"));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("qacm"), "ConflictLogFile",
                                                     StringValue(conflictLog));
    }
    else // proactive
    {
        // --use-onnx-cco swaps in the DQN ONNX policy instead (also TxPower-only).
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrOnnxCco",
                                      "DqnPath",
                                      StringValue("cco_dqn_nr.onnx"),
                                      "NominalTxPowerDbm",
                                      DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule("ns3::OranLmNr2NrCoverageCapacityOptimization",
                                      "LowRsrpThresholdDbm",
                                      DoubleValue(ccoRsrpDbm),
                                      "LowRsrpFractionThreshold",
                                      DoubleValue(ccoFracThr),
                                      "StepSize",
                                      DoubleValue(1.0),
                                      "MinSamplesPerCell",
                                      UintegerValue(1),
                                      "CriticalRsrpThresholdDbm",
                                      DoubleValue(ccoCritDbm),
                                      "CriticalFractionThreshold",
                                      DoubleValue(ccoCritFrac),
                                      "RetStepDeg",
                                      DoubleValue(0.0));
        }

        oranHelper->AddLogicModule("ns3::OranLmNr2NrKpiPrediction",
                                  "ProactiveThresholdDbm",
                                  DoubleValue(proactiveThresh),
                                  "ReactiveThresholdDbm",
                                  DoubleValue(-115.0),
                                  "EmaAlpha",
                                  DoubleValue(emaAlpha),
                                  "StepSizeDb",
                                  DoubleValue(1.0),
                                  "PredictionHorizon",
                                  UintegerValue(predHorizon),
                                  "MinRsrpSamples",
                                  UintegerValue(1));
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("proactive-gate"),
                                                     "ProactiveRiskThreshold",
                                                     DoubleValue(proactiveRiskThreshold),
                                                     "ProactiveRsrpGuardDb",
                                                     DoubleValue(proactiveRsrpGuardDb),
                                                     "ProactiveCellCapacityMbps",
                                                     DoubleValue(enbCapacityMbps),
                                                     "PredictionHorizonCycles",
                                                     UintegerValue(predHorizon),
                                                     "LmIntervalSec",
                                                     DoubleValue(lmInterval));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr", "Method",
                                                     StringValue("proactive-gate"),
                                                     "ProactiveRiskThreshold",
                                                     DoubleValue(proactiveRiskThreshold),
                                                     "ProactiveRsrpGuardDb",
                                                     DoubleValue(proactiveRsrpGuardDb),
                                                     "ProactiveCellCapacityMbps",
                                                     DoubleValue(enbCapacityMbps),
                                                     "PredictionHorizonCycles",
                                                     UintegerValue(predHorizon),
                                                     "LmIntervalSec",
                                                     DoubleValue(lmInterval),
                                                     "ConflictLogFile",
                                                     StringValue(conflictLog));
    }
    } // else (!calibrateTxpLag)

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    Ptr<OranCmmConflictTriageNr> triage =
        DynamicCast<OranCmmConflictTriageNr>(nearRtRic->GetCmm());

    // If --use-onnx-mro is active, wire the real ChannelConditionModel in so its
    // RSRP regressor sees actual LOS/NLOS state instead of a hidden variable it
    // has no way to predict (see OranLmNr2NrOnnxMro::SetChannelConditionModel).
    // Per-gNB/per-UE mobility gets wired in below, inside their E2 terminator
    // loops, once each one's E2 node ID actually exists.
    Ptr<OranLmNr2NrOnnxMro> mroLm =
        DynamicCast<OranLmNr2NrOnnxMro>(nearRtRic->GetDefaultLogicModule());
    if (mroLm)
    {
        mroLm->SetChannelConditionModel(g_channelConditionModel);
    }

    std::ofstream groundTruthFile;
    if (!groundTruthPath.empty())
    {
        groundTruthFile.open(groundTruthPath);
        groundTruthFile << "time,ue_idx,mode,velocity,accel,jerk,bearing_rate,pos_x,pos_y,"
                          "rsrp_reports,los_reports\n";
        // Fixed 2s cadence, independent of --lm-interval: ground-truth collection
        // must keep working even when --lm-interval is set very large (e.g. to
        // hold TxPower fixed during a PTX-sweep calibration run for CCO).
        Simulator::Schedule(Seconds(2.0),
                            &CollectGroundTruth,
                            nearRtRic->Data(),
                            ueNodes,
                            Seconds(2.0),
                            &groundTruthFile);
    }

    if (!losTracePath.empty())
    {
        g_losTraceFile.open(losTracePath);
        g_losTraceFile << "time_s,ue_idx,cell_id,los_state,rsrp_dbm,dist2d_m\n";
    }

    // ── UE E2 terminators ─────────────────────────────────────────────────────
    for (uint32_t u = 0; u < nUes; u++)
    {
        Ptr<OranReporterLocation> locRep = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> cellRep = CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranReporterNrUeRsrpRsrq> rsrpRep = CreateObject<OranReporterNrUeRsrpRsrq>();
        Ptr<OranReporterNrUeAppDemand> demandRep = CreateObject<OranReporterNrUeAppDemand>();
        Ptr<OranE2NodeTerminatorNrUe> ueTerm = CreateObject<OranE2NodeTerminatorNrUe>();

        locRep->SetAttribute("Terminator", PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));
        demandRep->SetAttribute("Terminator", PointerValue(ueTerm));

        Ptr<NrUeNetDevice> nrDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(u));
        Ptr<NrUePhy> phy = nrDev->GetPhy(0);
        phy->TraceConnectWithoutContext("ReportUeMeasurements",
                                        MakeCallback(&OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
        phy->TraceConnectWithoutContext("ReportUeMeasurements", MakeBoundCallback(&ObserveRsrp, u));
        phy->TraceConnectWithoutContext("DlDataSinr", MakeBoundCallback(&ObserveSinr, u));
        if (!losTracePath.empty())
        {
            Ptr<MobilityModel> ueMob = ueNodes.Get(u)->GetObject<MobilityModel>();
            phy->TraceConnectWithoutContext("ReportUeMeasurements",
                                            MakeBoundCallback(&ObserveLosNlosRsrp, ueMob, u));
        }

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=" +
                                         std::to_string(reportInterval) + "]"));
        ueTerm->AddReporter(locRep);
        ueTerm->AddReporter(cellRep);
        ueTerm->AddReporter(rsrpRep);
        ueTerm->AddReporter(demandRep);
        ueTerm->Attach(ueNodes.Get(u));
        Simulator::Schedule(Seconds(2.0), &OranE2NodeTerminatorNrUe::Activate, ueTerm);
        Simulator::Schedule(Seconds(2.0), &ReportUeDemand, u, demandRep, Seconds(reportInterval));
        g_e2UeTerms.Add(ueTerm);
        if (mroLm)
        {
            mroLm->SetUeMobility(ueTerm->GetE2NodeId(), ueNodes.Get(u)->GetObject<MobilityModel>());
        }
    }

    // ── gNB E2 terminators ────────────────────────────────────────────────────
    for (uint32_t i = 0; i < nEnbs; i++)
    {
        Ptr<OranReporterLocation> locRep = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrEnergyEfficiency> energyRep = CreateObject<OranReporterNrEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorNrGnb> gnbTerm = CreateObject<OranE2NodeTerminatorNrGnb>();

        locRep->SetAttribute("Terminator", PointerValue(gnbTerm));
        energyRep->SetAttribute("Terminator", PointerValue(gnbTerm));

        gnbTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        gnbTerm->SetAttribute("RegistrationIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        gnbTerm->SetAttribute("SendIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=" +
                                          std::to_string(reportInterval) + "]"));
        gnbTerm->AddReporter(locRep);
        gnbTerm->AddReporter(energyRep);
        gnbTerm->Attach(gnbNodes.Get(i));
        g_e2GnbTerms.Add(gnbTerm);
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, gnbTerm);

        uint16_t cellId = DynamicCast<NrGnbNetDevice>(g_gnbDevs.Get(i))->GetCellId();
        g_cellIdToEnbMobility[cellId] = gnbNodes.Get(i)->GetObject<MobilityModel>();
        if (mroLm)
        {
            mroLm->SetEnbMobility(gnbTerm->GetE2NodeId(), gnbNodes.Get(i)->GetObject<MobilityModel>());
        }
    }

    // ── Handover event hooks ──────────────────────────────────────────────────
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk", MakeCallback(&HandoverOk));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart", MakeCallback(&HandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureJoining",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureLeaving",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndError", MakeCallback(&HandoverFail));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionTimeout",
                    MakeCallback(&ConnectionTimeout));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/RadioLinkFailure",
                    MakeCallback(&RadioLinkFailure));

    // ── Scheduling ───────────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1.0), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    Simulator::Schedule(Seconds(lmInterval + 2.0), &LogCellState, Seconds(lmInterval), lmInterval,
                        triage, enbCapacityMbps);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    double throughputMbps = (g_rxBytesTotal * 8.0) / (simTime * 1e6);
    double totalHoAtt = g_hoOkTotal + g_hoFailTotal;
    double hsr = (totalHoAtt > 0) ? g_hoOkTotal / totalHoAtt : 1.0;
    double cdr = (totalHoAtt > 0) ? g_hoFailTotal / totalHoAtt : 0.0;

    std::cout << "\nRESULT: mode=" << mode << " enbs=" << nEnbs << " ues=" << nUes
              << " throughput_mbps=" << throughputMbps << " rsrp_viol_cycles=" << g_rsrpViolCycles
              << " lm_cycles=" << g_cycleCount << " ho_ok=" << g_hoOkTotal
              << " ho_fail=" << g_hoFailTotal << " rlf=" << g_rlfTotal
              << " ping_pong=" << g_pingPongTotal << " conn_timeouts=" << g_connTimeoutsTotal
              << " HSR=" << hsr << " CDR=" << cdr << std::endl;

    if (g_kpiCsvFile.is_open())
    {
        g_kpiCsvFile.close();
    }
    if (g_losTraceFile.is_open())
    {
        g_losTraceFile.close();
    }

    Simulator::Destroy();
    return 0;
}
