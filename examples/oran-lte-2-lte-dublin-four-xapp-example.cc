/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Dublin Four-xApp Conflict Dataset Example (Chapter 6, PhD Thesis).
 *
 * Deploys all four O-RAN xApps on the Dublin-Three topology (first 3 eNBs
 * from workspace/data/ns3_positions_Three_IE.txt):
 *
 *   ES  — Energy Saving: reduces TxPower toward target (conflict source)
 *   CCO — Coverage/Capacity Optimisation: raises TxPower + adjusts RET
 *           → RET change creates Indirect conflict with MRO (CDR/HSR)
 *   MRO — Mobility Robustness Optimisation: RSRP-based handover management
 *   MLB — Mobility Load Balancing: CIO adjustment
 *
 * Conflict taxonomy observable in output:
 *   Direct   — ES + CCO both issue TxPower for same eNB (same ICP, multiple owners)
 *   Indirect — CCO issues RET; MRO manages CDR/HSR; MRO ∉ P2X[RET]
 *   Direct   — MLB issues CIO; MRO manages CDR/HSR; MRO ∈ P2X[CIO] (structural)
 *
 * Per-xApp KPI thresholds embedded in [STATE] output:
 *   ES:  EE ≥ 1e6 bits/J
 *   CCO: RSRP ≥ -95 dBm, SINR ≥ 5 dB
 *   MRO: CDR ≤ 2%, HSR ≥ 95%
 *   MLB: TL ≤ 80%
 *
 * Dataset generation:
 *   python3 ns3 run 'oran-lte-2-lte-dublin-four-xapp-example' 2>/dev/null > raw.log
 *   python3 workspace/scripts/parse_ns3_conflict_log.py --input raw.log \
 *           --mode dublin_four_xapp --out conflict_dataset.csv
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-cmm-conflict-triage.h"
#include "ns3/oran-lm-lte-2-lte-kpi-prediction.h"
#include "ns3/oran-lte-cell-control-state.h"
#include "ns3/oran-module.h"
#include "ns3/oran-ru-energy-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/propagation-module.h"

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

NS_LOG_COMPONENT_DEFINE("OranDublinFourXappExample");

// ─── Per-xApp KPI thresholds (used in [STATE] and [PMON] log lines) ──────────
static constexpr double kEeThresh    = 1e6;   // bits/J  (ES)
static constexpr double kRsrpThresh  = -95.0; // dBm     (CCO)
static constexpr double kSinrThresh  =  5.0;  // dB      (CCO)
static constexpr double kCdrThresh   = 0.02;  // fraction (MRO)
static constexpr double kHsrThresh   = 0.95;  // fraction (MRO)
static constexpr double kTlThresh    = 0.80;  // fraction (MLB)

// ─── Global KPI accumulators ──────────────────────────────────────────────────
static std::map<uint16_t, double>   g_latestUeRsrp;   // RNTI → serving RSRP (dBm)
static std::map<uint16_t, double>   g_latestUeSinr;   // RNTI → serving SINR (dB)
static std::map<uint16_t, uint16_t> g_latestUeCell;   // RNTI → serving cellId

// Per-cell handover counters (reset each LM cycle by LogCellState)
static std::map<uint16_t, uint32_t> g_hoAttempts;     // cellId → attempts this cycle
static std::map<uint16_t, uint32_t> g_hoOk;           // cellId → successes this cycle
static std::map<uint16_t, uint32_t> g_hoFail;         // cellId → failures this cycle
static std::map<uint16_t, uint32_t> g_pingPong;       // cellId (source) → ping-pongs this cycle
static std::map<uint16_t, uint32_t> g_connTimeouts;   // cellId → connection timeouts this cycle

// Ping-pong detection: an A→B handover followed by a B→A handover for the
// same UE within g_pingPongWindowSec counts as one ping-pong, attributed to
// the first handover's source cell (same convention as g_hoAttempts).
struct LastHoTransition
{
    uint16_t sourceCellId = 0;
    uint16_t targetCellId = 0;
    double   timeSec      = 0.0;
};
static std::map<uint64_t, LastHoTransition> g_lastHoTransitionByImsi;
static double   g_pingPongWindowSec = 5.0;
static uint32_t g_pingPongTotal     = 0;

// Per-cell cumulative byte counters, keyed by the UE's REAL current serving
// cellId (queried live off the UE's own RRC at each Rx event via RxByCell
// below), not reset between cycles. Replaces an earlier `u % nEnbs` guess
// that assumed creation-order alignment with eNBs -- harmless while every
// UE had identical demand, but wrong once demand varies per UE/cell and
// wrong for any UE that's ever handed over.
static std::map<uint16_t, uint64_t> g_rxBytesByCellId;
static std::map<uint16_t, uint64_t> g_rxBytesByCellIdLastCycle; // snapshot for per-cell delta

// Per-UE cumulative bytes, for the periodic real-demand reports fed to the
// O-RAN data repository (OranReporterLteUeAppDemand) that bandwidth-aware
// MLB reads.
static std::map<uint32_t, uint64_t> g_ueRxBytesTotal;
static std::map<uint32_t, uint64_t> g_ueRxBytesLastReport;

static std::ofstream g_kpiCsvFile; // combined per-cycle, per-eNB parameter+KPI CSV

// Cumulative
static uint32_t g_hoOkTotal       = 0;
static uint32_t g_hoFailTotal     = 0;
static uint32_t g_rlfTotal        = 0;
static uint32_t g_connTimeoutsTotal = 0;
static uint64_t g_rxBytesTotal    = 0;
static uint64_t g_rxBytesLastCycle = 0;   // snapshot at end of last cycle for delta
static uint32_t g_rsrpViolCycles  = 0;    // number of LM cycles with worst RSRP < threshold

// eNB devices and terminators (global for LogCellState access)
static NetDeviceContainer            g_enbDevs;
static OranE2NodeTerminatorContainer g_e2EnbTerms;
static OranE2NodeTerminatorContainer g_e2UeTerms; //!< index-aligned with the UE creation loop
static std::vector<Ptr<OranRuDeviceEnergyModel>> g_enbEnergyModels;
static std::vector<double>           g_enbInitialEnergyJ;

static uint32_t g_cycleCount = 0;

// ─── UE mobility-type diversity ───────────────────────────────────────────
// Real per-UE mobility profiles (speed range, heading/speed noise), so
// MRO's mode classifier (workspace/ml/mro_dqn/) has something genuine to
// discriminate -- previously all UEs used a single ConstantVelocityMobility
// draw set once at t=0 and never updated, so real acceleration/jerk/bearing
// -rate were always exactly 0 regardless of "mode". Speed bands are
// non-overlapping (ascending: PED<CYCLIST<DRONE<BUS<CAR<UAV<TRAIN) and must
// match MODE_PARAMS in workspace/ml/mro_dqn/train_mro.py exactly, since
// that's the single source of truth for training data generation.
struct UeModeProfile
{
    std::string name;
    double vMin;             //!< m/s, before --ue-speed scaling
    double vMax;             //!< m/s, before --ue-speed scaling
    double headingNoiseStd;  //!< rad, std-dev of heading drift per update
    double speedNoiseFrac;   //!< fraction of (vMax-vMin) as speed-walk std per update
};

static const std::vector<UeModeProfile> g_ueModeProfiles = {
    {"PED",     0.8,  2.0,  0.35, 0.08},
    {"CYCLIST", 2.5,  5.0,  0.25, 0.06},
    {"DRONE",   5.5,  8.5,  0.30, 0.10},
    {"BUS",     9.0,  14.0, 0.05, 0.03},
    {"CAR",     15.0, 22.0, 0.12, 0.05},
    {"UAV",     23.0, 26.0, 0.20, 0.08},
    {"TRAIN",   27.0, 40.0, 0.02, 0.02},
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

static std::map<uint32_t, UeMobilityState> g_ueMobState; //!< UE index -> live kinematic state
static std::map<uint32_t, std::string>     g_ueMode;     //!< UE index -> assigned mode name

// ─── UE traffic-type diversity ────────────────────────────────────────────
// Real application-layer demand per UE, so bandwidth-capacity-aware MLB has
// something genuine to balance against (previously every UE ran identical
// 1 Mbps constant-on UDP, so aggregate cell demand was just a function of UE
// count -- no different from the old count-based balancing it was meant to
// replace). Weights are a rough 5G/B5G traffic-mix assumption (eMBB-heavy,
// smaller URLLC/mMTC/V2X slices), assigned independent of mobility mode, not
// correlated with it (confirmed choice). Rates are representative, not
// standards-derived -- flag alongside the QoS thresholds if exact figures
// need citing later.
struct TrafficTypeProfile
{
    std::string name;
    double weight;   //!< fraction of UEs assigned this type (weights sum to 1.0)
    double minMbps;
    double maxMbps;
    uint32_t packetSize;
    std::string onTimeExpr;  //!< ns3::RandomVariableStream config string
    std::string offTimeExpr; //!< ns3::RandomVariableStream config string
};

static const std::vector<TrafficTypeProfile> g_trafficProfiles = {
    // eMBB: high-throughput, mostly-on bursty (video/web-like).
    {"eMBB",  0.50, 5.0,  15.0, 1200, "ns3::ExponentialRandomVariable[Mean=4.0]",
                                     "ns3::ExponentialRandomVariable[Mean=1.0]"},
    // URLLC: low throughput but small, frequent, tightly-spaced packets
    // (critical low-latency messages).
    {"URLLC", 0.20, 0.5,  2.0,  150,  "ns3::ExponentialRandomVariable[Mean=0.05]",
                                     "ns3::ExponentialRandomVariable[Mean=0.05]"},
    // mMTC: very low rate, sparse (long idle, short report) IoT-style.
    {"mMTC",  0.20, 0.01, 0.1,  100,  "ns3::ExponentialRandomVariable[Mean=0.5]",
                                     "ns3::ExponentialRandomVariable[Mean=10.0]"},
    // V2X: moderate throughput, frequent periodic small-to-medium bursts
    // (status/safety messages).
    {"V2X",   0.10, 1.0,  5.0,  300,  "ns3::ExponentialRandomVariable[Mean=0.2]",
                                     "ns3::ExponentialRandomVariable[Mean=0.3]"},
};

static std::map<uint32_t, std::size_t> g_ueTrafficType; //!< UE index -> g_trafficProfiles index

/** Weighted draw (not round-robin) over g_trafficProfiles, independent of mobility mode. */
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
    return g_trafficProfiles.size() - 1; // floating-point safety net
}

// eNB mobility + shared channel condition model instance actually driving the
// downlink path loss -- used by both CollectGroundTruth (real LOS/NLOS logging)
// and the LOS/NLOS RSRP trace further below (so IsLos() there reflects the
// real state used for that RSRP sample, not a separate stochastic draw).
static std::map<uint16_t, Ptr<MobilityModel>> g_cellIdToEnbMobility;
static Ptr<ChannelConditionModel> g_channelConditionModel;

// Self-rescheduling per-UE kinematic update (idiom matches LogCellState's
// self-reschedule below): random-walks speed within the mode's band and
// drifts heading, producing genuine non-zero acceleration/jerk/bearing-rate
// that differs by mode, then applies it via SetVelocity.
void
UpdateUeKinematics(uint32_t ueIdx,
                   Ptr<ConstantVelocityMobilityModel> mob,
                   Ptr<NormalRandomVariable> speedNoiseRv,
                   Ptr<NormalRandomVariable> headingNoiseRv,
                   Time updateInterval)
{
    UeMobilityState& s = g_ueMobState[ueIdx];
    const UeModeProfile& p = g_ueModeProfiles[s.profileIdx];

    // NormalRandomVariable::GetValue takes (mean, VARIANCE, bound), not std-dev.
    // Clipped strictly to [vMin, vMax] (not vMin*0.5/vMax*1.2 as originally) --
    // that headroom let adjacent mode bands bleed into each other over a long
    // random walk (e.g. CAR drifting up to 26.4 m/s, indistinguishable from
    // UAV's nominal 23-26 m/s band), which was a real, avoidable contributor
    // to mode-classifier confusion, not just a data-volume problem.
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
// differenced from noisy position reports like OranLmLte2LteOnnxMro has to at
// inference time), true mode label, position, and real per-eNB RSRP (via the
// same OranDataRepository::GetLteUeRsrpRsrq accessor the ONNX LMs use, so this
// reflects the actual 3GPP channel model already running in this scenario).
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
        // Semicolon-separated within this one CSV field (commas would break column
        // alignment for downstream CSV parsers, since the report count varies per row).
        bool first = true;
        for (const auto& tup : data->GetLteUeRsrpRsrq(ueE2Id))
        {
            if (!first)
            {
                (*out) << ";";
            }
            (*out) << std::get<1>(tup) << ":" << std::get<2>(tup);
            first = false;
        }

        // Real LOS/NLOS per (UE, eNB), from the same ChannelConditionModel that
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

// ─── Utility: load eNB positions from workspace file ─────────────────────────
static std::vector<Vector>
LoadPositions(const std::string& path, uint32_t maxEnbs)
{
    std::ifstream in(path);
    NS_ABORT_MSG_IF(!in.is_open(), "Cannot open eNB position file: " << path);
    std::vector<Vector> pos;
    std::string line;
    std::regex rx(R"(Vector\s*\(\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+)\s*,\s*([-+]?\d*\.?\d+))");
    std::smatch m;
    while (std::getline(in, line))
    {
        if (std::regex_search(line, m, rx))
            pos.emplace_back(std::stod(m[1]), std::stod(m[2]), std::stod(m[3]));
        if (pos.size() >= maxEnbs)
            break;
    }
    NS_ABORT_MSG_IF(pos.empty(), "No positions found in " << path);
    return pos;
}

// ─── Trace callbacks ──────────────────────────────────────────────────────────
// ReportCurrentCellRsrpSinr fires as (cellId, rnti, rsrp_W, sinr_linear, ccId)
void
ObserveRsrpSinr(uint16_t cellId,
                uint16_t rnti,
                double rsrpW,
                double sinrLin,
                uint8_t /*ccId*/)
{
    g_latestUeRsrp[rnti] = 10.0 * std::log10(std::max(rsrpW, 1e-30) * 1000.0); // W → dBm
    g_latestUeSinr[rnti] = 10.0 * std::log10(std::max(sinrLin, 1e-12));          // linear → dB
    g_latestUeCell[rnti] = cellId;
}

// Bound to (ueMobility, ueIdx) per UE at connection time (see the UE E2 terminator
// loop below), so we get the serving UE's own mobility model directly instead of
// reverse-mapping it from rnti. Queries the *same* ChannelConditionModel instance
// that ThreeGppUmiStreetCanyonPropagationLossModel used to compute this RSRP
// sample's path loss, so the logged LOS/NLOS state is the one that was actually
// applied, not an independent re-roll.
void
ObserveLosNlosRsrp(Ptr<MobilityModel> ueMob,
                   uint32_t ueIdx,
                   uint16_t cellId,
                   uint16_t /*rnti*/,
                   double rsrpW,
                   double /*sinrLin*/,
                   uint8_t /*ccId*/)
{
    if (!g_channelConditionModel)
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
    double rsrpDbm = 10.0 * std::log10(std::max(rsrpW, 1e-30) * 1000.0);

    Vector enbPos = it->second->GetPosition();
    Vector uePos  = ueMob->GetPosition();
    double dist2d = std::sqrt(std::pow(enbPos.x - uePos.x, 2) + std::pow(enbPos.y - uePos.y, 2));

    g_losTraceFile << std::fixed << std::setprecision(3)
                   << Simulator::Now().GetSeconds() << ","
                   << ueIdx << ","
                   << cellId << ","
                   << (isLos ? "LOS" : "NLOS") << ","
                   << rsrpDbm << ","
                   << dist2d << "\n";
}

void
RxBytes(Ptr<const Packet> pkt, const Address& /*addr*/)
{
    g_rxBytesTotal += pkt->GetSize();
}

void
RxByCell(uint32_t ueIdx, Ptr<LteUeNetDevice> ueDev, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    // Live lookup, not a stored/stale map -- always reflects the UE's actual
    // serving cell at the moment of reception, correct across handovers.
    uint16_t cellId = ueDev->GetRrc()->GetCellId();
    uint32_t size = pkt->GetSize();
    g_rxBytesByCellId[cellId] += size;
    g_ueRxBytesTotal[ueIdx] += size;
}

// Self-rescheduling per-UE demand reporter: converts the cumulative byte
// counter RxByCell maintains into a per-interval Mbps figure and pushes it
// into the O-RAN data repository via the UE's OranReporterLteUeAppDemand,
// so bandwidth-capacity-aware MLB can read real per-UE demand instead of
// just counting UEs.
void
ReportUeDemand(uint32_t ueIdx, Ptr<OranReporterLteUeAppDemand> reporter, Time interval)
{
    uint64_t total = g_ueRxBytesTotal.count(ueIdx) ? g_ueRxBytesTotal[ueIdx] : 0;
    uint64_t last  = g_ueRxBytesLastReport.count(ueIdx) ? g_ueRxBytesLastReport[ueIdx] : 0;
    g_ueRxBytesLastReport[ueIdx] = total;

    double mbps = ((total - last) * 8.0) / (interval.GetSeconds() * 1e6);
    reporter->ReportDemand(mbps);

    Simulator::Schedule(interval, &ReportUeDemand, ueIdx, reporter, interval);
}

// HandoverStart (below) records each UE's pending attempt keyed by IMSI,
// with its SOURCE cell -- g_hoAttempts is keyed by that same source cell.
// But HandoverEndOk fires at whichever eNB completes the procedure (the
// TARGET), and the four HandoverFailure* traces are a mix of target-keyed
// (Joining/MaxRach/NoPreamble, which fire at the target eNB receiving the
// UE) and source-keyed (Leaving, which fires at the source eNB waiting for
// the UE to leave). Naively keying g_hoOk/g_hoFail by whatever cellId each
// individual trace happens to report -- as this code originally did --
// mixes source- and target-cell populations into the SAME per-cell
// counter, so ok/fail for a given cell aren't measuring the same set of
// attempts g_hoAttempts[cell] counted. That's exactly how hsr = ok/attempts
// could exceed 1 (a cell that's a common handover TARGET can rack up more
// "ok" than it ever logged as an attempting SOURCE) even with fail == 0.
//
// Fix: attribute every outcome (ok/fail) back to the ORIGINAL source cell
// of the matching attempt, looked up by IMSI from the same
// g_lastHoTransitionByImsi map HandoverStart already populates for
// ping-pong detection -- so attempts/ok/fail are all consistently keyed by
// "the cell this handover was attempted FROM." Falls back to the trace's
// own reported cellId only if no pending attempt is on record (shouldn't
// normally happen, but keeps behavior sane rather than silently dropping
// the event). Note this can still misattribute if the same UE starts a
// second handover before the first one's outcome arrives (overwriting the
// map entry) -- rare, and far better than the unconditional source/target
// mismatch this replaces.
uint16_t
SourceCellForOutcome(uint64_t imsi, uint16_t fallbackCellId)
{
    auto it = g_lastHoTransitionByImsi.find(imsi);
    return (it != g_lastHoTransitionByImsi.end()) ? it->second.sourceCellId : fallbackCellId;
}

void
HandoverOk(std::string /*ctx*/, uint64_t imsi, uint16_t targetCellId, uint16_t /*rnti*/)
{
    g_hoOkTotal++;
    g_hoOk[SourceCellForOutcome(imsi, targetCellId)]++;
}

void
HandoverStart(std::string /*ctx*/, uint64_t imsi, uint16_t srcCell,
              uint16_t /*rnti*/, uint16_t targetCell)
{
    g_hoAttempts[srcCell]++;

    const double now = Simulator::Now().GetSeconds();
    auto it = g_lastHoTransitionByImsi.find(imsi);
    if (it != g_lastHoTransitionByImsi.end() &&
        it->second.sourceCellId == targetCell &&
        it->second.targetCellId == srcCell &&
        (now - it->second.timeSec) <= g_pingPongWindowSec)
    {
        g_pingPongTotal++;
        g_pingPong[srcCell]++;
    }
    g_lastHoTransitionByImsi[imsi] = {srcCell, targetCell, now};
}

void
HandoverFail(std::string /*ctx*/, uint64_t imsi, uint16_t cellId, uint16_t /*rnti*/)
{
    g_hoFailTotal++;
    g_hoFail[SourceCellForOutcome(imsi, cellId)]++;
}

// LteEnbRrc's four HandoverFailure* trace sources (MaxRach, NoPreamble, Joining,
// Leaving) all fire as (imsi, rnti, cellId) -- see the m_handoverFailure*Trace(...)
// call sites in src/lte/model/lte-enb-rrc.cc -- which is the OPPOSITE order of
// LteUeRrc's HandoverEndError/ConnectionTimeout/RadioLinkFailure (imsi, cellId,
// rnti). Binding HandoverFail (above) directly to these four would silently
// swap rnti into the "cellId" slot, misattributing every ENB-side handover
// failure to whatever the UE's RNTI happened to be instead of its real cell --
// invisible in the per-cell CSV/[STATE] rows (since real cellIds are 1..3 and
// RNTIs rarely coincide), while still counted correctly in the network-wide
// aggregate (which just sums every g_hoFail[] entry regardless of key). That
// mismatch is exactly why net_cdr could be nonzero while every per-cell
// ho_fail column read 0 for the same cycle.
void
HandoverFailEnb(std::string /*ctx*/, uint64_t imsi, uint16_t /*rnti*/, uint16_t cellId)
{
    g_hoFailTotal++;
    g_hoFail[SourceCellForOutcome(imsi, cellId)]++;
}

void
ConnectionTimeout(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t cellId, uint16_t /*rnti*/,
                  uint8_t /*numberOfAttempts*/)
{
    g_connTimeouts[cellId]++;
    g_connTimeoutsTotal++;
}

void
RadioLinkFailure(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t /*cellId*/, uint16_t /*rnti*/)
{
    g_rlfTotal++;
}

// ─── Per-cycle cell state logger — emits [STATE] + [PMON] ───────────────────
void
LogCellState(Time interval, double lmIntervalSec, Ptr<OranCmmConflictTriage> triage,
            double enbCapacityMbps)
{
    double now = Simulator::Now().GetSeconds();
    g_cycleCount++;

    // Compute network-wide KPIs for [PMON]
    double worstRsrp = 0.0;
    uint32_t totalUes = 0;
    for (const auto& kv : g_latestUeRsrp)
    {
        if (worstRsrp == 0.0 || kv.second < worstRsrp) worstRsrp = kv.second;
        totalUes++;
    }

    uint32_t totalHoAttempts = 0, totalHoOk = 0, totalHoFail = 0;
    for (auto& kv : g_hoAttempts) totalHoAttempts += kv.second;
    for (auto& kv : g_hoOk)       totalHoOk       += kv.second;
    for (auto& kv : g_hoFail)     totalHoFail     += kv.second;

    double netCdr = (totalHoAttempts > 0)
                    ? static_cast<double>(totalHoFail) / totalHoAttempts : 0.0;
    double netHsr = (totalHoAttempts > 0)
                    ? static_cast<double>(totalHoOk)   / totalHoAttempts : 1.0;

    // Average serving RSRP across all UEs
    double sumAllRsrp = 0.0;
    uint32_t rsrpN = 0;
    for (const auto& kv : g_latestUeRsrp)
    {
        sumAllRsrp += kv.second;
        rsrpN++;
    }
    double avgRsrp = (rsrpN > 0) ? sumAllRsrp / rsrpN : 0.0;

    // Per-cycle throughput delta (Mbps since last LogCellState call)
    uint64_t deltaBytes = g_rxBytesTotal - g_rxBytesLastCycle;
    g_rxBytesLastCycle  = g_rxBytesTotal;
    double cycleThrMbps = (deltaBytes * 8.0) / (lmIntervalSec * 1e6);

    // [PMON] — network-wide quality snapshot
    bool rsrpViol = (totalUes > 0 && worstRsrp < kRsrpThresh);
    bool cdrViol  = (netCdr > kCdrThresh);
    bool hsrViol  = (totalHoAttempts > 0 && netHsr < kHsrThresh);
    if (rsrpViol) g_rsrpViolCycles++;
    std::vector<std::string> violationCauses;
    if (rsrpViol) violationCauses.push_back("RSRP");
    if (cdrViol)  violationCauses.push_back("CDR");
    if (hsrViol)  violationCauses.push_back("HSR");
    const std::string status = violationCauses.empty() ? "OK" : "VIOLATION";

    std::cout << "[PMON] t=" << std::fixed << std::setprecision(2) << now << "s"
              << " worst_RSRP=" << (totalUes > 0 ? worstRsrp : 0.0) << "dBm"
              << " avg_RSRP=" << std::setprecision(2) << (rsrpN > 0 ? avgRsrp : 0.0) << "dBm"
              << " throughput_mbps=" << std::setprecision(3) << cycleThrMbps
              << " QoS_threshold=" << kRsrpThresh << "dBm"
              << " net_CDR=" << std::setprecision(4) << netCdr
              << " net_HSR=" << netHsr
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

    // [STATE] — per-eNB snapshot with full ICP + KPI values
    for (uint32_t i = 0; i < g_enbDevs.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(g_enbDevs.Get(i));
        uint64_t e2id  = g_e2EnbTerms.Get(i)->GetE2NodeId();
        uint16_t cellId = dev->GetCellId();
        OranLteCellControlParams cp = GetLteCellControlParameters(e2id);

        // Per-cell RSRP / SINR (min RSRP, mean SINR of UEs served by this cell)
        double minRsrp = 0.0, sumSinr = 0.0;
        uint32_t nUes = 0;
        for (const auto& kv : g_latestUeRsrp)
        {
            auto cit = g_latestUeCell.find(kv.first);
            if (cit == g_latestUeCell.end() || cit->second != cellId) continue;
            if (nUes == 0 || kv.second < minRsrp) minRsrp = kv.second;
            sumSinr += g_latestUeSinr.count(kv.first) ? g_latestUeSinr[kv.first] : 0.0;
            nUes++;
        }
        double avgSinr = (nUes > 0) ? sumSinr / nUes : 0.0;

        // Per-cell handover KPIs
        uint32_t att  = g_hoAttempts.count(cellId) ? g_hoAttempts[cellId] : 0;
        uint32_t ok   = g_hoOk.count(cellId)       ? g_hoOk[cellId]       : 0;
        uint32_t fail = g_hoFail.count(cellId)      ? g_hoFail[cellId]     : 0;
        double cdr = (att > 0) ? static_cast<double>(fail) / att : 0.0;
        double hsr = (att > 0) ? static_cast<double>(ok)   / att : 1.0;

        // TL: fraction of total UEs served by this cell
        double tl = (totalUes > 0) ? static_cast<double>(nUes) / totalUes : 0.0;

        // EE: cumulative bits received by this cell's UEs / cumulative energy consumed.
        // Keyed by real cellId (via RxByCell's live RRC lookup), not creation-order index.
        uint64_t cellBytes = g_rxBytesByCellId.count(cellId) ? g_rxBytesByCellId[cellId] : 0;
        double energyConsumedJ = std::max(g_enbEnergyModels[i]->GetTotalEnergyConsumption(), 1e-9);
        double ee = (cellBytes * 8.0) / energyConsumedJ; // bits/J

        // Per-cell throughput delta (Mbps since last cycle), same style as
        // the network-wide cycleThrMbps above.
        uint64_t cellBytesLast =
            g_rxBytesByCellIdLastCycle.count(cellId) ? g_rxBytesByCellIdLastCycle[cellId] : 0;
        g_rxBytesByCellIdLastCycle[cellId] = cellBytes;
        double cellThrMbps = ((cellBytes - cellBytesLast) * 8.0) / (lmIntervalSec * 1e6);

        // Ping-pong / connection-timeout counts this cycle, this cell.
        uint32_t pingPong    = g_pingPong.count(cellId)     ? g_pingPong[cellId]     : 0;
        uint32_t connTimeout = g_connTimeouts.count(cellId) ? g_connTimeouts[cellId] : 0;

        // puf_* (param update frequency): real per-cycle command counts from
        // the CMM, keyed by ICP name -- reset on read, so each cycle reports
        // only commands issued since the previous cycle.
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

        // Which xApp role(s) instructed each parameter to change this cycle
        // (not just how many times, like puf_* above, but who) -- e.g. if
        // both ES and CCO wrote TxPower this cycle, txpBy="ES;CCO". "-"
        // means no xApp touched that parameter this cycle for this cell.
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
            std::map<std::string, std::set<std::string>> icpActors =
                triage->GetAndResetIcpActors(e2id);
            auto getActors = [&icpActors](const char* name) {
                auto it = icpActors.find(name);
                return it != icpActors.end() ? it->second : std::set<std::string>{};
            };
            txpBy = joinRoles(getActors("TxPower"));
            cioBy = joinRoles(getActors("CIO"));
            retBy = joinRoles(getActors("RET"));
            tttBy = joinRoles(getActors("TTT"));
        }

        // CDC-classified conflicts this cell had this cycle. type is always
        // Direct or Indirect (this scenario's taxonomy has no Implicit
        // category). A cell can have more than one distinct event in the
        // same cycle (e.g. a Direct TxPower conflict AND an Indirect RET
        // conflict at once) -- rather than squashing those into one joined
        // row, each event gets its OWN CSV row below, all sharing the same
        // time_s/enb_e2id/cell_id and identical KPI values, so it's
        // unambiguous which specific xApp commands caused which conflict.
        // A cell with no conflict this cycle still gets exactly one row
        // (conflict_type=None).
        std::vector<OranCmmConflictTriage::ConflictEventRecord> conflictEvents;
        if (triage)
        {
            conflictEvents = triage->GetAndResetConflictEvents(e2id);
        }

        // Per-cell, per-cycle violation flags: this row's own KPI value
        // against its corresponding threshold. Gated on nUes>0 (RSRP/SINR/EE)
        // or att>0 (CDR/HSR) so an empty/idle cell isn't flagged as violating
        // just because its default (no-data) value happens to read as "bad"
        // -- same convention as the existing network-wide rsrpViol/etc. below.
        bool rsrpViolCell = (nUes > 0 && minRsrp < kRsrpThresh);
        bool sinrViolCell = (nUes > 0 && avgSinr < kSinrThresh);
        bool cdrViolCell  = (att > 0 && cdr > kCdrThresh);
        bool hsrViolCell  = (att > 0 && hsr < kHsrThresh);
        bool tlViolCell   = (tl > kTlThresh);
        bool eeViolCell   = (nUes > 0 && ee < kEeThresh);

        std::cout << "[STATE] t=" << std::fixed << std::setprecision(2) << now << "s"
                  << " e2=" << e2id
                  << " cell=" << cellId
                  << " TXP=" << dev->GetPhy()->GetTxPower() << "dBm"
                  << " CIO=" << cp.cioDb << "dB"
                  << " TTT=" << cp.tttSec * 1000.0 << "ms"
                  << " HYS=" << cp.hysDb << "dB"
                  << " RET=" << cp.retDeg << "deg"
                  << " nUEs=" << nUes
                  << " minRSRP=" << (nUes > 0 ? minRsrp : 0.0) << "dBm"
                  << " avgSINR=" << avgSinr << "dB"
                  << " CDR=" << std::setprecision(4) << cdr
                  << " HSR=" << hsr
                  << " TL=" << tl
                  << " EE=" << std::setprecision(2) << ee
                  << " hoAttempts=" << att
                  << " puf_TXP=" << txpFreq
                  << " puf_CIO=" << cioFreq
                  << " puf_RET=" << retFreq
                  << " puf_TTT=" << tttFreq
                  // Per-xApp KPI thresholds
                  << " thr_RSRP=" << kRsrpThresh
                  << " thr_SINR=" << kSinrThresh
                  << " thr_CDR="  << kCdrThresh
                  << " thr_HSR="  << kHsrThresh
                  << " thr_TL="   << kTlThresh
                  << " thr_EE="   << kEeThresh
                  << "\n";

        // One CSV row per conflict event this cell had this cycle (or a
        // single "None" row if it had none) -- all sharing the same
        // time_s/enb_e2id/cell_id and identical KPI values, only the
        // conflict_* columns differ, so each row on its own tells you
        // exactly which xApp commands led to that specific conflict.
        auto writeCsvRow = [&](const std::string& conflictType,
                               const std::string& conflictIcp,
                               const std::string& conflictingXapps,
                               const std::string& affectedXapps,
                               const std::string& winnerXapp) {
            if (!g_kpiCsvFile.is_open())
            {
                return;
            }
            g_kpiCsvFile << now << "," << e2id << "," << cellId << ","
                        << dev->GetPhy()->GetTxPower() << "," << cp.cioDb << ","
                        << cp.tttSec * 1000.0 << "," << cp.hysDb << "," << cp.retDeg << ","
                        << nUes << "," << (nUes > 0 ? minRsrp : 0.0) << "," << avgSinr << ","
                        << cdr << "," << hsr << "," << tl << "," << ee << "," << cellThrMbps << ","
                        << cellThrMbps << "," << enbCapacityMbps << ","
                        << (cellThrMbps / enbCapacityMbps) << ","
                        << att << "," << ok << "," << fail << "," << pingPong << ","
                        << connTimeout << "," << txpFreq << "," << cioFreq << "," << retFreq
                        << "," << tttFreq << ","
                        << txpBy << "," << cioBy << "," << retBy << "," << tttBy << ","
                        << conflictType << "," << conflictIcp << "," << conflictingXapps << ","
                        << affectedXapps << "," << winnerXapp << ","
                        << cycleThrMbps << ","
                        << (totalUes > 0 ? worstRsrp : 0.0) << "," << (rsrpN > 0 ? avgRsrp : 0.0)
                        << "," << netCdr << "," << netHsr << "," << status << ","
                        << kRsrpThresh << "," << kSinrThresh << "," << kCdrThresh << ","
                        << kHsrThresh << "," << kTlThresh << "," << kEeThresh << ","
                        << rsrpViolCell << "," << sinrViolCell << "," << cdrViolCell << ","
                        << hsrViolCell << "," << tlViolCell << "," << eeViolCell << "\n";
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

    // Reset per-cycle handover/ping-pong/timeout counters (bytes/energy stay cumulative)
    g_hoAttempts.clear();
    g_hoOk.clear();
    g_hoFail.clear();
    g_pingPong.clear();
    g_connTimeouts.clear();

    Simulator::Schedule(interval, &LogCellState, interval, lmIntervalSec, triage, enbCapacityMbps);
}

// ─── Main ────────────────────────────────────────────────────────────────────
int
main(int argc, char* argv[])
{
    std::string enbPosFile       = "/workspace/data/ns3_positions_Three_IE.txt";
    std::string conflictLog      = "";           // CDC CSV path (empty = stdout only)
    std::string losTracePath     = "";           // LOS/NLOS RSRP trace CSV (empty = disabled)
    std::string kpiCsvPath       = "";           // combined per-cycle KPI/parameter CSV (empty = disabled)
    double      pingPongWindowSec = 5.0;         // A-B-A handover window counted as ping-pong
    // Fixed per-eNB capacity ceiling MLB balances demand against (uniform
    // across all eNBs -- confirmed choice, no heterogeneous macro/small-cell
    // mix for now). A placeholder default, not derived from DlBandwidth via
    // an invented physical-layer throughput formula -- same "assume for now,
    // cite properly later" treatment already agreed for the QoS thresholds.
    double      enbCapacityMbps  = 50.0;
    // Full real 13-site Dublin topology (ns3_positions_Three_IE.txt has all
    // 13 despite its name -- earlier work only loaded the first 3 via this
    // same --max-enbs flag). ONNX xApps still require exactly 3 (their
    // trained state/action shapes are fixed) -- pass --max-enbs=3 to use
    // them; rule-based xApps scale to any eNB count with no code changes.
    uint32_t    maxEnbs          = 13;           // use first N eNBs from Dublin file
    // 9/eNB (117 UEs at 13 eNBs) matches the density used in prior
    // calculations. Slower than the 78-UE case already found to take 220s+
    // wall-clock per 7 simulated seconds -- expect full-length runs to need
    // a patient background job (potentially an hour or more), not
    // foreground iteration.
    uint32_t    nUesPerEnb       = 9;
    double      simTime          = 150.0;
    // 500ms: comfortably inside the O-RAN Near-RT RIC's defining 10ms-1s
    // control-loop window (was 5.0, Non-RT territory). Kept equal to the E2
    // SendIntervalRv below so the RIC's decisions are never made on data
    // older than one query cycle -- tightening one without the other just
    // means deciding fast on stale data, or collecting fresh data the RIC
    // never asks for.
    double      lmInterval       = 0.5;
    double      txPower          = 35.0;         // start higher so ES degradation is visible
    double      esTargetW        = 20.0;         // ES: reduce toward 20 W (~13 dBm)
    double      esStepDb         = 1.0;
    bool        useOnnxEs        = false;        // opt-in: DQN ONNX ES instead of rule-based
    bool        useOnnxMro       = false;        // opt-in: DQN ONNX MRO/handover instead of rule-based
    bool        useOnnxMlb       = false;        // opt-in: DQN ONNX MLB instead of rule-based
    bool        useOnnxCco       = false;        // opt-in: DQN ONNX CCO instead of rule-based
    std::string groundTruthPath  = "";           // if set, dump real MRO training data (workspace/ml/mro_dqn/)
    double      ccoRsrpDbm       = -95.0;        // CCO reactive / violation threshold
    double      ccoCritDbm       = -105.0;       // CCO: issue RET when critically low
    double      ccoFracThr       = 0.15;
    double      ccoCritFrac      = 0.30;
    double      ccoRetStep       = 1.0;
    double      mlbThresh        = 0.20;
    double      ueSpeed          = 3.0;          // m/s — natural RSRP variation + handovers
    // method is now locked per-mode: baseline=noop, reactive=qacm, proactive=proactive-gate
    std::string dbFile           = "oran-dublin-four-xapp.db";
    // Three experimental modes for paper evaluation
    std::string mode             = "reactive";   // baseline | reactive | proactive
    double      proactiveThresh  = -90.0;        // KPI predictor acts 5 dBm before violation
    double      emaAlpha         = 0.3;          // EMA smoothing (lower = smoother, better for Dublin)
    uint32_t    predHorizon      = 3;            // 3-cycle look-ahead = 15 s at 5 s LM interval

    CommandLine cmd(__FILE__);
    cmd.AddValue("enb-pos-file",       "Dublin eNB position file",          enbPosFile);
    cmd.AddValue("max-enbs",           "Number of eNBs to load",            maxEnbs);
    cmd.AddValue("n-ues-per-enb",      "UEs per eNB",                       nUesPerEnb);
    cmd.AddValue("sim-time",           "Simulation duration (s)",            simTime);
    cmd.AddValue("lm-interval",        "LM query interval (s)",             lmInterval);
    cmd.AddValue("tx-power",           "Initial TxPower (dBm)",             txPower);
    cmd.AddValue("es-target-w",        "ES target power (W)",               esTargetW);
    cmd.AddValue("es-step-db",         "ES TxPower step (dB)",              esStepDb);
    cmd.AddValue("use-onnx-es",        "Use the DQN ONNX Energy Saving LM instead of the "
                                       "rule-based one (workspace/ml/es_dqn/)", useOnnxEs);
    cmd.AddValue("use-onnx-mro",       "Use the DQN ONNX MRO/handover LM instead of the "
                                       "rule-based one (workspace/ml/mro_dqn/)", useOnnxMro);
    cmd.AddValue("use-onnx-mlb",       "Use the DQN ONNX MLB LM instead of the "
                                       "rule-based one (workspace/ml/mlb_dqn/)", useOnnxMlb);
    cmd.AddValue("use-onnx-cco",       "Use the DQN ONNX CCO LM instead of the "
                                       "rule-based one (workspace/ml/cco_dqn/); TxPower-only, "
                                       "no RET", useOnnxCco);
    cmd.AddValue("ground-truth-file",  "If set, dump real ground-truth kinematics/mode/RSRP "
                                       "for offline MRO retraining (workspace/ml/mro_dqn/)",
                                       groundTruthPath);
    cmd.AddValue("cco-rsrp-dbm",       "CCO/violation RSRP threshold (dBm)",ccoRsrpDbm);
    cmd.AddValue("cco-crit-dbm",       "CCO critical-RSRP threshold",       ccoCritDbm);
    cmd.AddValue("cco-frac-thr",       "CCO low-RSRP fraction trigger",     ccoFracThr);
    cmd.AddValue("cco-crit-frac",      "CCO critical fraction trigger",      ccoCritFrac);
    cmd.AddValue("cco-ret-step",       "CCO RET step (deg)",                ccoRetStep);
    cmd.AddValue("mlb-threshold",      "MLB load imbalance threshold",      mlbThresh);
    cmd.AddValue("ue-speed",           "UE mobility speed (m/s)",           ueSpeed);
    cmd.AddValue("conflict-log",       "CDC CSV output path",               conflictLog);
    cmd.AddValue("los-trace",          "LOS/NLOS RSRP trace CSV output path (empty=disabled)", losTracePath);
    cmd.AddValue("kpi-csv",            "Combined per-cycle, per-eNB parameter+KPI CSV output "
                                       "path (empty=disabled)",             kpiCsvPath);
    cmd.AddValue("ping-pong-window-sec", "A-B-A handover window counted as ping-pong",
                                         pingPongWindowSec);
    cmd.AddValue("enb-capacity-mbps",  "Fixed per-eNB bandwidth capacity MLB balances "
                                       "demand against (uniform across all eNBs)",
                                       enbCapacityMbps);
    cmd.AddValue("mode",               "baseline|reactive|proactive",       mode);
    cmd.AddValue("proactive-thresh",   "KPI predictor RSRP threshold (dBm)",proactiveThresh);
    cmd.AddValue("ema-alpha",          "EMA smoothing factor",              emaAlpha);
    cmd.AddValue("pred-horizon",       "Prediction horizon (cycles)",       predHorizon);
    cmd.Parse(argc, argv);
    g_pingPongWindowSec = pingPongWindowSec;

    // Real handover failures/RLF (non-ideal RRC and/or EnableRlfDetection)
    // were investigated and found architecturally incompatible with this
    // scenario: the RIC issues handovers directly via
    // OranCommandLte2LteHandover (NoOpHandoverAlgorithm means ns-3's native
    // handover algorithm never acts on its own), and ns-3's real RRC/RLF
    // procedures aren't designed to coexist with externally-issued
    // handovers -- every combination tried (non-ideal RRC alone, RLF
    // detection alone, both together) hit a hard ns-3 LTE RRC state-machine
    // abort ("method unexpected in state ...", lte-enb-rrc.cc:794) within
    // the first 1-2 minutes of simulated time. So ho_fail/rlf/conn_timeouts
    // stay structurally at zero here (same as before) -- a known, deliberate
    // limitation, not an oversight. PHY error models are still enabled
    // (stable in isolation) for real packet-level throughput/error effects.
    // Ping-pong handovers ARE real and measured (see g_pingPong below) since
    // that only needs ideal RRC's existing HandoverStart/HandoverEndOk
    // events, not the failure-prone procedures above.
    Config::SetDefault("ns3::LteHelper::UseIdealRrc",              BooleanValue(true));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize",           UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection",        BooleanValue(false));
    Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(true));
    Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(true));
    // Default is 0 = never re-evaluated after the first LOS/NLOS draw for a given
    // eNB-UE pair. At ueSpeed~3 m/s, 1 s steps re-roll the state roughly every 3 m
    // of movement, so mobility actually changes LOS/NLOS over the run instead of
    // freezing it at whatever was drawn on attach.
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(Seconds(1.0)));

    if (!losTracePath.empty())
    {
        g_losTraceFile.open(losTracePath);
        g_losTraceFile << "time_s,ue_idx,cell_id,los_state,rsrp_dbm,dist2d_m\n";
    }

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
                       "rsrp_viol,sinr_viol,cdr_viol,hsr_viol,tl_viol,ee_viol\n";
    }

    LogComponentEnable("OranLmLte2LteCoverageCapacityOptimization",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranCmmConflictTriage",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    // ── Topology ──────────────────────────────────────────────────────────────
    std::vector<Vector> sites = LoadPositions(enbPosFile, maxEnbs);
    uint32_t nEnbs = sites.size();
    uint32_t nUes  = nEnbs * nUesPerEnb;

    // 3GPP TR 38.901 UMi-StreetCanyon: matches this scenario's 20 m eNB height and
    // ~53 m inter-site spacing (dense urban small-cell, not macro). Replaces the
    // deterministic Cost231 model with a stochastic per-link LOS/NLOS state
    // (ThreeGppUmiStreetCanyonChannelConditionModel, set internally as the model's
    // default) plus log-normal shadowing, so UEs behind other UEs/eNBs from the
    // serving cell's perspective sometimes see NLOS path loss instead of always LOS.
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetAttribute("PathlossModel",
                            StringValue("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(50));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHosts;
    remoteHosts.Create(1);
    Ptr<Node> remoteHost = remoteHosts.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHosts);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevs = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevs);
    Ipv4StaticRoutingHelper ipv4Routing;
    Ptr<Ipv4StaticRouting> rhRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    rhRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Nodes
    NodeContainer enbNodes, ueNodes;
    enbNodes.Create(nEnbs);
    ueNodes.Create(nUes);

    // eNB positions from Dublin file
    Ptr<ListPositionAllocator> enbPos = CreateObject<ListPositionAllocator>();
    for (const auto& v : sites)
        // z forced to 10 m: ThreeGppUmiStreetCanyonPropagationLossModel disambiguates
        // which of the two mobility models is the BS by an exact z==10.0 check
        // (GetBsUtHeightsUmiStreetCanyon); the source file's z=20 fails that check,
        // silently swapping hBS/hUT in the loss formula. 10 m also matches the
        // fixed BS height TR 38.901 Table 7.4.2-1 assumes for the LOS probability
        // curve itself, so this isn't a workaround, it's what the scenario type requires.
        enbPos->Add(Vector(v.x, v.y, 10.0));
    MobilityHelper enbMob;
    enbMob.SetPositionAllocator(enbPos);
    enbMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    enbMob.Install(enbNodes);

    // UE positions: disc around each eNB; velocity toward adjacent eNB to provoke handovers
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Stream", IntegerValue(42));
    Ptr<ListPositionAllocator> uePos = CreateObject<ListPositionAllocator>();
    const double twoPi = 6.283185307179586;
    for (uint32_t i = 0; i < nUes; i++)
    {
        const Vector& c = sites[i % nEnbs];
        double r     = 60.0 * std::sqrt(rng->GetValue());
        double theta = twoPi * rng->GetValue();
        uePos->Add(Vector(c.x + r * std::cos(theta), c.y + r * std::sin(theta), 1.5));
    }
    MobilityHelper ueMob;
    ueMob.SetPositionAllocator(uePos);
    ueMob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMob.Install(ueNodes);

    // Assign each UE one of the 7 mobility-type profiles (cycling through
    // g_ueModeProfiles), then start its self-rescheduling kinematic update
    // (see UpdateUeKinematics above) instead of setting velocity once and
    // never touching it again -- so acceleration/jerk/bearing-rate are
    // genuinely non-zero and mode-dependent, matching what MRO's mode
    // classifier (workspace/ml/mro_dqn/) is trained to recognize.
    // --ue-speed scales all mode bands proportionally (default 3.0 m/s was
    // the old single-speed baseline, kept as the reference point).
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
        mob->SetVelocity(Vector(state.speed * std::cos(state.heading),
                                state.speed * std::sin(state.heading),
                                0.0));

        Simulator::Schedule(kinematicUpdateInterval,
                            &UpdateUeKinematics,
                            i,
                            mob,
                            speedNoiseRv,
                            headingNoiseRv,
                            kinematicUpdateInterval);
    }

    // LTE devices
    g_enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    for (uint32_t i = 0; i < g_enbDevs.GetN(); i++)
        DynamicCast<LteEnbNetDevice>(g_enbDevs.Get(i))->GetPhy()->SetTxPower(txPower);
    NetDeviceContainer ueDevs = lteHelper->InstallUeDevice(ueNodes);

    // InstallEnbDevice() triggers LteHelper::DoInitialize() -> ChannelModelInitialization(),
    // which is when the actual ThreeGppUmiStreetCanyonPropagationLossModel instance (and its
    // default ThreeGppUmiStreetCanyonChannelConditionModel) gets created, so it's only safe to
    // fetch it after this point.
    // Unconditional now (used to be gated on --los-trace): also needed by
    // CollectGroundTruth and by OranLmLte2LteOnnxMro's real LOS/NLOS feature.
    {
        Ptr<PropagationLossModel> plm =
            lteHelper->GetDownlinkSpectrumChannel()->GetPropagationLossModel();
        Ptr<ThreeGppPropagationLossModel> tgppPlm = DynamicCast<ThreeGppPropagationLossModel>(plm);
        NS_ABORT_MSG_IF(!tgppPlm,
                        "Expected a ThreeGppPropagationLossModel on the downlink channel");
        g_channelConditionModel = tgppPlm->GetChannelConditionModel();

        for (uint32_t i = 0; i < g_enbDevs.GetN(); i++)
        {
            uint16_t cellId = DynamicCast<LteEnbNetDevice>(g_enbDevs.Get(i))->GetCellId();
            g_cellIdToEnbMobility[cellId] = enbNodes.Get(i)->GetObject<MobilityModel>();
        }
    }

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); u++)
    {
        ipv4Routing.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>())
            ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    // Attach UEs: spread across eNBs
    for (uint32_t u = 0; u < nUes; u++)
        lteHelper->Attach(ueDevs.Get(u), g_enbDevs.Get(u % nEnbs));
    lteHelper->AddX2Interface(enbNodes);

    // Energy models
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1e7));
    srcHelper.Set("BasicEnergySupplyVoltageV",        DoubleValue(48.0));
    energy::EnergySourceContainer eSources = srcHelper.Install(enbNodes);
    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(g_enbDevs.Get(i));
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(eSources.Get(i));
        Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetLteEnbPhy(dev->GetPhy());
        Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
        ru->SetAttribute("NumTrx",         UintegerValue(4));
        ru->SetAttribute("EtaPA",          DoubleValue(0.30));
        ru->SetAttribute("FixedOverheadW", DoubleValue(5.0));
        ru->SetAttribute("DeltaAf",        DoubleValue(0.5));
        ru->SetAttribute("DeltaDC",        DoubleValue(0.07));
        ru->SetAttribute("DeltaMS",        DoubleValue(0.09));
        ru->SetAttribute("DeltaCool",      DoubleValue(0.10));
        ru->SetAttribute("Vdc",            DoubleValue(48.0));
        ru->SetAttribute("SleepPowerW",    DoubleValue(2.0));
        ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));
        src->AppendDeviceEnergyModel(dem);
        g_enbEnergyModels.push_back(dem);
        g_enbInitialEnergyJ.push_back(src->GetInitialEnergy());
    }

    // UDP DL traffic -- per-UE eMBB/URLLC/mMTC/V2X profile (see g_trafficProfiles),
    // instead of the old homogeneous 1 Mbps constant-on traffic, so real demand
    // actually varies per UE/cell for bandwidth-capacity-aware MLB to act on.
    for (uint32_t u = 0; u < nUes; u++)
    {
        const TrafficTypeProfile& tp = g_trafficProfiles[g_ueTrafficType[u]];
        double rateMbps = tp.minMbps + (tp.maxMbps - tp.minMbps) * rng->GetValue();

        uint16_t port = 9000 + u;
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sink.Install(ueNodes.Get(u));
        Ptr<LteUeNetDevice> sinkUeDev = DynamicCast<LteUeNetDevice>(ueDevs.Get(u));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxBytes));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx",
            MakeBoundCallback(&RxByCell, u, sinkUeDev));
        sinkApp.Start(Seconds(1.0));
        sinkApp.Stop(Seconds(simTime));

        Ptr<OnOffApplication> src = CreateObject<OnOffApplication>();
        src->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIps.GetAddress(u), port)));
        src->SetAttribute("DataRate",   DataRateValue(DataRate(static_cast<uint64_t>(rateMbps * 1e6))));
        src->SetAttribute("PacketSize", UintegerValue(tp.packetSize));
        src->SetAttribute("OnTime",  StringValue(tp.onTimeExpr));
        src->SetAttribute("OffTime", StringValue(tp.offTimeExpr));
        remoteHost->AddApplication(src);
        src->SetStartTime(Seconds(2.0));
        src->SetStopTime(Seconds(simTime - 1.0));
    }

    // ── Near-RT RIC with all four xApps ───────────────────────────────────────
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose",                   BooleanValue(false));
    oranHelper->SetAttribute("LmQueryInterval",           TimeValue(Seconds(lmInterval)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",        TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy",  StringValue("DROP"));
    std::remove(dbFile.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile", StringValue(dbFile));

    // Default LM: MRO (handover-based, issues TxPower adjustments for handover trigger)
    if (useOnnxMro)
    {
        oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteOnnxMro");
    }
    else
    {
        oranHelper->SetDefaultLogicModule(
            "ns3::OranLmLte2LteRsrpHandover",
            // 500ms: under the 1s Near-RT threshold (was 1.5s). This is an
            // xApp-internal anti-ping-pong cooldown, not the RIC's own
            // control-loop timing -- tightening it is a deliberate choice,
            // not a compliance requirement, and will likely increase
            // ping-pong events (expected, not a bug).
            // Cannot safely go below ~1.5s at a 500ms RIC loop: testing found
            // 1.0s and 0.5s both crash reliably (segfault/abort in ns-3's
            // LTE RRC state machine, "method unexpected in state ...") --
            // MRO re-triggers a handover for the same UE before its PRIOR
            // handover has fully settled internally. This is an xApp-level
            // stability parameter, not part of the RIC's own control-loop
            // timing (see the --lm-interval/SendIntervalRv comments above),
            // so keeping it above 1s does not weaken the Near-RT claim --
            // real deployments carry comparable per-action cooldowns (HOM/TTT)
            // alongside a sub-second control loop for the same reason.
            "HandoverHoldoffSec",    DoubleValue(1.5),
            "RsrpHysteresisDb",      DoubleValue(2.0),
            "EnableCellControlBias", BooleanValue(true));
    }

    // ES: always on in all modes — aggressively reduces TxPower (conflict source).
    // --use-onnx-es swaps in the DQN ONNX policy (workspace/ml/es_dqn/) instead;
    // note this changes the ES behavior this example's conflict-mitigation study
    // is built around, so it defaults to off.
    if (useOnnxEs)
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteOnnxEnergySaving",
            "OnnxModelPath",     StringValue("es_dqn.onnx"),
            "NominalTxPowerDbm", DoubleValue(txPower));
    }
    else
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteEnergySaving",
            "TargetPowerW",  DoubleValue(esTargetW),
            "StepSize",      DoubleValue(esStepDb),
            "LmIntervalSec", DoubleValue(lmInterval));
    }

    // MLB: always on — CIO-based load balancing.
    // --use-onnx-mlb swaps in the DQN ONNX policy (workspace/ml/mlb_dqn/) instead;
    // it still issues CIO (not handovers) so CDC keeps recognizing it as MLB.
    if (useOnnxMlb)
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteOnnxMlb",
            "DqnPath",   StringValue("mlb_dqn.onnx"),
            "CioStep",   DoubleValue(1.0),
            "MaxAbsCio", DoubleValue(6.0));
    }
    else
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteMobilityLoadBalancing",
            "LoadImbalanceThreshold", DoubleValue(mlbThresh),
            "CioStep",                DoubleValue(1.0),
            "MaxAbsCio",              DoubleValue(6.0),
            "EnbCapacityMbps",        DoubleValue(enbCapacityMbps));
    }

    // ── Mode-specific CCO, predictor, and CMM ─────────────────────────────────
    //
    // ALL three modes run the same four xApps: MRO (default) + ES + MLB + CCO.
    // The ONLY differences are:
    //   baseline      — Noop CMM (detects conflicts via CDC, resolves nothing)
    //   reactive      — QACM CMM (QoS-aware mitigation after observed violation)
    //   proactive     — proactive-gate CMM (ES suppressed before CDC for at-risk
    //                   cells; QACM fallback for remaining conflicts) + EMA predictor
    //
    // CCO parameters are identical across all modes. RET adjustment is enabled
    // only in reactive (creates Indirect conflicts with MRO). Proactive uses
    // TxPower-only CCO (no RET → no Indirect conflicts).

    if (mode == "baseline")
    {
        // CCO: same params as reactive — raises TxPower AND adjusts RET.
        // With Noop CMM, ES wins every conflict → TXP drops → RSRP collapses.
        // CDC still DETECTS and LOGS conflicts (same count as reactive).
        // --use-onnx-cco swaps in the DQN ONNX policy (workspace/ml/cco_dqn/)
        // instead; it is TxPower-only (no RET), which removes the Indirect
        // RET→MRO conflict path in this mode too.
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteOnnxCco",
                "DqnPath",           StringValue("cco_dqn.onnx"),
                "NominalTxPowerDbm", DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteCoverageCapacityOptimization",
                "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
                "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
                "StepSize",                  DoubleValue(1.0),
                "MinSamplesPerCell",         UintegerValue(1),
                "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
                "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
                "RetStepDeg",                DoubleValue(ccoRetStep));
        }
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method", StringValue("noop"));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method",          StringValue("noop"),
                                                     "ConflictLogFile", StringValue(conflictLog));
    }
    else if (mode == "reactive")
    {
        // CCO: raises TxPower AND adjusts RET when RSRP is low.
        // RET change → Indirect conflict with MRO (CDR/HSR affected, MRO ∉ P2X[RET]).
        // QACM CMM: QoS-aware fraction p = clamp(margin/20, 0, 1).
        //   At threshold → p=0 → CCO fully wins; 10 dBm above → p=0.5.
        // --use-onnx-cco swaps in the DQN ONNX policy instead (TxPower-only,
        // no RET → no Indirect RET→MRO conflict in this mode either).
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteOnnxCco",
                "DqnPath",           StringValue("cco_dqn.onnx"),
                "NominalTxPowerDbm", DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteCoverageCapacityOptimization",
                "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
                "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
                "StepSize",                  DoubleValue(1.0),
                "MinSamplesPerCell",         UintegerValue(1),
                "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
                "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
                "RetStepDeg",                DoubleValue(ccoRetStep));
        }
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method", StringValue("qacm"));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method",          StringValue("qacm"),
                                                     "ConflictLogFile", StringValue(conflictLog));
    }
    else // proactive
    {
        // CCO: TxPower only (RetStepDeg=0) to avoid Indirect RET→MRO conflicts.
        // --use-onnx-cco swaps in the DQN ONNX policy instead (also TxPower-only).
        if (useOnnxCco)
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteOnnxCco",
                "DqnPath",           StringValue("cco_dqn.onnx"),
                "NominalTxPowerDbm", DoubleValue(txPower));
        }
        else
        {
            oranHelper->AddLogicModule(
                "ns3::OranLmLte2LteCoverageCapacityOptimization",
                "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
                "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
                "StepSize",                  DoubleValue(1.0),
                "MinSamplesPerCell",         UintegerValue(1),
                "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
                "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
                "RetStepDeg",                DoubleValue(0.0));
        }

        // EMA predictor: when predicted RSRP < proactiveThresh the proactive-gate
        // CMM suppresses ES's TxPower reduction for that cell BEFORE CDC detection.
        // Result: near-zero conflict events for predicted-risk cells.
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteKpiPrediction",
            "ProactiveThresholdDbm", DoubleValue(proactiveThresh),
            "ReactiveThresholdDbm",  DoubleValue(-115.0),
            "EmaAlpha",              DoubleValue(emaAlpha),
            "StepSizeDb",            DoubleValue(1.0),
            "PredictionHorizon",     UintegerValue(predHorizon),
            "MinRsrpSamples",        UintegerValue(1));
        if (conflictLog.empty())
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method", StringValue("proactive-gate"));
        else
            oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                     "Method",          StringValue("proactive-gate"),
                                                     "ConflictLogFile", StringValue(conflictLog));
    }

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    // Real per-cycle parameter-update counts (puf_TXP/CIO/RET/TTT in [STATE]
    // and the combined KPI CSV) come straight from the CMM, which already
    // counts every command it sees when printing [CMD] lines.
    Ptr<OranCmmConflictTriage> triage =
        DynamicCast<OranCmmConflictTriage>(nearRtRic->GetCmm());

    // If --use-onnx-mro is active, wire the real ChannelConditionModel in so its
    // RSRP regressor sees actual LOS/NLOS state instead of a hidden variable it
    // has no way to predict (see OranLmLte2LteOnnxMro::SetChannelConditionModel).
    // Per-eNB/per-UE mobility gets wired in below, inside their E2 terminator
    // loops, once each one's E2 node ID actually exists.
    Ptr<OranLmLte2LteOnnxMro> mroLm =
        DynamicCast<OranLmLte2LteOnnxMro>(nearRtRic->GetDefaultLogicModule());
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
        // hold TxPower fixed during a PTX-sweep calibration run for CCO -- see
        // workspace/ml/cco_dqn/collect_ptx_sweep.sh).
        Simulator::Schedule(Seconds(2.0),
                            &CollectGroundTruth,
                            nearRtRic->Data(),
                            ueNodes,
                            Seconds(2.0),
                            &groundTruthFile);
    }

    // ── UE E2 terminators ─────────────────────────────────────────────────────
    for (uint32_t u = 0; u < nUes; u++)
    {
        Ptr<OranReporterLocation>       locRep  = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo>  cellRep = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRep = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranReporterLteUeAppDemand> demandRep = CreateObject<OranReporterLteUeAppDemand>();
        Ptr<OranE2NodeTerminatorLteUe>  ueTerm  = CreateObject<OranE2NodeTerminatorLteUe>();

        locRep->SetAttribute("Terminator",  PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));
        demandRep->SetAttribute("Terminator", PointerValue(ueTerm));

        Ptr<LteUeNetDevice> lteDev = DynamicCast<LteUeNetDevice>(ueDevs.Get(u));
        Ptr<LteUePhy>       phy    = lteDev->GetPhy();
        phy->TraceConnectWithoutContext("ReportUeMeasurements",
            MakeCallback(&OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
        phy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr",
            MakeCallback(&ObserveRsrpSinr));
        if (!losTracePath.empty())
        {
            Ptr<MobilityModel> ueMob = ueNodes.Get(u)->GetObject<MobilityModel>();
            phy->TraceConnectWithoutContext("ReportCurrentCellRsrpSinr",
                MakeBoundCallback(&ObserveLosNlosRsrp, ueMob, u));
        }

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        // Matches lmInterval: the RIC's decisions are only as fresh as the
        // last E2 report, so this must track the query cadence, not sit at
        // a fixed 1s regardless of it.
        ueTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
        ueTerm->AddReporter(locRep);
        ueTerm->AddReporter(cellRep);
        ueTerm->AddReporter(rsrpRep);
        ueTerm->AddReporter(demandRep);
        ueTerm->Attach(ueNodes.Get(u));
        Simulator::Schedule(Seconds(2.0), &OranE2NodeTerminatorLteUe::Activate, ueTerm);
        Simulator::Schedule(Seconds(2.0), &ReportUeDemand, u, demandRep, Seconds(2.0));
        g_e2UeTerms.Add(ueTerm);
        if (mroLm)
        {
            mroLm->SetUeMobility(ueTerm->GetE2NodeId(), ueNodes.Get(u)->GetObject<MobilityModel>());
        }
    }

    // ── eNB E2 terminators ────────────────────────────────────────────────────
    for (uint32_t i = 0; i < nEnbs; i++)
    {
        Ptr<OranReporterLocation>            locRep    = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteEnergyEfficiency> energyRep =
            CreateObject<OranReporterLteEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorLteEnb> enbTerm = CreateObject<OranE2NodeTerminatorLteEnb>();

        locRep->SetAttribute("Terminator",    PointerValue(enbTerm));
        energyRep->SetAttribute("Terminator", PointerValue(enbTerm));

        enbTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        enbTerm->SetAttribute("RegistrationIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        // Matches lmInterval -- see the UE terminator's SendIntervalRv comment above.
        enbTerm->SetAttribute("SendIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=0.5]"));
        enbTerm->AddReporter(locRep);
        enbTerm->AddReporter(energyRep);
        enbTerm->Attach(enbNodes.Get(i));
        g_e2EnbTerms.Add(enbTerm);
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorLteEnb::Activate, enbTerm);
        if (mroLm)
        {
            mroLm->SetEnbMobility(enbTerm->GetE2NodeId(), enbNodes.Get(i)->GetObject<MobilityModel>());
        }
    }

    // ── Handover event hooks ──────────────────────────────────────────────────
    // HandoverStart/HandoverEndOk fire under ideal RRC too, so ho_ok/ho_attempts
    // and ping-pong are real. HandoverFailureMaxRach/NoPreamble require a real
    // RACH procedure and so structurally can't fire under ideal RRC (they stay
    // at zero) -- but HandoverFailureJoining/Leaving and HandoverEndError are
    // RRC-message-timeout/logical-state failures that CAN and do fire even
    // under ideal RRC, so ho_fail is genuinely non-zero in this config.
    //
    // The four LteEnbRrc HandoverFailure* traces fire as (imsi, rnti, cellId)
    // -- see HandoverFailEnb's comment above -- so they're bound to that
    // dedicated callback, not the (imsi, cellId, rnti)-ordered HandoverFail
    // used for the LteUeRrc-side HandoverEndError.
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverOk));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&HandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
                    MakeCallback(&HandoverFailEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndError",
                    MakeCallback(&HandoverFail));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
                    MakeCallback(&ConnectionTimeout));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RadioLinkFailure",
                    MakeCallback(&RadioLinkFailure));

    // ── Scheduling ───────────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1.0),
                        &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    // LogCellState fires 2 s after first LM correction (same offset as CheckViolation)
    Simulator::Schedule(Seconds(lmInterval + 2.0),
                        &LogCellState, Seconds(lmInterval), lmInterval, triage, enbCapacityMbps);

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ── Final summary ─────────────────────────────────────────────────────────
    double throughputMbps = (g_rxBytesTotal * 8.0) / (simTime * 1e6);
    double totalHoAtt = g_hoOkTotal + g_hoFailTotal;
    double hsr = (totalHoAtt > 0) ? g_hoOkTotal / totalHoAtt : 1.0;
    double cdr = (totalHoAtt > 0) ? g_hoFailTotal / totalHoAtt : 0.0;

    std::cout << "\nRESULT: mode=" << mode
              << " enbs=" << nEnbs << " ues=" << nUes
              << " throughput_mbps=" << throughputMbps
              << " rsrp_viol_cycles=" << g_rsrpViolCycles
              << " lm_cycles=" << g_cycleCount
              << " ho_ok=" << g_hoOkTotal
              << " ho_fail=" << g_hoFailTotal
              << " rlf=" << g_rlfTotal
              << " ping_pong=" << g_pingPongTotal
              << " conn_timeouts=" << g_connTimeoutsTotal
              << " HSR=" << hsr
              << " CDR=" << cdr
              << std::endl;

    if (g_losTraceFile.is_open())
    {
        g_losTraceFile.close();
    }
    if (g_kpiCsvFile.is_open())
    {
        g_kpiCsvFile.close();
    }

    Simulator::Destroy();
    return 0;
}
