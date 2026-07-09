/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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
#include <fstream>
#include <iomanip>
#include <map>
#include <regex>
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

// Per-eNB cumulative byte counters (by eNB index, not reset between cycles)
static std::map<uint32_t, uint64_t> g_rxBytesByEnbIdx; // enbIdx → cumulative bytes

// Cumulative
static uint32_t g_hoOkTotal       = 0;
static uint32_t g_hoFailTotal     = 0;
static uint32_t g_rlfTotal        = 0;
static uint64_t g_rxBytesTotal    = 0;
static uint64_t g_rxBytesLastCycle = 0;   // snapshot at end of last cycle for delta
static uint32_t g_rsrpViolCycles  = 0;    // number of LM cycles with worst RSRP < threshold

// eNB devices and terminators (global for LogCellState access)
static NetDeviceContainer            g_enbDevs;
static OranE2NodeTerminatorContainer g_e2EnbTerms;
static std::vector<Ptr<OranRuDeviceEnergyModel>> g_enbEnergyModels;
static std::vector<double>           g_enbInitialEnergyJ;

static uint32_t g_cycleCount = 0;

// LOS/NLOS RSRP trace: cellId -> serving eNB's mobility model, plus the shared
// channel condition model instance actually driving the downlink path loss
// (so IsLos() below reflects the real state used for that RSRP sample, not a
// separate stochastic draw).
static std::map<uint16_t, Ptr<MobilityModel>> g_cellIdToEnbMobility;
static Ptr<ChannelConditionModel> g_channelConditionModel;
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
RxByEnb(uint32_t enbIdx, Ptr<const Packet> pkt, const Address& /*addr*/)
{
    g_rxBytesByEnbIdx[enbIdx] += pkt->GetSize();
}

void
HandoverOk(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t targetCellId, uint16_t /*rnti*/)
{
    g_hoOkTotal++;
    g_hoOk[targetCellId]++;
}

void
HandoverStart(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t srcCell,
              uint16_t /*rnti*/, uint16_t /*targetCell*/)
{
    g_hoAttempts[srcCell]++;
}

void
HandoverFail(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t cellId, uint16_t /*rnti*/)
{
    g_hoFailTotal++;
    g_hoFail[cellId]++;
}

void
RadioLinkFailure(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t /*cellId*/, uint16_t /*rnti*/)
{
    g_rlfTotal++;
}

// ─── Per-cycle cell state logger — emits [STATE] + [PMON] ───────────────────
void
LogCellState(Time interval, double lmIntervalSec)
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

        // EE: cumulative bits received by this eNB's UEs / cumulative energy consumed
        uint64_t cellBytes = g_rxBytesByEnbIdx.count(i) ? g_rxBytesByEnbIdx[i] : 0;
        double energyConsumedJ = std::max(g_enbEnergyModels[i]->GetTotalEnergyConsumption(), 1e-9);
        double ee = (cellBytes * 8.0) / energyConsumedJ; // bits/J

        // puf_* (param update frequency) is computed by the parser from [CMD] lines
        constexpr double txpFreq = 0.0, cioFreq = 0.0, retFreq = 0.0, tttFreq = 0.0;

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
    }

    // Reset per-cycle handover counters (bytes/energy stay cumulative)
    g_hoAttempts.clear();
    g_hoOk.clear();
    g_hoFail.clear();

    Simulator::Schedule(interval, &LogCellState, interval, lmIntervalSec);
}

// ─── Main ────────────────────────────────────────────────────────────────────
int
main(int argc, char* argv[])
{
    std::string enbPosFile       = "/workspace/data/ns3_positions_Three_IE.txt";
    std::string conflictLog      = "";           // CDC CSV path (empty = stdout only)
    std::string losTracePath     = "";           // LOS/NLOS RSRP trace CSV (empty = disabled)
    uint32_t    maxEnbs          = 3;            // use first N eNBs from Dublin file
    uint32_t    nUesPerEnb       = 6;
    double      simTime          = 150.0;
    double      lmInterval       = 5.0;
    double      txPower          = 35.0;         // start higher so ES degradation is visible
    double      esTargetW        = 20.0;         // ES: reduce toward 20 W (~13 dBm)
    double      esStepDb         = 1.0;
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
    cmd.AddValue("cco-rsrp-dbm",       "CCO/violation RSRP threshold (dBm)",ccoRsrpDbm);
    cmd.AddValue("cco-crit-dbm",       "CCO critical-RSRP threshold",       ccoCritDbm);
    cmd.AddValue("cco-frac-thr",       "CCO low-RSRP fraction trigger",     ccoFracThr);
    cmd.AddValue("cco-crit-frac",      "CCO critical fraction trigger",      ccoCritFrac);
    cmd.AddValue("cco-ret-step",       "CCO RET step (deg)",                ccoRetStep);
    cmd.AddValue("mlb-threshold",      "MLB load imbalance threshold",      mlbThresh);
    cmd.AddValue("ue-speed",           "UE mobility speed (m/s)",           ueSpeed);
    cmd.AddValue("conflict-log",       "CDC CSV output path",               conflictLog);
    cmd.AddValue("los-trace",          "LOS/NLOS RSRP trace CSV output path (empty=disabled)", losTracePath);
    cmd.AddValue("mode",               "baseline|reactive|proactive",       mode);
    cmd.AddValue("proactive-thresh",   "KPI predictor RSRP threshold (dBm)",proactiveThresh);
    cmd.AddValue("ema-alpha",          "EMA smoothing factor",              emaAlpha);
    cmd.AddValue("pred-horizon",       "Prediction horizon (cycles)",       predHorizon);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::LteHelper::UseIdealRrc",              BooleanValue(true));
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize",           UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection",        BooleanValue(false));
    Config::SetDefault("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue(false));
    Config::SetDefault("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue(false));
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
    // Give each UE a random direction; some will cross into adjacent cells
    for (uint32_t i = 0; i < nUes; i++)
    {
        double theta = twoPi * rng->GetValue();
        double speed = (i % 3 == 0) ? ueSpeed * 2.0 : ueSpeed;
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
            ->SetVelocity(Vector(speed * std::cos(theta), speed * std::sin(theta), 0.0));
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
    if (!losTracePath.empty())
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

    // UDP DL traffic
    for (uint32_t u = 0; u < nUes; u++)
    {
        uint16_t port = 9000 + u;
        PacketSinkHelper sink("ns3::UdpSocketFactory",
                              InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApp = sink.Install(ueNodes.Get(u));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxBytes));
        sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeBoundCallback(&RxByEnb, u % nEnbs));
        sinkApp.Start(Seconds(1.0));
        sinkApp.Stop(Seconds(simTime));

        Ptr<OnOffApplication> src = CreateObject<OnOffApplication>();
        src->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIps.GetAddress(u), port)));
        src->SetAttribute("DataRate",   DataRateValue(DataRate("1Mbps")));
        src->SetAttribute("PacketSize", UintegerValue(1000));
        src->SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        src->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
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
    oranHelper->SetDefaultLogicModule(
        "ns3::OranLmLte2LteRsrpHandover",
        "HandoverHoldoffSec",    DoubleValue(1.5),
        "RsrpHysteresisDb",      DoubleValue(2.0),
        "EnableCellControlBias", BooleanValue(true));

    // ES: always on in all modes — aggressively reduces TxPower (conflict source)
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteEnergySaving",
        "TargetPowerW",  DoubleValue(esTargetW),
        "StepSize",      DoubleValue(esStepDb),
        "LmIntervalSec", DoubleValue(lmInterval));

    // MLB: always on — CIO-based load balancing
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteMobilityLoadBalancing",
        "LoadImbalanceThreshold", DoubleValue(mlbThresh),
        "CioStep",                DoubleValue(1.0),
        "MaxAbsCio",              DoubleValue(6.0));

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
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteCoverageCapacityOptimization",
            "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
            "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
            "StepSize",                  DoubleValue(1.0),
            "MinSamplesPerCell",         UintegerValue(1),
            "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
            "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
            "RetStepDeg",                DoubleValue(ccoRetStep));
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
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteCoverageCapacityOptimization",
            "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
            "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
            "StepSize",                  DoubleValue(1.0),
            "MinSamplesPerCell",         UintegerValue(1),
            "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
            "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
            "RetStepDeg",                DoubleValue(ccoRetStep));
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
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteCoverageCapacityOptimization",
            "LowRsrpThresholdDbm",       DoubleValue(ccoRsrpDbm),
            "LowRsrpFractionThreshold",  DoubleValue(ccoFracThr),
            "StepSize",                  DoubleValue(1.0),
            "MinSamplesPerCell",         UintegerValue(1),
            "CriticalRsrpThresholdDbm",  DoubleValue(ccoCritDbm),
            "CriticalFractionThreshold", DoubleValue(ccoCritFrac),
            "RetStepDeg",                DoubleValue(0.0));

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

    // ── UE E2 terminators ─────────────────────────────────────────────────────
    for (uint32_t u = 0; u < nUes; u++)
    {
        Ptr<OranReporterLocation>       locRep  = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo>  cellRep = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRep = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe>  ueTerm  = CreateObject<OranE2NodeTerminatorLteUe>();

        locRep->SetAttribute("Terminator",  PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));

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
        ueTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->AddReporter(locRep);
        ueTerm->AddReporter(cellRep);
        ueTerm->AddReporter(rsrpRep);
        ueTerm->Attach(ueNodes.Get(u));
        Simulator::Schedule(Seconds(2.0), &OranE2NodeTerminatorLteUe::Activate, ueTerm);
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
        enbTerm->SetAttribute("SendIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        enbTerm->AddReporter(locRep);
        enbTerm->AddReporter(energyRep);
        enbTerm->Attach(enbNodes.Get(i));
        g_e2EnbTerms.Add(enbTerm);
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorLteEnb::Activate, enbTerm);
    }

    // ── Handover event hooks ──────────────────────────────────────────────────
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverOk));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&HandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
                    MakeCallback(&HandoverFail));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
                    MakeCallback(&HandoverFail));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RadioLinkFailure",
                    MakeCallback(&RadioLinkFailure));

    // ── Scheduling ───────────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1.0),
                        &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    // LogCellState fires 2 s after first LM correction (same offset as CheckViolation)
    Simulator::Schedule(Seconds(lmInterval + 2.0),
                        &LogCellState, Seconds(lmInterval), lmInterval);

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
              << " HSR=" << hsr
              << " CDR=" << cdr
              << std::endl;

    if (g_losTraceFile.is_open())
    {
        g_losTraceFile.close();
    }

    Simulator::Destroy();
    return 0;
}
