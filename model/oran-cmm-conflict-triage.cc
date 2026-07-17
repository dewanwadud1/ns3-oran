/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-cmm-conflict-triage.h"

#include "oran-command-lte-2-lte-cell-parameter.h"
#include "oran-command-lte-2-lte-handover.h"
#include "oran-command-lte-2-lte-tx-power.h"
#include "oran-command.h"
#include "oran-data-repository.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCmmConflictTriage");
NS_OBJECT_ENSURE_REGISTERED(OranCmmConflictTriage);

// ── CDC relationship tables (Algorithm 1 from Computer Networks manuscript) ──
// P2X: which xApp roles own/control each ICP
static const std::map<std::string, std::set<std::string>> kP2X = {
    {"TxPower", {"ES", "MRO", "CCO", "MLB"}},
    {"CIO",     {"MRO", "MLB"}},
    {"TTT",     {"MRO", "MLB"}},
    {"RET",     {"CCO", "MLB"}},
};

// P2K: which KPIs an ICP directly influences
static const std::map<std::string, std::vector<std::string>> kP2K = {
    {"TxPower", {"RSRP", "SINR", "Throughput", "EE", "CDR", "HSR"}},
    {"CIO",     {"CDR", "HSR", "TL", "RSRP-bias"}},
    {"TTT",     {"CDR", "HSR", "TL"}},
    {"RET",     {"RSRP", "SINR", "CDR", "HSR"}},
};

// K2X: which xApp role manages each KPI
static const std::map<std::string, std::string> kK2X = {
    {"EE",        "ES"},
    {"SINR",      "CCO"}, {"Throughput", "CCO"}, {"RSRP", "CCO"},
    {"CDR",       "MRO"}, {"HSR",        "MRO"}, {"CBR",  "MRO"}, {"RSRP-bias", "MRO"},
    {"TL",        "MLB"}, {"RUR",        "MLB"},
};

TypeId
OranCmmConflictTriage::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCmmConflictTriage")
            .SetParent<OranCmm>()
            .AddConstructor<OranCmmConflictTriage>()
            .AddAttribute("Method",
                          "Mitigation mechanism: cancel | dampen | priority | qacm",
                          StringValue("priority"),
                          MakeStringAccessor(&OranCmmConflictTriage::m_method),
                          MakeStringChecker())
            .AddAttribute("TolerateThreshold",
                          "Triage score below which conflict is tolerated (all commands pass).",
                          DoubleValue(0.15),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_tolerateThresh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("DeferThreshold",
                          "Triage score below which conflict is deferred (if D_max not reached).",
                          DoubleValue(0.40),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_deferThresh),
                          MakeDoubleChecker<double>(0.0, 2.0))
            .AddAttribute("DeferMax",
                          "Maximum consecutive deferral cycles (D_max) before forced mitigation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&OranCmmConflictTriage::m_deferMax),
                          MakeUintegerChecker<uint32_t>(0))
            .AddAttribute("DampFraction",
                          "Fraction of the delta removed from lower-priority command (dampen).",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_dampFraction),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("QacmZeta",
                          "Scale for QACM weighted-distance objective.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_qacmZeta),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightSeverity",
                          "Triage score weight for conflict severity.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_wSeverity),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightPersistence",
                          "Triage score weight for conflict persistence.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_wPersistence),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightScope",
                          "Triage score weight for conflict scope (fraction of UEs affected).",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&OranCmmConflictTriage::m_wScope),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ConflictLogFile",
                          "Path to CDC CSV output file (empty = disabled). "
                          "Columns: time_s,enb_e2id,icp,type,conflicting,affected,winner.",
                          StringValue(""),
                          MakeStringAccessor(&OranCmmConflictTriage::m_conflictLogFile),
                          MakeStringChecker());
    return tid;
}

OranCmmConflictTriage::OranCmmConflictTriage()
    : OranCmm()
{
    NS_LOG_FUNCTION(this);
    m_name = "CmmConflictTriage";
}

OranCmmConflictTriage::~OranCmmConflictTriage()
{
    NS_LOG_FUNCTION(this);
}

std::map<std::string, uint32_t>
OranCmmConflictTriage::GetAndResetIcpCounts(uint64_t e2NodeId)
{
    auto it = m_icpCounts.find(e2NodeId);
    if (it == m_icpCounts.end())
    {
        return {};
    }
    std::map<std::string, uint32_t> counts = it->second;
    m_icpCounts.erase(it);
    return counts;
}

std::map<std::string, std::set<std::string>>
OranCmmConflictTriage::GetAndResetIcpActors(uint64_t e2NodeId)
{
    auto it = m_icpActors.find(e2NodeId);
    if (it == m_icpActors.end())
    {
        return {};
    }
    std::map<std::string, std::set<std::string>> actors = it->second;
    m_icpActors.erase(it);
    return actors;
}

std::vector<OranCmmConflictTriage::ConflictEventRecord>
OranCmmConflictTriage::GetAndResetConflictEvents(uint64_t e2NodeId)
{
    auto it = m_conflictEvents.find(e2NodeId);
    if (it == m_conflictEvents.end())
    {
        return {};
    }
    std::vector<ConflictEventRecord> events = it->second;
    m_conflictEvents.erase(it);
    return events;
}

// ── CDC helpers ───────────────────────────────────────────────────────────────
std::string
OranCmmConflictTriage::LmToXappRole(const std::string& lmName) const
{
    // Each check also matches the ONNX-inference counterpart's class name
    // (OranLmLte2LteOnnx{EnergySaving,Cco,Mro,Mlb}), which swaps in for the
    // rule-based LM at the same xApp role without changing role semantics.
    if (lmName.find("EnergySaving")         != std::string::npos) return "ES";
    if (lmName.find("CoverageCapacity")      != std::string::npos) return "CCO";
    if (lmName.find("OnnxCco")               != std::string::npos) return "CCO";
    if (lmName.find("KpiPrediction")         != std::string::npos) return "CCO";  // proactive ≡ CCO role
    if (lmName.find("RsrpHandover")          != std::string::npos) return "MRO";
    if (lmName.find("OnnxMro")               != std::string::npos) return "MRO";
    if (lmName.find("MobilityLoadBalancing") != std::string::npos) return "MLB";
    if (lmName.find("OnnxMlb")               != std::string::npos) return "MLB";
    return "Unknown";
}

void
OranCmmConflictTriage::EmitConflictEvent(uint64_t          enbE2Id,
                                          const std::string& icp,
                                          const std::string& type,
                                          const std::vector<std::string>& conflicting,
                                          const std::vector<std::string>& affected,
                                          const std::string& winnerRole)
{
    double now = Simulator::Now().GetSeconds();

    // Record for GetAndResetConflictEvents() -- type is always "Direct" or
    // "Indirect" here (this taxonomy has no "Implicit" category).
    m_conflictEvents[enbE2Id].push_back(
        ConflictEventRecord{icp, type, conflicting, affected, winnerRole});

    // ── Build human-readable [CDC] line ──────────────────────────────────────
    std::ostringstream oss;
    oss << "[CDC] t=" << now << "s eNB=" << enbE2Id
        << " ICP=" << icp << " type=" << type << " conflicting=[";
    for (size_t i = 0; i < conflicting.size(); i++)
        oss << (i ? "," : "") << conflicting[i];
    oss << "]";
    if (!affected.empty())
    {
        oss << " affected=[";
        for (size_t i = 0; i < affected.size(); i++)
            oss << (i ? "," : "") << affected[i];
        oss << "]";
    }
    oss << " kpis=[";
    auto pit = kP2K.find(icp);
    if (pit != kP2K.end())
        for (size_t i = 0; i < pit->second.size(); i++)
            oss << (i ? "," : "") << pit->second[i];
    oss << "] winner=" << winnerRole;

    std::cout << oss.str() << "\n";
    NS_LOG_INFO(oss.str());
    LogLogicToStorage(oss.str());

    // ── Optional CSV output ───────────────────────────────────────────────────
    if (!m_conflictLogFile.empty())
    {
        if (!m_logStream.is_open())
        {
            m_logStream.open(m_conflictLogFile, std::ios::out | std::ios::trunc);
            if (m_logStream.is_open())
                m_logStream << "time_s,enb_e2id,icp,type,conflicting,affected,winner\n";
        }
        if (m_logStream.is_open())
        {
            std::ostringstream conflRow, affRow;
            for (size_t i = 0; i < conflicting.size(); i++)
                conflRow << (i ? "|" : "") << conflicting[i];
            for (size_t i = 0; i < affected.size(); i++)
                affRow  << (i ? "|" : "") << affected[i];
            m_logStream << now << "," << enbE2Id << "," << icp << "," << type << ","
                        << conflRow.str() << "," << affRow.str() << "," << winnerRole << "\n";
            m_logStream.flush();
        }
    }
}

// ── Priority rank: lower value = higher priority ──────────────────────────────
int
OranCmmConflictTriage::LmPriority(const std::string& lmName) const
{
    // Priority order (lower = higher priority):
    // MRO > CCO = KpiPrediction > MLB > ES
    if (lmName.find("RsrpHandover") != std::string::npos)
        return 0; // MRO
    if (lmName.find("OnnxMro") != std::string::npos)
        return 0; // MRO (ONNX)
    if (lmName.find("CoverageCapacity") != std::string::npos)
        return 1; // CCO
    if (lmName.find("OnnxCco") != std::string::npos)
        return 1; // CCO (ONNX)
    if (lmName.find("KpiPrediction") != std::string::npos)
        return 1; // Proactive predictor — same priority as CCO
    if (lmName.find("MobilityLoadBalancing") != std::string::npos)
        return 2; // MLB
    if (lmName.find("OnnxMlb") != std::string::npos)
        return 2; // MLB (ONNX)
    if (lmName.find("EnergySaving") != std::string::npos)
        return 3; // ES
    return 4; // unknown → lowest priority
}

double
OranCmmConflictTriage::ComputeSeverity(double negDelta, double posDelta) const
{
    // Normalized sum of opposing deltas relative to 10 dB range
    return std::min(1.0, (std::abs(negDelta) + std::abs(posDelta)) / 10.0);
}

double
OranCmmConflictTriage::ComputeTriageScore(const ConflictRecord& rec,
                                          double severity,
                                          double scopeFraction) const
{
    double persistenceNorm = std::min(1.0, rec.persistenceCount / 10.0);
    return m_wSeverity * severity + m_wPersistence * persistenceNorm +
           m_wScope * std::min(1.0, scopeFraction);
}

double
OranCmmConflictTriage::GetWorstServingRsrpForEnb(uint64_t e2NodeId) const
{
    // Query minimum serving-cell RSRP for any UE served by this eNB.
    // Returns kRsrpQosThresholdDbm - 1 (one below threshold) if no data found.
    if (m_nearRtRic == nullptr)
        return kRsrpQosThresholdDbm - 1.0;

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    bool foundCell = false;
    uint16_t cellId = 0;
    std::tie(foundCell, cellId) = data->GetLteEnbCellInfo(e2NodeId);
    if (!foundCell)
        return kRsrpQosThresholdDbm - 1.0;

    double worstRsrp = 0.0;
    bool   hasRsrp   = false;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool ufound = false; uint16_t ueCell = 0, ueRnti = 0;
        std::tie(ufound, ueCell, ueRnti) = data->GetLteUeCellInfo(ueId);
        if (!ufound || ueCell != cellId)
            continue;
        for (const auto& meas : data->GetLteUeRsrpRsrq(ueId))
        {
            uint16_t r = 0, c = 0; double rsrp = 0.0, rsrq = 0.0;
            bool serving = false; uint8_t ccid = 0;
            std::tie(r, c, rsrp, rsrq, serving, ccid) = meas;
            if (c != cellId) continue;
            if (!hasRsrp || rsrp < worstRsrp) { worstRsrp = rsrp; hasRsrp = true; }
        }
    }
    return hasRsrp ? worstRsrp : (kRsrpQosThresholdDbm - 1.0);
}

double
OranCmmConflictTriage::SelectQacmFraction(double /*severity*/, double rsrpMarginDb) const
{
    // QoS-aware QACM: fraction p controls how much of each xApp's command is applied.
    //   p = 0 → ES command cancelled, CCO gets full action (maximum coverage protection)
    //   p = 1 → ES gets full reduction, CCO cancelled (maximum energy saving)
    //
    // Mapping: linear on RSRP margin above the QoS threshold (-95 dBm):
    //   margin ≤ 0 dBm  → p = 0.0  (at/below threshold: CCO fully wins)
    //   margin = 10 dBm → p = 0.5  (balanced compromise)
    //   margin ≥ 20 dBm → p = 1.0  (well above threshold: ES can act freely)
    return std::max(0.0, std::min(1.0, rsrpMarginDb / 20.0));
}

// ── Main Filter ───────────────────────────────────────────────────────────────
std::vector<Ptr<OranCommand>>
OranCmmConflictTriage::Filter(
    std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run ConflictTriage CMM with NULL Near-RT RIC");

    // ── Proactive-Gate pre-processing ─────────────────────────────────────────
    // When method=="proactive-gate": scan for KpiPrediction TXP+ commands.
    // For every eNB where the predictor issued a TXP increase (flagging predicted
    // RSRP risk), SUPPRESS ES's TXP reduction for that eNB BEFORE CDC detection.
    // This means the CDC never sees a conflict for those eNBs — the harmful ES
    // action is gated out before it can be filed. CCO still runs for non-predicted
    // cells; conflicts there are resolved with QACM.
    if (m_method == "proactive-gate")
    {
        // Find eNBs where the predictor raised TXP (= at-risk cells)
        std::set<uint64_t> predictorTargets;
        for (const auto& entry : inputCommands)
        {
            if (std::get<0>(entry.first).find("KpiPrediction") == std::string::npos)
                continue;
            for (auto cmd : entry.second)
            {
                Ptr<OranCommandLte2LteTxPower> txp = cmd->GetObject<OranCommandLte2LteTxPower>();
                if (txp && txp->GetPowerDeltaDb() > 0.0)
                    predictorTargets.insert(txp->GetTargetE2NodeId());
            }
        }
        // Suppress ES TXP- commands for predictor-targeted eNBs
        uint32_t suppressed = 0;
        for (auto& entry : inputCommands)
        {
            if (LmToXappRole(std::get<0>(entry.first)) != "ES")
                continue;
            auto& cmds = entry.second;
            cmds.erase(
                std::remove_if(cmds.begin(), cmds.end(),
                    [&predictorTargets, &suppressed](Ptr<OranCommand> cmd) {
                        Ptr<OranCommandLte2LteTxPower> txp =
                            cmd->GetObject<OranCommandLte2LteTxPower>();
                        if (txp && txp->GetPowerDeltaDb() < 0.0 &&
                            predictorTargets.count(txp->GetTargetE2NodeId()))
                        {
                            suppressed++;
                            return true; // remove (suppress)
                        }
                        return false;
                    }),
                cmds.end());
        }
        if (suppressed > 0)
        {
            std::ostringstream gateMsg;
            gateMsg << "[GATE] t=" << Simulator::Now().GetSeconds()
                    << "s proactive-gate suppressed " << suppressed
                    << " ES TXP- command(s) for " << predictorTargets.size()
                    << " predicted-risk eNB(s)";
            std::cout << gateMsg.str() << "\n";
            NS_LOG_INFO(gateMsg.str());
        }
    }

    // ── Step 0: collect all active xApp roles (for indirect conflict detection) ─
    std::set<std::string> allActiveRoles;
    for (const auto& entry : inputCommands)
        allActiveRoles.insert(LmToXappRole(std::get<0>(entry.first)));

    // ── Step 1: bucket TXP and cell-parameter commands by target eNB ──────────
    // TXP bucket: Key = e2NodeId, Value = [(lmName, cmd)]
    std::map<uint64_t, std::vector<std::pair<std::string, Ptr<OranCommandLte2LteTxPower>>>>
        txpByNode;

    // Cell-parameter bucket: Key = (e2NodeId, paramName), Value = [(lmName, cmd)]
    std::map<std::pair<uint64_t, std::string>,
             std::vector<std::pair<std::string, Ptr<OranCommandLte2LteCellParameter>>>>
        cellParamByNodeAndParam;

    // All non-TXP commands pass through (cell-param commands are also bucketed above)
    std::vector<Ptr<OranCommand>> passThrough;

    double nowCmdLog = Simulator::Now().GetSeconds();
    for (const auto& entry : inputCommands)
    {
        const std::string& lmName = std::get<0>(entry.first);
        const std::string  role   = LmToXappRole(lmName);
        for (auto cmd : entry.second)
        {
            uint64_t e2id = cmd->GetTargetE2NodeId();
            Ptr<OranCommandLte2LteTxPower> txp = cmd->GetObject<OranCommandLte2LteTxPower>();
            if (txp != nullptr)
            {
                // [CMD] log for TXP commands
                std::cout << "[CMD] t=" << nowCmdLog << "s"
                          << " e2=" << e2id
                          << " xapp=" << role
                          << " icp=TxPower"
                          << " delta=" << std::showpos << txp->GetPowerDeltaDb()
                          << std::noshowpos << "\n";
                txpByNode[e2id].emplace_back(lmName, txp);
                m_icpCounts[e2id]["TxPower"]++;
                m_icpActors[e2id]["TxPower"].insert(role);
            }
            else
            {
                Ptr<OranCommandLte2LteCellParameter> cp =
                    cmd->GetObject<OranCommandLte2LteCellParameter>();
                if (cp != nullptr)
                {
                    // [CMD] log for cell-parameter commands
                    std::cout << "[CMD] t=" << nowCmdLog << "s"
                              << " e2=" << e2id
                              << " xapp=" << role
                              << " icp=" << cp->GetParameterName()
                              << " value=" << cp->GetValue() << "\n";
                    auto key = std::make_pair(e2id, cp->GetParameterName());
                    cellParamByNodeAndParam[key].emplace_back(lmName, cp);
                    m_icpCounts[e2id][cp->GetParameterName()]++;
                    m_icpActors[e2id][cp->GetParameterName()].insert(role);
                }
                // All non-TXP commands (handover, cell-parameter, etc.) pass through
                passThrough.push_back(cmd);
            }
        }
    }

    std::vector<Ptr<OranCommand>> result = passThrough;

    // ── Step 2: process each eNB's TXP commands ─────────────────────────────
    for (auto& nodeEntry : txpByNode)
    {
        uint64_t e2NodeId = nodeEntry.first;
        auto& cmds        = nodeEntry.second;

        if (cmds.size() <= 1)
        {
            // No conflict — pass through
            for (auto& pr : cmds)
                result.push_back(pr.second);
            // Reset persistence since no conflict this cycle
            if (m_txpConflicts.count(e2NodeId))
            {
                m_txpConflicts[e2NodeId].persistenceCount = 0;
                m_txpConflicts[e2NodeId].deferCount       = 0;
            }
            continue;
        }

        // ── Conflict detected: multiple LMs issued TXP for this eNB ─────────
        double negSum = 0.0; // sum of negative deltas (ES wants TXP-)
        double posSum = 0.0; // sum of positive deltas (CCO wants TXP+)
        for (auto& pr : cmds)
        {
            double d = pr.second->GetPowerDeltaDb();
            if (d < 0.0) negSum += d;
            else          posSum += d;
        }

        // Update conflict history
        auto& rec = m_txpConflicts[e2NodeId];
        rec.persistenceCount++;
        rec.lastSeverity = ComputeSeverity(negSum, posSum);

        // Scope proxy: assume 1.0 (all UEs affected) — can be refined with
        // repository data if needed
        double triageScore = ComputeTriageScore(rec, rec.lastSeverity, 1.0);

        // ── CDC: classify the TXP conflict (Algorithm 1) ────────────────────
        // All issuing roles own TXP (ES, CCO, MRO, MLB all ∈ P2X["TxPower"]) →
        // this is always a Direct conflict at the TXP level.
        {
            std::vector<std::string> conflictingRoles;
            int bestPriCdc = std::numeric_limits<int>::max();
            std::string winnerLmCdc;
            for (auto& pr : cmds)
            {
                conflictingRoles.push_back(LmToXappRole(pr.first));
                int pri = LmPriority(pr.first);
                if (pri < bestPriCdc) { bestPriCdc = pri; winnerLmCdc = pr.first; }
            }
            EmitConflictEvent(e2NodeId, "TxPower", "Direct",
                              conflictingRoles, {}, LmToXappRole(winnerLmCdc));
        }

        std::ostringstream msg;
        msg << "TXP conflict on e2=" << e2NodeId
            << " nCmds=" << cmds.size()
            << " negSum=" << negSum << " posSum=" << posSum
            << " severity=" << rec.lastSeverity
            << " persistence=" << rec.persistenceCount
            << " score=" << triageScore;

        // ── Triage decision ────────────────────────────────────────────────
        if (m_method == "noop")
        {
            // NOOP (baseline): CDC detects conflicts; lowest-priority (ES) wins.
            // CCO's protective TXP+ is blocked — energy saving takes precedence.
            // This represents "no conflict management": ES degrades coverage unchecked.
            msg << " decision=NOOP(ES-wins)";
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());
            int worstPri = -1;
            Ptr<OranCommandLte2LteTxPower> esCmd = nullptr;
            for (auto& pr : cmds)
            {
                int pri = LmPriority(pr.first);
                if (pri > worstPri) { worstPri = pri; esCmd = pr.second; }
            }
            if (esCmd)
                result.push_back(esCmd); // only ES's TXP- passes
        }
        else if (triageScore < m_tolerateThresh)
        {
            // TOLERATE: all commands pass
            msg << " decision=TOLERATE";
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());
            rec.benignCount++;
            rec.deferCount = 0;
            for (auto& pr : cmds)
                result.push_back(pr.second);
        }
        else if (triageScore < m_deferThresh && rec.deferCount < m_deferMax)
        {
            // DEFER: pass only the highest-priority command; delay others
            msg << " decision=DEFER deferCount=" << rec.deferCount + 1;
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());
            rec.deferCount++;
            // Find and pass the highest-priority command only
            int bestPri = std::numeric_limits<int>::max();
            Ptr<OranCommandLte2LteTxPower> bestCmd = nullptr;
            for (auto& pr : cmds)
            {
                int pri = LmPriority(pr.first);
                if (pri < bestPri)
                {
                    bestPri = pri;
                    bestCmd = pr.second;
                }
            }
            if (bestCmd)
                result.push_back(bestCmd);
        }
        else
        {
            // MITIGATE: apply selected mechanism
            rec.harmfulCount++;
            rec.deferCount = 0;
            msg << " decision=MITIGATE method=" << m_method;
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());

            if (m_method == "cancel")
            {
                // Cancel all lower-priority commands; keep highest-priority
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandLte2LteTxPower> bestCmd = nullptr;
                std::string bestLm;
                for (auto& pr : cmds)
                {
                    int pri = LmPriority(pr.first);
                    if (pri < bestPri)
                    {
                        bestPri = pri;
                        bestCmd = pr.second;
                        bestLm  = pr.first;
                    }
                }
                if (bestCmd)
                {
                    NS_LOG_INFO("Triage cancel: keeping " << bestLm << " command, cancelling "
                                << (cmds.size() - 1) << " others.");
                    result.push_back(bestCmd);
                }
            }
            else if (m_method == "dampen")
            {
                // Scale lower-priority TXP deltas; keep highest-priority unchanged
                int bestPri = std::numeric_limits<int>::max();
                for (auto& pr : cmds)
                    bestPri = std::min(bestPri, LmPriority(pr.first));

                for (auto& pr : cmds)
                {
                    int pri = LmPriority(pr.first);
                    if (pri == bestPri)
                    {
                        result.push_back(pr.second); // highest priority unchanged
                    }
                    else
                    {
                        double scaledDelta = pr.second->GetPowerDeltaDb() * (1.0 - m_dampFraction);
                        if (std::abs(scaledDelta) > 1e-9)
                        {
                            Ptr<OranCommandLte2LteTxPower> dampened =
                                CreateObject<OranCommandLte2LteTxPower>();
                            dampened->SetAttribute("TargetE2NodeId",
                                                   UintegerValue(e2NodeId));
                            dampened->SetAttribute("PowerDeltaDb", DoubleValue(scaledDelta));
                            result.push_back(dampened);
                        }
                    }
                }
            }
            else if (m_method == "priority")
            {
                // MRO > CCO > MLB > ES — only highest-priority LM passes
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandLte2LteTxPower> bestCmd = nullptr;
                for (auto& pr : cmds)
                {
                    int pri = LmPriority(pr.first);
                    if (pri < bestPri)
                    {
                        bestPri = pri;
                        bestCmd = pr.second;
                    }
                }
                if (bestCmd)
                    result.push_back(bestCmd);
            }
            else if (m_method == "qacm" || m_method == "proactive-gate")
            {
                // QACM (reactive) and proactive-gate residual conflicts:
                // Compute QoS-aware fraction p from current RSRP margin.
                //   p = 0  → CCO fully wins (ES command cancelled)
                //   p = 1  → ES fully wins
                // Linear: p = clamp(margin / 20, 0, 1) where margin = RSRP - (-95 dBm).
                double worstRsrp  = GetWorstServingRsrpForEnb(e2NodeId);
                double margin     = worstRsrp - kRsrpQosThresholdDbm;
                double p          = SelectQacmFraction(rec.lastSeverity, margin);
                NS_LOG_INFO("Triage QACM: worstRsrp=" << worstRsrp
                            << " margin=" << margin << " p=" << p);
                for (auto& pr : cmds)
                {
                    double d = pr.second->GetPowerDeltaDb();
                    double scaledD = (d < 0.0) ? d * p : d * (1.0 - p);
                    if (std::abs(scaledD) > 1e-9)
                    {
                        Ptr<OranCommandLte2LteTxPower> scaled =
                            CreateObject<OranCommandLte2LteTxPower>();
                        scaled->SetAttribute("TargetE2NodeId", UintegerValue(e2NodeId));
                        scaled->SetAttribute("PowerDeltaDb", DoubleValue(scaledD));
                        result.push_back(scaled);
                    }
                }
            }
            else
            {
                // Unknown method — fall back to priority
                NS_LOG_WARN("Unknown triage method '" << m_method
                            << "'; falling back to priority.");
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandLte2LteTxPower> bestCmd = nullptr;
                for (auto& pr : cmds)
                {
                    int pri = LmPriority(pr.first);
                    if (pri < bestPri)
                    {
                        bestPri = pri;
                        bestCmd = pr.second;
                    }
                }
                if (bestCmd)
                    result.push_back(bestCmd);
            }
        }
    } // for each eNB

    // ── Step 3: classify cell-parameter conflicts (CIO / TTT / RET) ──────────
    // Cell-parameter commands always pass through — this step is CDC-only (logging).
    for (auto& entry : cellParamByNodeAndParam)
    {
        uint64_t   nodeId    = entry.first.first;
        const std::string& paramName = entry.first.second;
        auto& pCmds = entry.second;

        // xApp roles that own this ICP
        auto ownerIt = kP2X.find(paramName);
        if (ownerIt == kP2X.end())
            continue;
        const std::set<std::string>& owners = ownerIt->second;

        // Roles that issued commands for this (eNB, paramName) this cycle
        std::set<std::string> issuingSet;
        for (auto& pr : pCmds)
            issuingSet.insert(LmToXappRole(pr.first));

        // ── Determine conflict type for each affected KPI ────────────────────
        std::set<std::string> directStructural;   // other owners of ICP (KPI affected)
        std::set<std::string> indirectAffected;   // non-owners whose KPI is affected

        auto kpit = kP2K.find(paramName);
        if (kpit != kP2K.end())
        {
            for (const auto& kpi : kpit->second)
            {
                auto kxit = kK2X.find(kpi);
                if (kxit == kK2X.end())
                    continue;
                const std::string& managingRole = kxit->second;
                if (issuingSet.count(managingRole))
                    continue;  // same xApp that issued — not a conflict
                if (!allActiveRoles.count(managingRole))
                    continue;  // managing xApp not active this run

                if (owners.count(managingRole))
                    directStructural.insert(managingRole);   // owns ICP → Direct
                else
                    indirectAffected.insert(managingRole);   // doesn't own ICP → Indirect
            }
        }

        std::vector<std::string> issuingVec(issuingSet.begin(), issuingSet.end());

        // ── Multiple issuers for same ICP → command-level Direct conflict ─────
        if (issuingSet.size() > 1)
        {
            EmitConflictEvent(nodeId, paramName, "Direct", issuingVec, {}, issuingVec[0]);
        }
        else
        {
            // ── Structural Direct: one issuer, but another owner's KPI affected ─
            if (!directStructural.empty())
            {
                std::vector<std::string> structVec(directStructural.begin(),
                                                    directStructural.end());
                EmitConflictEvent(nodeId, paramName, "Direct",
                                  issuingVec, structVec, issuingVec[0]);
            }
            // ── Indirect: non-owner's KPI affected ────────────────────────────
            if (!indirectAffected.empty())
            {
                std::vector<std::string> indVec(indirectAffected.begin(),
                                                 indirectAffected.end());
                EmitConflictEvent(nodeId, paramName, "Indirect",
                                  issuingVec, indVec, issuingVec[0]);
            }
        }
    }

    return result;
}

} // namespace ns3
