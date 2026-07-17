/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-cmm-conflict-triage-nr.h"

#include "oran-command-nr-2-nr-cell-parameter.h"
#include "oran-command-nr-2-nr-handover.h"
#include "oran-command-nr-2-nr-tx-power.h"
#include "oran-command.h"
#include "oran-data-repository.h"
#include "oran-near-rt-ric.h"
#include "oran-nr-cell-control-state.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/vector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCmmConflictTriageNr");
NS_OBJECT_ENSURE_REGISTERED(OranCmmConflictTriageNr);

// ── CDC relationship tables (Algorithm 1 from Computer Networks manuscript) ──
static const std::map<std::string, std::set<std::string>> kP2X = {
    {"TxPower", {"ES", "MRO", "CCO", "MLB"}},
    {"CIO",     {"MRO", "MLB"}},
    {"TTT",     {"MRO", "MLB"}},
    {"RET",     {"CCO", "MLB"}},
};

static const std::map<std::string, std::vector<std::string>> kP2K = {
    {"TxPower", {"RSRP", "SINR", "Throughput", "EE", "CDR", "HSR"}},
    {"CIO",     {"CDR", "HSR", "TL", "RSRP-bias"}},
    {"TTT",     {"CDR", "HSR", "TL"}},
    {"RET",     {"RSRP", "SINR", "CDR", "HSR"}},
};

static const std::map<std::string, std::string> kK2X = {
    {"EE",        "ES"},
    {"SINR",      "CCO"}, {"Throughput", "CCO"}, {"RSRP", "CCO"},
    {"CDR",       "MRO"}, {"HSR",        "MRO"}, {"CBR",  "MRO"}, {"RSRP-bias", "MRO"},
    {"TL",        "MLB"}, {"RUR",        "MLB"},
};

static double
Distance2d(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

TypeId
OranCmmConflictTriageNr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCmmConflictTriageNr")
            .SetParent<OranCmm>()
            .AddConstructor<OranCmmConflictTriageNr>()
            .AddAttribute("Method",
                          "Mitigation mechanism: cancel | dampen | priority | qacm",
                          StringValue("priority"),
                          MakeStringAccessor(&OranCmmConflictTriageNr::m_method),
                          MakeStringChecker())
            .AddAttribute("TolerateThreshold",
                          "Triage score below which conflict is tolerated (all commands pass).",
                          DoubleValue(0.15),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_tolerateThresh),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("DeferThreshold",
                          "Triage score below which conflict is deferred (if D_max not reached).",
                          DoubleValue(0.40),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_deferThresh),
                          MakeDoubleChecker<double>(0.0, 2.0))
            .AddAttribute("DeferMax",
                          "Maximum consecutive deferral cycles (D_max) before forced mitigation.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&OranCmmConflictTriageNr::m_deferMax),
                          MakeUintegerChecker<uint32_t>(0))
            .AddAttribute("DampFraction",
                          "Fraction of the delta removed from lower-priority command (dampen).",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_dampFraction),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("QacmZeta",
                          "Scale for QACM weighted-distance objective.",
                          DoubleValue(100.0),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_qacmZeta),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightSeverity",
                          "Triage score weight for conflict severity.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_wSeverity),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightPersistence",
                          "Triage score weight for conflict persistence.",
                          DoubleValue(0.3),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_wPersistence),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("WeightScope",
                          "Triage score weight for conflict scope (fraction of UEs affected).",
                          DoubleValue(0.2),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_wScope),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ProactiveRiskThreshold",
                          "Noisy-OR conflict likelihood above which proactive-gate suppresses "
                          "a harmful ES TxPower reduction.",
                          DoubleValue(0.65),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_proactiveRiskThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("ProactiveRsrpGuardDb",
                          "RSRP guard band above the QoS floor used to turn predicted RSRP "
                          "margin into runtime-risk evidence.",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_proactiveRsrpGuardDb),
                          MakeDoubleChecker<double>(0.1))
            .AddAttribute("ProactiveCellCapacityMbps",
                          "Per-gNB capacity used to normalize demand/load evidence in the "
                          "proactive conflict prior.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_proactiveCellCapacityMbps),
                          MakeDoubleChecker<double>(0.001))
            .AddAttribute("PredictionHorizonCycles",
                          "Number of LM cycles used for proactive trajectory/load/KPI lookahead.",
                          UintegerValue(5),
                          MakeUintegerAccessor(&OranCmmConflictTriageNr::m_predictionHorizonCycles),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("LmIntervalSec",
                          "LM query interval in seconds; horizon time is "
                          "PredictionHorizonCycles * LmIntervalSec.",
                          DoubleValue(0.5),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_lmIntervalSec),
                          MakeDoubleChecker<double>(0.001))
            .AddAttribute("PositionLookbackSec",
                          "Recent position window used to estimate UE velocity for trajectory "
                          "prediction.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_positionLookbackSec),
                          MakeDoubleChecker<double>(0.001))
            .AddAttribute("RsrpPathlossExponent",
                          "Distance-ratio path-loss exponent used to project future RSRP "
                          "from current measurements.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&OranCmmConflictTriageNr::m_rsrpPathlossExponent),
                          MakeDoubleChecker<double>(1.0, 6.0))
            .AddAttribute("ConflictLogFile",
                          "Path to CDC CSV output file (empty = disabled). "
                          "Columns: time_s,enb_e2id,icp,type,conflicting,affected,winner.",
                          StringValue(""),
                          MakeStringAccessor(&OranCmmConflictTriageNr::m_conflictLogFile),
                          MakeStringChecker());
    return tid;
}

OranCmmConflictTriageNr::OranCmmConflictTriageNr()
    : OranCmm()
{
    NS_LOG_FUNCTION(this);
    m_name = "CmmConflictTriageNr";
}

OranCmmConflictTriageNr::~OranCmmConflictTriageNr()
{
    NS_LOG_FUNCTION(this);
}

std::map<std::string, uint32_t>
OranCmmConflictTriageNr::GetAndResetIcpCounts(uint64_t e2NodeId)
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
OranCmmConflictTriageNr::GetAndResetIcpActors(uint64_t e2NodeId)
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

std::vector<OranCmmConflictTriageNr::ConflictEventRecord>
OranCmmConflictTriageNr::GetAndResetConflictEvents(uint64_t e2NodeId)
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
OranCmmConflictTriageNr::LmToXappRole(const std::string& lmName) const
{
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
OranCmmConflictTriageNr::EmitConflictEvent(uint64_t          enbE2Id,
                                            const std::string& icp,
                                            const std::string& type,
                                            const std::vector<std::string>& conflicting,
                                            const std::vector<std::string>& affected,
                                            const std::string& winnerRole)
{
    double now = Simulator::Now().GetSeconds();

    m_conflictEvents[enbE2Id].push_back(
        ConflictEventRecord{icp, type, conflicting, affected, winnerRole});

    std::ostringstream oss;
    oss << "[CDC] t=" << now << "s gNB=" << enbE2Id
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

int
OranCmmConflictTriageNr::LmPriority(const std::string& lmName) const
{
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
OranCmmConflictTriageNr::ComputeSeverity(double negDelta, double posDelta) const
{
    return std::min(1.0, (std::abs(negDelta) + std::abs(posDelta)) / 10.0);
}

double
OranCmmConflictTriageNr::ComputeTriageScore(const ConflictRecord& rec,
                                            double severity,
                                            double scopeFraction) const
{
    double persistenceNorm = std::min(1.0, rec.persistenceCount / 10.0);
    return m_wSeverity * severity + m_wPersistence * persistenceNorm +
           m_wScope * std::min(1.0, scopeFraction);
}

double
OranCmmConflictTriageNr::GetWorstServingRsrpForEnb(uint64_t e2NodeId) const
{
    if (m_nearRtRic == nullptr)
        return kRsrpQosThresholdDbm - 1.0;

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    bool foundCell = false;
    uint16_t cellId = 0;
    std::tie(foundCell, cellId) = data->GetNrGnbCellInfo(e2NodeId);
    if (!foundCell)
        return kRsrpQosThresholdDbm - 1.0;

    double worstRsrp = 0.0;
    bool   hasRsrp   = false;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool ufound = false; uint16_t ueCell = 0, ueRnti = 0;
        std::tie(ufound, ueCell, ueRnti) = data->GetNrUeCellInfo(ueId);
        if (!ufound || ueCell != cellId)
            continue;
        for (const auto& meas : data->GetNrUeRsrpRsrq(ueId))
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

bool
OranCmmConflictTriageNr::IsRsrpKpiViolated(uint64_t e2NodeId) const
{
    return GetWorstServingRsrpForEnb(e2NodeId) < kRsrpQosThresholdDbm;
}

double
OranCmmConflictTriageNr::SelectQacmFraction(double /*severity*/, double rsrpMarginDb) const
{
    return std::max(0.0, std::min(1.0, rsrpMarginDb / 20.0));
}

double
OranCmmConflictTriageNr::GetNrDemandMbpsForGnb(uint64_t e2NodeId) const
{
    if (m_nearRtRic == nullptr)
        return 0.0;

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    bool foundCell = false;
    uint16_t cellId = 0;
    std::tie(foundCell, cellId) = data->GetNrGnbCellInfo(e2NodeId);
    if (!foundCell)
        return 0.0;

    double demandMbps = 0.0;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool foundUe = false;
        uint16_t ueCell = 0;
        uint16_t ueRnti = 0;
        std::tie(foundUe, ueCell, ueRnti) = data->GetNrUeCellInfo(ueId);
        if (foundUe && ueCell == cellId)
            demandMbps += data->GetNrUeAppDemand(ueId);
    }
    return demandMbps;
}

double
OranCmmConflictTriageNr::GetNrCellScopeFraction(uint64_t e2NodeId) const
{
    if (m_nearRtRic == nullptr)
        return 0.0;

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    bool foundCell = false;
    uint16_t cellId = 0;
    std::tie(foundCell, cellId) = data->GetNrGnbCellInfo(e2NodeId);
    if (!foundCell)
        return 0.0;

    uint32_t totalUes = 0;
    uint32_t cellUes = 0;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool foundUe = false;
        uint16_t ueCell = 0;
        uint16_t ueRnti = 0;
        std::tie(foundUe, ueCell, ueRnti) = data->GetNrUeCellInfo(ueId);
        if (!foundUe)
            continue;
        totalUes++;
        if (ueCell == cellId)
            cellUes++;
    }
    return totalUes > 0 ? static_cast<double>(cellUes) / totalUes : 0.0;
}

OranCmmConflictTriageNr::FutureCellPrediction
OranCmmConflictTriageNr::PredictFutureCellState(uint64_t e2NodeId, double proposedDeltaDb) const
{
    FutureCellPrediction out;
    if (m_nearRtRic == nullptr)
        return out;

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    const Time now = Simulator::Now();
    const Time lookback = Seconds(m_positionLookbackSec);
    const double horizonSec =
        static_cast<double>(m_predictionHorizonCycles) * m_lmIntervalSec;

    std::map<uint16_t, uint64_t> cellToE2;
    std::map<uint64_t, Vector> gnbPos;
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        bool foundCell = false;
        uint16_t cellId = 0;
        std::tie(foundCell, cellId) = data->GetNrGnbCellInfo(gnbId);
        if (!foundCell)
            continue;

        auto posHist = data->GetNodePositions(gnbId, Seconds(0), now, 1);
        if (posHist.empty())
            continue;
        cellToE2[cellId] = gnbId;
        gnbPos[gnbId] = posHist.rbegin()->second;
    }

    bool foundTargetCell = false;
    uint16_t targetCellId = 0;
    std::tie(foundTargetCell, targetCellId) = data->GetNrGnbCellInfo(e2NodeId);
    if (!foundTargetCell || gnbPos.find(e2NodeId) == gnbPos.end())
        return out;

    uint32_t totalUes = 0;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        const Time from = now.GetSeconds() > m_positionLookbackSec ? now - lookback : Seconds(0);
        auto uePosHist = data->GetNodePositions(ueId, from, now, 2);
        if (uePosHist.empty())
            continue;

        Vector currentPos = uePosHist.rbegin()->second;
        Vector velocity(0.0, 0.0, 0.0);
        if (uePosHist.size() >= 2)
        {
            auto latest = uePosHist.rbegin();
            auto prev = latest;
            ++prev;
            const double dt = (latest->first - prev->first).GetSeconds();
            if (dt > 1e-9)
            {
                velocity.x = (latest->second.x - prev->second.x) / dt;
                velocity.y = (latest->second.y - prev->second.y) / dt;
                velocity.z = (latest->second.z - prev->second.z) / dt;
            }
        }

        Vector futurePos(currentPos.x + velocity.x * horizonSec,
                         currentPos.y + velocity.y * horizonSec,
                         currentPos.z + velocity.z * horizonSec);

        std::vector<std::tuple<uint16_t, uint16_t, double, double, bool, uint8_t>> meas =
            data->GetNrUeRsrpRsrq(ueId);
        if (meas.empty())
            continue;

        totalUes++;

        double bestEffectiveRsrp = -std::numeric_limits<double>::infinity();
        double bestRawRsrp = -std::numeric_limits<double>::infinity();
        uint64_t bestE2 = 0;

        for (const auto& m : meas)
        {
            uint16_t rnti = 0;
            uint16_t cellId = 0;
            double rsrp = 0.0;
            double rsrq = 0.0;
            bool serving = false;
            uint8_t ccid = 0;
            std::tie(rnti, cellId, rsrp, rsrq, serving, ccid) = m;
            if (!std::isfinite(rsrp))
                continue;

            auto e2It = cellToE2.find(cellId);
            if (e2It == cellToE2.end())
                continue;
            uint64_t candidateE2 = e2It->second;
            auto posIt = gnbPos.find(candidateE2);
            if (posIt == gnbPos.end())
                continue;

            const double dNow = std::max(1.0, Distance2d(currentPos, posIt->second));
            const double dFuture = std::max(1.0, Distance2d(futurePos, posIt->second));
            double predictedRsrp =
                rsrp - 10.0 * m_rsrpPathlossExponent * std::log10(dFuture / dNow);
            if (candidateE2 == e2NodeId)
                predictedRsrp += proposedDeltaDb;

            const OranNrCellControlParams cp = GetNrCellControlParameters(candidateE2);
            const double effectiveRsrp = predictedRsrp + cp.cioDb;
            if (effectiveRsrp > bestEffectiveRsrp)
            {
                bestEffectiveRsrp = effectiveRsrp;
                bestRawRsrp = predictedRsrp;
                bestE2 = candidateE2;
            }
        }

        if (bestE2 == e2NodeId)
        {
            out.valid = true;
            out.servedUes++;
            out.loadMbps += data->GetNrUeAppDemand(ueId);
            if (out.servedUes == 1 || bestRawRsrp < out.worstPredictedRsrpDbm)
                out.worstPredictedRsrpDbm = bestRawRsrp;
        }
    }

    if (totalUes > 0)
    {
        out.valid = true;
        out.ueFraction = static_cast<double>(out.servedUes) / totalUes;
    }
    out.loadFraction = std::max(0.0, std::min(1.0, out.loadMbps / m_proactiveCellCapacityMbps));
    return out;
}

double
OranCmmConflictTriageNr::ComputeNoisyOr(const std::vector<double>& evidence) const
{
    double survival = 1.0;
    for (double p : evidence)
    {
        double bounded = std::max(0.0, std::min(1.0, p));
        survival *= (1.0 - bounded);
    }
    return 1.0 - survival;
}

double
OranCmmConflictTriageNr::ComputeStructuralIcpEvidence(
    const std::string& icp,
    const std::string& issuingRole,
    const std::set<std::string>& activeRoles) const
{
    auto it = kP2X.find(icp);
    if (it == kP2X.end())
        return 0.0;

    uint32_t competingActiveOwners = 0;
    uint32_t possibleCompetingOwners = 0;
    for (const auto& owner : it->second)
    {
        if (owner == issuingRole)
            continue;
        possibleCompetingOwners++;
        if (activeRoles.count(owner))
            competingActiveOwners++;
    }

    if (possibleCompetingOwners == 0)
        return 0.0;
    return static_cast<double>(competingActiveOwners) / possibleCompetingOwners;
}

double
OranCmmConflictTriageNr::ComputeKpiEvidence(const std::string& icp,
                                            const std::string& issuingRole,
                                            const std::set<std::string>& activeRoles) const
{
    auto it = kP2K.find(icp);
    if (it == kP2K.end() || it->second.empty())
        return 0.0;

    uint32_t affectedManagedKpis = 0;
    for (const auto& kpi : it->second)
    {
        auto xit = kK2X.find(kpi);
        if (xit == kK2X.end())
            continue;
        const std::string& managingRole = xit->second;
        if (managingRole != issuingRole && activeRoles.count(managingRole))
            affectedManagedKpis++;
    }
    return static_cast<double>(affectedManagedKpis) / it->second.size();
}

double
OranCmmConflictTriageNr::ComputeProactiveConflictLikelihood(
    uint64_t e2NodeId,
    const std::string& issuingRole,
    const std::string& icp,
    double proposedDeltaDb,
    const std::set<std::string>& activeRoles,
    std::vector<double>* evidenceOut) const
{
    std::vector<double> evidence;

    double pIcp = ComputeStructuralIcpEvidence(icp, issuingRole, activeRoles);
    double pKpi = ComputeKpiEvidence(icp, issuingRole, activeRoles);

    double pSeverity = std::max(0.0, std::min(1.0, std::abs(proposedDeltaDb) / 10.0));

    FutureCellPrediction future = PredictFutureCellState(e2NodeId, proposedDeltaDb);

    double pScope = future.valid ? future.ueFraction : GetNrCellScopeFraction(e2NodeId);

    double predictedWorstRsrp = kRsrpQosThresholdDbm + m_proactiveRsrpGuardDb;
    if (future.valid)
    {
        if (future.servedUes > 0)
            predictedWorstRsrp = future.worstPredictedRsrpDbm;
    }
    else
    {
        predictedWorstRsrp = GetWorstServingRsrpForEnb(e2NodeId) + proposedDeltaDb;
    }
    double predictedMarginDb = predictedWorstRsrp - kRsrpQosThresholdDbm;
    double pRsrp = 1.0 - (predictedMarginDb / m_proactiveRsrpGuardDb);
    pRsrp = std::max(0.0, std::min(1.0, pRsrp));

    double pLoad = future.valid
                       ? future.loadFraction
                       : std::max(0.0,
                                  std::min(1.0,
                                           GetNrDemandMbpsForGnb(e2NodeId) /
                                               m_proactiveCellCapacityMbps));
    double pRuntime = ComputeNoisyOr({pRsrp, pLoad});

    evidence.push_back(pIcp);
    evidence.push_back(pKpi);
    evidence.push_back(pSeverity);
    evidence.push_back(pScope);
    evidence.push_back(pRuntime);

    if (evidenceOut != nullptr)
        *evidenceOut = evidence;

    double structuralPrior = ComputeNoisyOr({pIcp, pKpi, pSeverity, pScope});
    return structuralPrior * pRuntime;
}

// ── Main Filter ───────────────────────────────────────────────────────────────
std::vector<Ptr<OranCommand>>
OranCmmConflictTriageNr::Filter(
    std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run ConflictTriageNr CMM with NULL Near-RT RIC");

    std::set<std::string> allActiveRoles;
    for (const auto& entry : inputCommands)
        allActiveRoles.insert(LmToXappRole(std::get<0>(entry.first)));

    if (m_method == "proactive-gate")
    {
        std::set<uint64_t> predictorTargets;
        for (const auto& entry : inputCommands)
        {
            if (std::get<0>(entry.first).find("KpiPrediction") == std::string::npos)
                continue;
            for (auto cmd : entry.second)
            {
                Ptr<OranCommandNr2NrTxPower> txp = cmd->GetObject<OranCommandNr2NrTxPower>();
                if (txp && txp->GetPowerDeltaDb() > 0.0)
                    predictorTargets.insert(txp->GetTargetE2NodeId());
            }
        }
        uint32_t suppressed = 0;
        for (auto& entry : inputCommands)
        {
            if (LmToXappRole(std::get<0>(entry.first)) != "ES")
                continue;
            auto& cmds = entry.second;
            cmds.erase(
                std::remove_if(cmds.begin(), cmds.end(),
                    [this, &predictorTargets, &suppressed](Ptr<OranCommand> cmd) {
                        Ptr<OranCommandNr2NrTxPower> txp =
                            cmd->GetObject<OranCommandNr2NrTxPower>();
                        if (txp && txp->GetPowerDeltaDb() < 0.0 &&
                            predictorTargets.count(txp->GetTargetE2NodeId()))
                        {
                            suppressed++;
                            EmitConflictEvent(txp->GetTargetE2NodeId(),
                                              "TxPower",
                                              "Prevented",
                                              {"ES"},
                                              {"KPI-Predictor"},
                                              "ProactiveGate");
                            return true;
                        }
                        return false;
                    }),
                cmds.end());
        }

        uint32_t priorSuppressed = 0;
        for (auto& entry : inputCommands)
        {
            const std::string role = LmToXappRole(std::get<0>(entry.first));
            if (role != "ES")
                continue;

            auto& cmds = entry.second;
            cmds.erase(
                std::remove_if(cmds.begin(),
                               cmds.end(),
                               [this, &allActiveRoles, &priorSuppressed, role](Ptr<OranCommand> cmd) {
                                   Ptr<OranCommandNr2NrTxPower> txp =
                                       cmd->GetObject<OranCommandNr2NrTxPower>();
                                   if (!txp || txp->GetPowerDeltaDb() >= 0.0)
                                       return false;

                                   std::vector<double> evidence;
                                   double likelihood = ComputeProactiveConflictLikelihood(
                                       txp->GetTargetE2NodeId(),
                                       role,
                                       "TxPower",
                                       txp->GetPowerDeltaDb(),
                                       allActiveRoles,
                                       &evidence);

                                   std::ostringstream msg;
                                   msg << "[GATE] t=" << Simulator::Now().GetSeconds()
                                       << "s e2=" << txp->GetTargetE2NodeId()
                                       << " xapp=" << role
                                       << " icp=TxPower delta=" << txp->GetPowerDeltaDb()
                                       << " likelihood=" << likelihood
                                       << " evidence={icp=" << evidence[0]
                                       << ",kpi=" << evidence[1]
                                       << ",severity=" << evidence[2]
                                       << ",scope=" << evidence[3]
                                       << ",runtime=" << evidence[4] << "}"
                                       << " threshold=" << m_proactiveRiskThreshold;

                                   if (likelihood >= m_proactiveRiskThreshold)
                                   {
                                       msg << " decision=SUPPRESS";
                                       std::cout << msg.str() << "\n";
                                       NS_LOG_INFO(msg.str());
                                       LogLogicToStorage(msg.str());
                                       std::vector<std::string> affected;
                                       for (const auto& activeRole : allActiveRoles)
                                       {
                                           if (activeRole != role)
                                               affected.push_back(activeRole);
                                       }
                                       EmitConflictEvent(txp->GetTargetE2NodeId(),
                                                         "TxPower",
                                                         "Prevented",
                                                         {role},
                                                         affected,
                                                         "ProactiveGate");
                                       priorSuppressed++;
                                       return true;
                                   }

                                   msg << " decision=ALLOW";
                                   NS_LOG_INFO(msg.str());
                                   LogLogicToStorage(msg.str());
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
                    << " predicted-risk gNB(s)";
            std::cout << gateMsg.str() << "\n";
            NS_LOG_INFO(gateMsg.str());
        }
        if (priorSuppressed > 0)
        {
            std::ostringstream gateMsg;
            gateMsg << "[GATE] t=" << Simulator::Now().GetSeconds()
                    << "s noisy-or prior suppressed " << priorSuppressed
                    << " ES TXP- command(s)";
            std::cout << gateMsg.str() << "\n";
            NS_LOG_INFO(gateMsg.str());
        }
    }

    std::map<uint64_t, std::vector<std::pair<std::string, Ptr<OranCommandNr2NrTxPower>>>>
        txpByNode;

    std::map<std::pair<uint64_t, std::string>,
             std::vector<std::pair<std::string, Ptr<OranCommandNr2NrCellParameter>>>>
        cellParamByNodeAndParam;

    std::vector<Ptr<OranCommand>> passThrough;

    double nowCmdLog = Simulator::Now().GetSeconds();
    for (const auto& entry : inputCommands)
    {
        const std::string& lmName = std::get<0>(entry.first);
        const std::string  role   = LmToXappRole(lmName);
        for (auto cmd : entry.second)
        {
            uint64_t e2id = cmd->GetTargetE2NodeId();
            Ptr<OranCommandNr2NrTxPower> txp = cmd->GetObject<OranCommandNr2NrTxPower>();
            if (txp != nullptr)
            {
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
                Ptr<OranCommandNr2NrCellParameter> cp =
                    cmd->GetObject<OranCommandNr2NrCellParameter>();
                if (cp != nullptr)
                {
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
                passThrough.push_back(cmd);
            }
        }
    }

    std::vector<Ptr<OranCommand>> result = passThrough;

    for (auto& nodeEntry : txpByNode)
    {
        uint64_t e2NodeId = nodeEntry.first;
        auto& cmds        = nodeEntry.second;

        if (cmds.size() <= 1)
        {
            for (auto& pr : cmds)
                result.push_back(pr.second);
            if (m_txpConflicts.count(e2NodeId))
            {
                m_txpConflicts[e2NodeId].persistenceCount = 0;
                m_txpConflicts[e2NodeId].deferCount       = 0;
            }
            continue;
        }

        std::set<std::string> txpRoles;
        for (auto& pr : cmds)
            txpRoles.insert(LmToXappRole(pr.first));
        if (txpRoles.size() <= 1)
        {
            for (auto& pr : cmds)
                result.push_back(pr.second);
            if (m_txpConflicts.count(e2NodeId))
            {
                m_txpConflicts[e2NodeId].persistenceCount = 0;
                m_txpConflicts[e2NodeId].deferCount       = 0;
            }
            continue;
        }

        double negSum = 0.0;
        double posSum = 0.0;
        for (auto& pr : cmds)
        {
            double d = pr.second->GetPowerDeltaDb();
            if (d < 0.0) negSum += d;
            else          posSum += d;
        }

        auto& rec = m_txpConflicts[e2NodeId];
        rec.persistenceCount++;
        rec.lastSeverity = ComputeSeverity(negSum, posSum);

        double triageScore = ComputeTriageScore(rec, rec.lastSeverity, 1.0);

        if (IsRsrpKpiViolated(e2NodeId))
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

        if (m_method == "noop")
        {
            msg << " decision=NOOP(ES-wins)";
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());
            int worstPri = -1;
            Ptr<OranCommandNr2NrTxPower> esCmd = nullptr;
            for (auto& pr : cmds)
            {
                int pri = LmPriority(pr.first);
                if (pri > worstPri) { worstPri = pri; esCmd = pr.second; }
            }
            if (esCmd)
                result.push_back(esCmd);
        }
        else if (triageScore < m_tolerateThresh)
        {
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
            msg << " decision=DEFER deferCount=" << rec.deferCount + 1;
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());
            rec.deferCount++;
            int bestPri = std::numeric_limits<int>::max();
            Ptr<OranCommandNr2NrTxPower> bestCmd = nullptr;
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
            rec.harmfulCount++;
            rec.deferCount = 0;
            msg << " decision=MITIGATE method=" << m_method;
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());

            if (m_method == "cancel")
            {
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandNr2NrTxPower> bestCmd = nullptr;
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
                int bestPri = std::numeric_limits<int>::max();
                for (auto& pr : cmds)
                    bestPri = std::min(bestPri, LmPriority(pr.first));

                for (auto& pr : cmds)
                {
                    int pri = LmPriority(pr.first);
                    if (pri == bestPri)
                    {
                        result.push_back(pr.second);
                    }
                    else
                    {
                        double scaledDelta = pr.second->GetPowerDeltaDb() * (1.0 - m_dampFraction);
                        if (std::abs(scaledDelta) > 1e-9)
                        {
                            Ptr<OranCommandNr2NrTxPower> dampened =
                                CreateObject<OranCommandNr2NrTxPower>();
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
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandNr2NrTxPower> bestCmd = nullptr;
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
                        Ptr<OranCommandNr2NrTxPower> scaled =
                            CreateObject<OranCommandNr2NrTxPower>();
                        scaled->SetAttribute("TargetE2NodeId", UintegerValue(e2NodeId));
                        scaled->SetAttribute("PowerDeltaDb", DoubleValue(scaledD));
                        result.push_back(scaled);
                    }
                }
            }
            else
            {
                NS_LOG_WARN("Unknown triage method '" << m_method
                            << "'; falling back to priority.");
                int bestPri = std::numeric_limits<int>::max();
                Ptr<OranCommandNr2NrTxPower> bestCmd = nullptr;
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
    } // for each gNB

    for (auto& entry : cellParamByNodeAndParam)
    {
        uint64_t   nodeId    = entry.first.first;
        const std::string& paramName = entry.first.second;
        auto& pCmds = entry.second;

        if (kP2X.find(paramName) == kP2X.end())
            continue;

        std::set<std::string> issuingSet;
        for (auto& pr : pCmds)
            issuingSet.insert(LmToXappRole(pr.first));

        std::set<std::string> affectedRoles;

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
                    continue;
                if (!allActiveRoles.count(managingRole))
                    continue;

                affectedRoles.insert(managingRole);
            }
        }

        std::vector<std::string> issuingVec(issuingSet.begin(), issuingSet.end());

        if (issuingSet.size() > 1)
        {
            if (IsRsrpKpiViolated(nodeId))
            {
                EmitConflictEvent(nodeId, paramName, "Direct", issuingVec, {}, issuingVec[0]);
            }
        }
        else
        {
            if (!affectedRoles.empty() && IsRsrpKpiViolated(nodeId))
            {
                std::vector<std::string> indVec(affectedRoles.begin(), affectedRoles.end());
                EmitConflictEvent(nodeId, paramName, "Indirect",
                                  issuingVec, indVec, issuingVec[0]);
            }
        }
    }

    return result;
}

} // namespace ns3
