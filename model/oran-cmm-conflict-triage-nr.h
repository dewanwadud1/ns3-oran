/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_CMM_CONFLICT_TRIAGE_NR_H
#define ORAN_CMM_CONFLICT_TRIAGE_NR_H

#include "oran-cmm.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \brief NR gNB counterpart of OranCmmConflictTriage.
 *
 * Identical triage framework and CDC classification logic (see
 * oran-cmm-conflict-triage.h for the full design rationale) -- only the
 * bucketed command/data-repository types differ (Nr2Nr commands, Nr-named
 * data-repository accessors) since this scenario keeps LTE and NR as
 * separate example families rather than a mixed-RAT run.
 */
class OranCmmConflictTriageNr : public OranCmm
{
  public:
    static TypeId GetTypeId();
    OranCmmConflictTriageNr();
    ~OranCmmConflictTriageNr() override;

    std::vector<Ptr<OranCommand>> Filter(
        std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
        override;

    std::map<std::string, uint32_t> GetAndResetIcpCounts(uint64_t e2NodeId);

    std::map<std::string, std::set<std::string>> GetAndResetIcpActors(uint64_t e2NodeId);

    struct ConflictEventRecord
    {
        std::string icp;
        std::string type;
        std::vector<std::string> conflicting;
        std::vector<std::string> affected;
        std::string winner;
    };

    std::vector<ConflictEventRecord> GetAndResetConflictEvents(uint64_t e2NodeId);

  private:
    struct ConflictRecord
    {
        uint32_t persistenceCount = 0;
        uint32_t deferCount       = 0;
        uint32_t harmfulCount     = 0;
        uint32_t benignCount      = 0;
        double   lastSeverity     = 0;
    };

    struct FutureCellPrediction
    {
        bool valid = false;
        uint32_t servedUes = 0;
        double ueFraction = 0.0;
        double loadMbps = 0.0;
        double loadFraction = 0.0;
        double worstPredictedRsrpDbm = 0.0;
    };

    double ComputeSeverity(double negDelta, double posDelta) const;
    double ComputeTriageScore(const ConflictRecord& rec,
                              double severity,
                              double scopeFraction) const;
    double SelectQacmFraction(double severity, double rsrpMarginDb) const;
    double GetWorstServingRsrpForEnb(uint64_t e2NodeId) const;
    double GetNrDemandMbpsForGnb(uint64_t e2NodeId) const;
    double GetNrCellScopeFraction(uint64_t e2NodeId) const;
    FutureCellPrediction PredictFutureCellState(uint64_t e2NodeId,
                                                double proposedDeltaDb) const;
    double ComputeNoisyOr(const std::vector<double>& evidence) const;
    double ComputeStructuralIcpEvidence(const std::string& icp,
                                        const std::string& issuingRole,
                                        const std::set<std::string>& activeRoles) const;
    double ComputeKpiEvidence(const std::string& icp,
                              const std::string& issuingRole,
                              const std::set<std::string>& activeRoles) const;
    double ComputeProactiveConflictLikelihood(uint64_t e2NodeId,
                                              const std::string& issuingRole,
                                              const std::string& icp,
                                              double proposedDeltaDb,
                                              const std::set<std::string>& activeRoles,
                                              std::vector<double>* evidenceOut) const;

    int LmPriority(const std::string& lmName) const;

    std::string LmToXappRole(const std::string& lmName) const;

    void EmitConflictEvent(uint64_t          enbE2Id,
                           const std::string& icp,
                           const std::string& type,
                           const std::vector<std::string>& conflicting,
                           const std::vector<std::string>& affected,
                           const std::string& winnerRole);

    /**
     * True if this gNB's worst serving RSRP is currently below the QoS
     * threshold, checked same-cycle. CDC only tags a Direct/Indirect
     * conflict when this is true -- structural param-issuer overlap alone
     * (e.g. a routine MLB CIO nudge with no other xApp under stress) is not
     * a conflict by itself. Same-cycle is empirically justified: a
     * dedicated large-amplitude TxPower step-response calibration (see
     * OranLmNr2NrTxpCalibration) showed RSRP responds same-cycle 98% of the
     * time (median lag 0). SINR does not show a same-cycle (or any
     * background-adjusted) signal, so SINR-based gating is deliberately not
     * implemented yet -- a known limitation, not an oversight.
     */
    bool IsRsrpKpiViolated(uint64_t e2NodeId) const;

    std::map<uint64_t, ConflictRecord> m_txpConflicts;
    std::map<uint64_t, std::map<std::string, uint32_t>> m_icpCounts;
    std::map<uint64_t, std::map<std::string, std::set<std::string>>> m_icpActors;
    std::map<uint64_t, std::vector<ConflictEventRecord>> m_conflictEvents;

    static constexpr double kRsrpQosThresholdDbm = -95.0;

    std::string m_method;
    double      m_tolerateThresh;
    double      m_deferThresh;
    uint32_t    m_deferMax;
    double      m_dampFraction;
    double      m_qacmZeta;
    double      m_wSeverity;
    double      m_wPersistence;
    double      m_wScope;
    double      m_proactiveRiskThreshold;
    double      m_proactiveRsrpGuardDb;
    double      m_proactiveCellCapacityMbps;
    uint32_t    m_predictionHorizonCycles;
    double      m_lmIntervalSec;
    double      m_positionLookbackSec;
    double      m_rsrpPathlossExponent;
    std::string m_conflictLogFile;
    std::ofstream m_logStream;
};

} // namespace ns3

#endif // ORAN_CMM_CONFLICT_TRIAGE_NR_H
