/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_CMM_CONFLICT_TRIAGE_H
#define ORAN_CMM_CONFLICT_TRIAGE_H

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
 * \brief Generic conflict triage module for the four-xApp O-RAN scenario.
 *
 * Implements the cost-aware triage framework described in the TNSE paper
 * (Chapter 5 of the PhD thesis). Mediates conflicts between ES, CCO, MRO,
 * and MLB xApps.
 *
 * Triage decisions per conflict:
 *   tolerate  — all commands pass unchanged (severity too low to act)
 *   defer     — lower-priority command held for up to D_max cycles
 *   mitigate  — apply the configured mechanism (cancel/dampen/priority/qacm)
 *
 * Feature vector (8D proxy for the full 9D TNSE model):
 *   [severity, degradation, persistence, scope, kpi_criticality,
 *    system_load, uncertainty, recurrence]
 *
 * Conflict triage score:
 *   score = w_sev*severity + w_deg*degradation + w_per*persistence
 *           + w_sco*scope  + w_cri*kpi_criticality
 *
 * Mechanism selection (configurable via --method):
 *   cancel   — cancel the lower-priority xApp command
 *   dampen   — scale the lower-priority command by (1-dampFraction)
 *   priority — MRO > CCO > MLB > ES; only highest-priority passes
 *   qacm     — game-theoretic sweep of fraction p in [0,1]
 */
class OranCmmConflictTriage : public OranCmm
{
  public:
    static TypeId GetTypeId();
    OranCmmConflictTriage();
    ~OranCmmConflictTriage() override;

    std::vector<Ptr<OranCommand>> Filter(
        std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
        override;

  private:
    // ── Per-conflict tracking record ────────────────────────────────────────
    struct ConflictRecord
    {
        uint32_t persistenceCount = 0; //!< Consecutive cycles with this conflict
        uint32_t deferCount       = 0; //!< Times deferred so far
        uint32_t harmfulCount     = 0; //!< Times classified as harmful
        uint32_t benignCount      = 0; //!< Times classified as benign (tolerated)
        double   lastSeverity     = 0; //!< Severity at last detection
    };

    // ── Triage helpers ───────────────────────────────────────────────────────
    double ComputeSeverity(double negDelta, double posDelta) const;
    double ComputeTriageScore(const ConflictRecord& rec,
                              double severity,
                              double scopeFraction) const;
    // QoS-aware QACM fraction: p=0 → CCO fully wins; p=1 → ES fully wins.
    // rsrpMarginDb = currentRsrp - kRsrpQosThresholdDbm (negative = violation).
    double SelectQacmFraction(double severity, double rsrpMarginDb) const;
    // Query worst (minimum) serving-cell RSRP for an eNB from the data repository.
    double GetWorstServingRsrpForEnb(uint64_t e2NodeId) const;

    // ── Priority rank (lower = higher priority) ──────────────────────────────
    int LmPriority(const std::string& lmName) const;

    // ── CDC: rule-based conflict classification (Algorithm 1) ────────────────
    // Map an LM name to its xApp role label (ES | MRO | CCO | MLB | Unknown).
    std::string LmToXappRole(const std::string& lmName) const;

    // Classify and emit a conflict event.
    //   icp         — the controlled parameter (e.g. "TxPower", "CIO", "TTT", "RET")
    //   type        — "Direct" or "Indirect"
    //   conflicting — xApp roles that issued commands for this ICP
    //   affected    — xApp roles whose KPIs are affected but did not issue a command
    //   winnerRole  — xApp role whose command the CMM kept (empty for Indirect)
    void EmitConflictEvent(uint64_t enbE2Id,
                           const std::string& icp,
                           const std::string& type,
                           const std::vector<std::string>& conflicting,
                           const std::vector<std::string>& affected,
                           const std::string& winnerRole);

    // ── State ────────────────────────────────────────────────────────────────
    // Key: e2NodeId of the conflicting eNB
    std::map<uint64_t, ConflictRecord> m_txpConflicts;

    // ── Configuration ────────────────────────────────────────────────────────
    static constexpr double kRsrpQosThresholdDbm = -95.0; //!< Coverage QoS floor (dBm)

    std::string m_method;          //!< noop | cancel | dampen | priority | qacm | proactive-gate
    double      m_tolerateThresh;  //!< Triage score below which → tolerate
    double      m_deferThresh;     //!< Score below which (if > tolerate) → defer
    uint32_t    m_deferMax;        //!< D_max: max deferral cycles before forced mitigate
    double      m_dampFraction;    //!< Fraction removed from lower-priority delta (dampen)
    double      m_qacmZeta;        //!< Exponent for QACM utility curve
    double      m_wSeverity;       //!< Feature weight: severity
    double      m_wPersistence;    //!< Feature weight: persistence
    double      m_wScope;          //!< Feature weight: scope
    std::string m_conflictLogFile; //!< Path for CDC CSV output (empty = disabled)
    std::ofstream m_logStream;     //!< CDC CSV output stream (opened lazily)
};

} // namespace ns3

#endif // ORAN_CMM_CONFLICT_TRIAGE_H
