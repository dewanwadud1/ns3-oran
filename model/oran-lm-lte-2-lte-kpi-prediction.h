/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_LM_LTE_2_LTE_KPI_PREDICTION_H
#define ORAN_LM_LTE_2_LTE_KPI_PREDICTION_H

#include "oran-lm.h"

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \brief Proactive conflict management through RSRP KPI prediction (Chapter 6).
 *
 * This LM implements proactive conflict prevention: it predicts future RSRP
 * degradation using an Exponential Moving Average (EMA) applied to the recent
 * per-eNB average RSRP history, and issues preemptive TxPower increase commands
 * before the coverage KPI crosses the reactive CCO threshold.
 *
 * Rationale (from TNSM Chapter 6):
 *   - Reactive conflict management (triage CMM) acts AFTER a KPI violates a threshold.
 *   - Proactive conflict management acts BEFORE the violation occurs, based on
 *     predicted KPI trajectories from observed trends.
 *   - A proactive +TXP command at t prevents an ES-induced RSRP violation at t+Δt,
 *     reducing the time UEs spend below the coverage threshold.
 *
 * Algorithm:
 *   For each serving eNB e:
 *     1. Read all UE RSRP reports for e from the data repository.
 *     2. Compute avg_rsrp(e, t) from current reports.
 *     3. Update EMA: rsrp_ema(e) = alpha * avg_rsrp(e,t) + (1-alpha) * rsrp_ema(e)
 *     4. Predict: rsrp_pred(e) = rsrp_ema(e) + slope * prediction_horizon
 *        where slope is estimated from the last two EMA values.
 *     5. If rsrp_pred(e) < proactive_threshold AND avg_rsrp(e,t) > reactive_threshold:
 *           → Issue OranCommandLte2LteTxPower(+stepDb) for eNB e.
 *           → Log: predicted RSRP, threshold, action.
 *
 * The proactive_threshold is set above the reactive CCO threshold, so this LM
 * acts earlier, preventing the conflict rather than reacting to it.
 *
 * Attributes:
 *   ProactiveThresholdDbm  — RSRP threshold for proactive action (e.g. -65 dBm)
 *   ReactiveThresholdDbm   — Reactive threshold to skip if already above (e.g. -70 dBm)
 *   EmaAlpha               — EMA smoothing factor in (0,1); higher = more reactive
 *   StepSizeDb             — TxPower increase step (dB) per proactive action
 *   PredictionHorizon      — Number of cycles ahead to predict (integer)
 *   MinRsrpSamples         — Minimum RSRP samples per eNB before acting
 */
class OranLmLte2LteKpiPrediction : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmLte2LteKpiPrediction();
    ~OranLmLte2LteKpiPrediction() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    // ── Per-eNB EMA state ────────────────────────────────────────────────────
    struct EmaState
    {
        double   ema        = 0.0;
        double   prevEma    = 0.0;
        bool     initialized = false;
        uint32_t actionCount = 0; //!< Proactive actions issued for this eNB
    };

    std::map<uint64_t, EmaState> m_emaState; // keyed by eNB E2 node ID

    // ── Configuration ─────────────────────────────────────────────────────────
    double   m_proactiveThreshDbm;  //!< Act if predicted RSRP drops below this
    double   m_reactiveThreshDbm;   //!< Skip action if current RSRP already below this
    double   m_emaAlpha;            //!< EMA smoothing factor
    double   m_stepSizeDb;          //!< TxPower increment per proactive action
    uint32_t m_predictionHorizon;   //!< Cycles ahead to predict (integer)
    uint32_t m_minRsrpSamples;      //!< Min UE RSRP samples per eNB to act
};

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_KPI_PREDICTION_H
