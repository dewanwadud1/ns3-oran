/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_CMM_LTE_2_LTE_ES_MRO_H
#define ORAN_CMM_LTE_2_LTE_ES_MRO_H

#include "oran-cmm.h"

#include <cstdint>

namespace ns3
{

/**
 * \brief Conflict mitigation module for the first ES/MRO experiment.
 *
 * This CMM mediates LTE eNB TxPower reduction commands emitted by the ES logic
 * module.  The controlled parameter is the fraction of the requested power
 * reduction that is allowed through:
 *
 *   p = 0.0 -> protect MRO fully, cancel the ES reduction
 *   p = 1.0 -> allow the ES reduction unchanged
 *
 * Game-theoretic modes sweep p in [0, 1] and optimize simple proxy utility
 * curves.  The shape mirrors the MATLAB prototype and can later be replaced by
 * data-driven KPI utility models.
 */
class OranCmmLte2LteEsMro : public OranCmm
{
  public:
    static TypeId GetTypeId();
    OranCmmLte2LteEsMro();
    ~OranCmmLte2LteEsMro() override;

    std::vector<Ptr<OranCommand>> Filter(
        std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
        override;

  private:
    bool ShouldForwardHandover(Ptr<OranCommand> command);
    double SelectFraction(double requestedDeltaDb) const;
    double SolveGame(const std::string& method) const;

    std::string m_method;
    uint32_t m_sweepPoints;
    double m_esPriority;
    double m_mroPriority;
    double m_utilitySigma;
    double m_qosThreshold;
    double m_qacmZeta;
    double m_handoverHoldoffSec;
    std::map<uint64_t, double> m_lastHandoverByUe;
};

} // namespace ns3

#endif // ORAN_CMM_LTE_2_LTE_ES_MRO_H
