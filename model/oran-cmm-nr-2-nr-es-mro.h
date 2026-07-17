/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_CMM_NR_2_NR_ES_MRO_H
#define ORAN_CMM_NR_2_NR_ES_MRO_H

#include "oran-cmm.h"

#include <cstdint>

namespace ns3
{

/**
 * \brief NR gNB counterpart of OranCmmLte2LteEsMro. Mirrors its game-theoretic
 * ES/MRO TxPower conflict mitigation logic unchanged.
 */
class OranCmmNr2NrEsMro : public OranCmm
{
  public:
    static TypeId GetTypeId();
    OranCmmNr2NrEsMro();
    ~OranCmmNr2NrEsMro() override;

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

#endif // ORAN_CMM_NR_2_NR_ES_MRO_H
