/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_NR_2_NR_MOBILITY_LOAD_BALANCING_H
#define ORAN_LM_NR_2_NR_MOBILITY_LOAD_BALANCING_H

#include "oran-lm.h"

namespace ns3
{

/**
 * Mirrors OranLmLte2LteMobilityLoadBalancing for NR gNBs.
 */
class OranLmNr2NrMobilityLoadBalancing : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrMobilityLoadBalancing();
    ~OranLmNr2NrMobilityLoadBalancing() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    double m_loadImbalanceThreshold;
    double m_cioStepDb;
    double m_maxAbsCioDb;
    double m_hotCellTttSec;
    bool m_controlTtt;
    double m_enbCapacityMbps; //!< Fixed per-gNB capacity ceiling demand is balanced against.
};

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_MOBILITY_LOAD_BALANCING_H
