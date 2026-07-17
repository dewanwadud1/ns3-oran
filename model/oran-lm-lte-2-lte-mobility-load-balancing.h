/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_LTE_2_LTE_MOBILITY_LOAD_BALANCING_H
#define ORAN_LM_LTE_2_LTE_MOBILITY_LOAD_BALANCING_H

#include "oran-lm.h"

namespace ns3
{

class OranLmLte2LteMobilityLoadBalancing : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmLte2LteMobilityLoadBalancing();
    ~OranLmLte2LteMobilityLoadBalancing() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    double m_loadImbalanceThreshold;
    double m_cioStepDb;
    double m_maxAbsCioDb;
    double m_hotCellTttSec;
    bool m_controlTtt;
    double m_enbCapacityMbps; //!< Fixed per-eNB capacity ceiling demand is balanced against.
};

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_MOBILITY_LOAD_BALANCING_H
