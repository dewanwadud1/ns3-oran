/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
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
};

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_MOBILITY_LOAD_BALANCING_H
