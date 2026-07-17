/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_LTE_2_LTE_COVERAGE_CAPACITY_OPTIMIZATION_H
#define ORAN_LM_LTE_2_LTE_COVERAGE_CAPACITY_OPTIMIZATION_H

#include "oran-lm.h"

namespace ns3
{

class OranLmLte2LteCoverageCapacityOptimization : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmLte2LteCoverageCapacityOptimization();
    ~OranLmLte2LteCoverageCapacityOptimization() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    double   m_lowRsrpThresholdDbm;
    double   m_lowRsrpFractionThreshold;
    double   m_stepSizeDb;
    uint32_t m_minSamplesPerCell;
    // RET: when coverage is critically poor, CCO also adjusts antenna tilt.
    // This creates an Indirect conflict with MRO (CDR/HSR affected, MRO ∉ P2X[RET]).
    double   m_criticalRsrpThresholdDbm;
    double   m_criticalFractionThreshold;
    double   m_retStepDeg;
};

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_COVERAGE_CAPACITY_OPTIMIZATION_H
