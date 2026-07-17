/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_NR_2_NR_COVERAGE_CAPACITY_OPTIMIZATION_H
#define ORAN_LM_NR_2_NR_COVERAGE_CAPACITY_OPTIMIZATION_H

#include "oran-lm.h"

namespace ns3
{

/**
 * Mirrors OranLmLte2LteCoverageCapacityOptimization for NR gNBs.
 */
class OranLmNr2NrCoverageCapacityOptimization : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrCoverageCapacityOptimization();
    ~OranLmNr2NrCoverageCapacityOptimization() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    double   m_lowRsrpThresholdDbm;
    double   m_lowRsrpFractionThreshold;
    double   m_stepSizeDb;
    uint32_t m_minSamplesPerCell;
    double   m_criticalRsrpThresholdDbm;
    double   m_criticalFractionThreshold;
    double   m_retStepDeg;
};

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_COVERAGE_CAPACITY_OPTIMIZATION_H
