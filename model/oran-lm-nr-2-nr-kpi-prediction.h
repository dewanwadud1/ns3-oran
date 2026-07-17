/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_NR_2_NR_KPI_PREDICTION_H
#define ORAN_LM_NR_2_NR_KPI_PREDICTION_H

#include "oran-lm.h"

#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \brief NR gNB counterpart of OranLmLte2LteKpiPrediction. Mirrors its
 * EMA-based proactive RSRP prediction logic unchanged.
 */
class OranLmNr2NrKpiPrediction : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrKpiPrediction();
    ~OranLmNr2NrKpiPrediction() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    struct EmaState
    {
        double   ema        = 0.0;
        double   prevEma    = 0.0;
        bool     initialized = false;
        uint32_t actionCount = 0;
    };

    std::map<uint64_t, EmaState> m_emaState;

    double   m_proactiveThreshDbm;
    double   m_reactiveThreshDbm;
    double   m_emaAlpha;
    double   m_stepSizeDb;
    uint32_t m_predictionHorizon;
    uint32_t m_minRsrpSamples;
};

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_KPI_PREDICTION_H
