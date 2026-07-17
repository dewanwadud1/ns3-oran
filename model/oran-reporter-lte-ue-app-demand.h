/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORTER_LTE_UE_APP_DEMAND
#define ORAN_REPORTER_LTE_UE_APP_DEMAND

#include "oran-report.h"
#include "oran-reporter.h"

#include "ns3/ptr.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup oran
 *
 * A Reporter that captures a UE's observed application-layer demand
 * (throughput, Mbps) over the last reporting interval, for
 * bandwidth-capacity-aware xApps (e.g. MLB) to act on.
 */
class OranReporterLteUeAppDemand : public OranReporter
{
  public:
    static TypeId GetTypeId();
    OranReporterLteUeAppDemand();
    ~OranReporterLteUeAppDemand() override;

    /**
     * Reports the observed demand for this UE.
     *
     * @param demandMbps Observed demand (Mbps) over the last reporting interval.
     */
    void ReportDemand(double demandMbps);

  protected:
    std::vector<Ptr<OranReport>> GenerateReports() override;

  private:
    std::vector<Ptr<OranReport>> m_reports;
};

} // namespace ns3

#endif // ORAN_REPORTER_LTE_UE_APP_DEMAND
