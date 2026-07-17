/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORT_LTE_UE_APP_DEMAND
#define ORAN_REPORT_LTE_UE_APP_DEMAND

#include "oran-report.h"

#include <string>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Report with a UE's observed application-layer demand (throughput, Mbps)
 * over the last reporting interval. Feeds bandwidth-capacity-aware xApps
 * (e.g. MLB) that need real per-UE demand rather than raw UE counts.
 */
class OranReportLteUeAppDemand : public OranReport
{
  public:
    static TypeId GetTypeId();
    OranReportLteUeAppDemand();
    ~OranReportLteUeAppDemand() override;

    std::string ToString() const override;

    /**
     * Gets the reported demand.
     *
     * @return Demand in Mbps.
     */
    double GetDemandMbps() const;

  private:
    double m_demandMbps; //!< Observed demand (Mbps) over the last reporting interval.

}; // class OranReportLteUeAppDemand

} // namespace ns3

#endif // ORAN_REPORT_LTE_UE_APP_DEMAND
