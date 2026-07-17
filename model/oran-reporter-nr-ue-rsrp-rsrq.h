/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORTER_NR_UE_RSRP_RSRQ
#define ORAN_REPORTER_NR_UE_RSRP_RSRQ

#include "oran-report.h"
#include "oran-reporter.h"

#include "ns3/ptr.h"

#include <vector>

namespace ns3
{

class Packet;
class Address;

/**
 * @ingroup oran
 *
 * A Reporter that captures the NR UE RSRP and RSRQ of the node (fed from
 * NrUePhy's "ReportUeMeasurements" trace source).
 */
class OranReporterNrUeRsrpRsrq : public OranReporter
{
  public:
    static TypeId GetTypeId();
    OranReporterNrUeRsrpRsrq();
    ~OranReporterNrUeRsrpRsrq() override;
    /**
     * Reports the RSRP and RSRQ for an NR UE.
     *
     * @param rnti The RNTI of the UE.
     * @param cellId The cell ID.
     * @param rsrp The RSRP.
     * @param rsrq The RSRQ.
     * @param isServingCell A flag that indicates if this is the serving cell.
     * @param componentCarrierId The component carrier ID.
     */
    void ReportRsrpRsrq(uint16_t rnti,
                        uint16_t cellId,
                        double rsrp,
                        double rsrq,
                        bool isServingCell,
                        uint8_t componentCarrierId);

  protected:
    std::vector<Ptr<OranReport>> GenerateReports() override;

  private:
    std::vector<Ptr<OranReport>> m_reports;
};

} // namespace ns3

#endif // ORAN_REPORTER_NR_UE_RSRP_RSRQ
