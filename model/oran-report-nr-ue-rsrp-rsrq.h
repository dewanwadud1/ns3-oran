/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORT_NR_UE_RSRP_RSRQ
#define ORAN_REPORT_NR_UE_RSRP_RSRQ

#include "oran-report.h"

#include <string>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Report with an NR UE's per-cell RSRP/RSRQ measurement (NrUePhy's
 * "ReportUeMeasurements" trace source, same units/semantics as the LTE
 * report this mirrors: RSRP dBm, RSRQ dB, per TS 36.214/38.215).
 */
class OranReportNrUeRsrpRsrq : public OranReport
{
  public:
    static TypeId GetTypeId();
    OranReportNrUeRsrpRsrq();
    ~OranReportNrUeRsrpRsrq() override;
    std::string ToString() const override;
    uint16_t GetRnti() const;
    uint16_t GetCellId() const;
    double GetRsrp() const;
    double GetRsrq() const;
    bool GetIsServingCell() const;
    uint16_t GetComponentCarrierId() const;

  private:
    uint16_t m_rnti;
    uint16_t m_cellId;
    double m_rsrp;
    double m_rsrq;
    bool m_isServingCell;
    uint16_t m_componentCarrierId;

}; // class OranReportNrUeRsrpRsrq

} // namespace ns3

#endif // ORAN_REPORT_NR_UE_RSRP_RSRQ
