/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORT_NR_UE_CELL_INFO_H
#define ORAN_REPORT_NR_UE_CELL_INFO_H

#include "oran-report.h"

#include <string>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Report with the Cell ID of the serving NR gNB for an NR UE.
 */
class OranReportNrUeCellInfo : public OranReport
{
  public:
    static TypeId GetTypeId();
    OranReportNrUeCellInfo();
    ~OranReportNrUeCellInfo() override;
    std::string ToString() const override;

  private:
    uint16_t m_cellId;
    uint16_t m_rnti;

  public:
    uint16_t GetCellId() const;
    uint16_t GetRnti() const;
}; // class OranReportNrUeCellInfo

} // namespace ns3

#endif /* ORAN_REPORT_NR_UE_CELL_INFO_H */
