/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORTER_NR_UE_CELL_INFO_H
#define ORAN_REPORTER_NR_UE_CELL_INFO_H

#include "oran-report.h"
#include "oran-reporter.h"

#include "ns3/ptr.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Reporter that attaches to an NR UE and captures the NR Cell ID
 * of the gNB the UE is attached to.
 */
class OranReporterNrUeCellInfo : public OranReporter
{
  public:
    static TypeId GetTypeId();
    OranReporterNrUeCellInfo();
    ~OranReporterNrUeCellInfo() override;

  protected:
    std::vector<Ptr<OranReport>> GenerateReports() override;
}; // class OranReporterNrUeCellInfo

} // namespace ns3

#endif /* ORAN_REPORTER_NR_UE_CELL_INFO_H */
