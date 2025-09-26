/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * A Reporter that captures the LTE energy‐efficiency KPI of the node.
 */
 
#ifndef ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H
#define ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H

#include "oran-reporter.h"
#include "oran-report-lte-energy-efficiency.h"

#include <ns3/ptr.h>
#include <ns3/basic-energy-source.h>  

#include <vector>

namespace ns3 {

/**
 * \ingroup oran
 * \brief Captures and forwards energy-related KPI samples for an LTE eNB into the ORAN pipeline.
 */
class OranReporterLteEnergyEfficiency : public OranReporter
{
public:
  static TypeId GetTypeId ();

  OranReporterLteEnergyEfficiency ();
  ~OranReporterLteEnergyEfficiency () override;

  // Typed setter for the energy source (preferred over Attributes)
  void SetEnergySource (Ptr<BasicEnergySource> src) { m_energySource = src; }

  // Called by the trigger to enqueue a sample for reporting
  void ReportEnergyEfficiency (void);

protected:
  std::vector<Ptr<OranReport>> GenerateReports () override;

private:
  std::vector<Ptr<OranReport>> m_reports;
  Ptr<BasicEnergySource>       m_energySource;  // may be nullptr; .cc can fall back to scanning the node
};

} // namespace ns3

#endif // ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H

