/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * A Reporter that captures the remaining energy of an LTE eNB's energy source.
 * Updated for ns-3.42: energy classes are in ns3::energy:: namespace.
 */

#ifndef ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H
#define ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H

#include "oran-reporter.h"
#include "oran-report-lte-energy-efficiency.h"

#include "ns3/ptr.h"
#include "ns3/basic-energy-source.h"

#include <vector>

namespace ns3 {

/**
 * \ingroup oran
 * \brief Captures and forwards remaining-energy samples for an LTE eNB into the O-RAN pipeline.
 */
class OranReporterLteEnergyEfficiency : public OranReporter
{
public:
  static TypeId GetTypeId ();

  OranReporterLteEnergyEfficiency ();
  ~OranReporterLteEnergyEfficiency () override;

  void SetEnergySource (Ptr<energy::BasicEnergySource> src) { m_energySource = src; }

  void ReportEnergyEfficiency (void);

protected:
  std::vector<Ptr<OranReport>> GenerateReports () override;

private:
  std::vector<Ptr<OranReport>> m_reports;
  Ptr<energy::BasicEnergySource> m_energySource;
};

} // namespace ns3

#endif // ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H
