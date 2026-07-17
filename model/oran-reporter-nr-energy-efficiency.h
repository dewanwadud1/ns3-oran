/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * A Reporter that captures the remaining energy of an NR gNB's energy source.
 */

#ifndef ORAN_REPORTER_NR_ENERGY_EFFICIENCY_H
#define ORAN_REPORTER_NR_ENERGY_EFFICIENCY_H

#include "oran-reporter.h"
#include "oran-report-nr-energy-efficiency.h"

#include "ns3/ptr.h"
#include "ns3/basic-energy-source.h"

#include <vector>

namespace ns3 {

/**
 * \ingroup oran
 * \brief Captures and forwards remaining-energy samples for an NR gNB into the O-RAN pipeline.
 */
class OranReporterNrEnergyEfficiency : public OranReporter
{
public:
  static TypeId GetTypeId ();

  OranReporterNrEnergyEfficiency ();
  ~OranReporterNrEnergyEfficiency () override;

  void SetEnergySource (Ptr<energy::BasicEnergySource> src) { m_energySource = src; }

  void ReportEnergyEfficiency (void);

protected:
  std::vector<Ptr<OranReport>> GenerateReports () override;

private:
  std::vector<Ptr<OranReport>> m_reports;
  Ptr<energy::BasicEnergySource> m_energySource;
};

} // namespace ns3

#endif // ORAN_REPORTER_NR_ENERGY_EFFICIENCY_H
