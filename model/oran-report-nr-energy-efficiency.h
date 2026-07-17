/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * Report with the NR energy-efficiency KPI at a given time.
 */

#ifndef ORAN_REPORT_NR_ENERGY_EFFICIENCY_H
#define ORAN_REPORT_NR_ENERGY_EFFICIENCY_H

#include "oran-report.h"
#include <string>

namespace ns3 {

/**
 * \brief Carry a single energy-efficiency sample through the E2 interface.
 */
class OranReportNrEnergyEfficiency : public OranReport
{
public:
  static TypeId GetTypeId ();

  OranReportNrEnergyEfficiency ();

  ~OranReportNrEnergyEfficiency () override;

  std::string ToString () const override;

  /**
   * \brief Retrieve the remaining energy value.
   * \return joules remaining
   */
  double GetNrEnergyRemaining () const;

private:
  double m_energyRemaining; //!< bits transmitted per joule
};

} // namespace ns3

#endif /* ORAN_REPORT_NR_ENERGY_EFFICIENCY_H */
