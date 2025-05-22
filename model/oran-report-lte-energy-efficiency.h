/*
 * File: model/oran-report-lte-energy-efficiency.h
 * This file has been created by A. Wadud from University College Dublin
 */

 /* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Report with the LTE energy‐efficiency KPI at a given time.
 */

#ifndef ORAN_REPORT_LTE_ENERGY_EFFICIENCY_H
#define ORAN_REPORT_LTE_ENERGY_EFFICIENCY_H

#include "oran-report.h"
#include <string>

namespace ns3 {

/**
 * \brief Carry a single energy‐efficiency sample through the E2 interface.
 */
class OranReportLteEnergyEfficiency : public OranReport
{
public:
  /**
   * \brief Get the TypeId for this class.
   * \return the TypeId.
   */
  static TypeId GetTypeId ();

  /**
   * \brief Default constructor.
   */
  OranReportLteEnergyEfficiency ();

  /**
   * \brief Destructor.
   */
  ~OranReportLteEnergyEfficiency () override;

  /**
   * \brief Print a human‐readable form.
   * \return A string describing the report.
   */
  std::string ToString () const override;

  /**
   * \brief Retrieve the energy‐efficiency value.
   * \return bits‐per‐joule (or whatever units you chose).
   */
  double GetLteEnergyEfficiency () const;

private:
  double m_energyEfficiency; //!< KPI: bits transmitted per joule
};

} // namespace ns3

#endif /* ORAN_REPORT_LTE_ENERGY_EFFICIENCY_H */
