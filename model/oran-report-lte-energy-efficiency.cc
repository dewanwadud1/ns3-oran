/*
 * File: model/oran-report-lte-energy-efficiency.cc
 * This file has been created by A. Wadud from University College Dublin
 */
 
 /* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Implementation of the energy‐efficiency report.
 */

#include "oran-report-lte-energy-efficiency.h"

#include "oran-report.h"
#include <ns3/log.h>
#include <ns3/double.h>
#include <sstream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranReportLteEnergyEfficiency");
NS_OBJECT_ENSURE_REGISTERED(OranReportLteEnergyEfficiency);

TypeId
OranReportLteEnergyEfficiency::GetTypeId()
{
  static TypeId tid = TypeId("ns3::OranReportLteEnergyEfficiency")
      .SetParent<OranReport>()
      .AddConstructor<OranReportLteEnergyEfficiency>()
      .AddAttribute(
          "EnergyRemaining",
          "The remaingin joules",
          DoubleValue(),
          MakeDoubleAccessor(&OranReportLteEnergyEfficiency::m_energyRemaining),
          MakeDoubleChecker<double>());
  return tid;
}

OranReportLteEnergyEfficiency::OranReportLteEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

OranReportLteEnergyEfficiency::~OranReportLteEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

std::string
OranReportLteEnergyEfficiency::ToString() const
{
  NS_LOG_FUNCTION(this);
  std::stringstream ss;
  Time time = GetTime();
  ss << "OranReportLteEnergyEfficiency(";
  ss << "E2NodeId=" << GetReporterE2NodeId();
  ss << ";Time=" << time.As(Time::S);
  ss << ";EnergyRemaining=" << m_energyRemaining;
  ss << ")";
  return ss.str();
}

double
OranReportLteEnergyEfficiency::GetLteEnergyRemaining() const
{
  NS_LOG_FUNCTION(this);
  return m_energyRemaining;
}

} // namespace ns3
