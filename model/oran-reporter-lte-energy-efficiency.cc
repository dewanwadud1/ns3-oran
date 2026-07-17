/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 * Implementation of the LTE energy reporter (reports remaining energy).
 * Updated for ns-3.42: energy classes are in ns3::energy:: namespace.
 */

#include "oran-reporter-lte-energy-efficiency.h"
#include "oran-report-lte-energy-efficiency.h"

#include "ns3/log.h"
#include "ns3/abort.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/energy-module.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranReporterLteEnergyEfficiency");
NS_OBJECT_ENSURE_REGISTERED(OranReporterLteEnergyEfficiency);

TypeId
OranReporterLteEnergyEfficiency::GetTypeId()
{
  static TypeId tid = TypeId("ns3::OranReporterLteEnergyEfficiency")
      .SetParent<OranReporter>()
      .AddConstructor<OranReporterLteEnergyEfficiency>();
  return tid;
}

OranReporterLteEnergyEfficiency::OranReporterLteEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

OranReporterLteEnergyEfficiency::~OranReporterLteEnergyEfficiency()
{
  NS_LOG_FUNCTION(this);
}

void
OranReporterLteEnergyEfficiency::ReportEnergyEfficiency(void)
{
  NS_LOG_FUNCTION(this);
  if (!m_active)
    return;

  NS_ABORT_MSG_IF(m_terminator == nullptr, "Reporter has no E2 terminator set");

  double remaining = 0.0;

  if (m_energySource)
  {
    remaining += m_energySource->GetRemainingEnergy();
  }
  else
  {
    // Fallback: sum all BasicEnergySource(s) on the node
    Ptr<Node> node = m_terminator->GetNode();
    NS_ABORT_MSG_IF(node == nullptr, "Terminator has no Node");

    Ptr<energy::EnergySourceContainer> container =
        node->GetObject<energy::EnergySourceContainer>();
    NS_ABORT_MSG_IF(container == nullptr,
                    "Unable to find EnergySourceContainer on node — "
                    "did you install an energy source?");

    for (auto it = container->Begin(); it != container->End(); ++it)
    {
      Ptr<energy::BasicEnergySource> bes =
          (*it)->GetObject<energy::BasicEnergySource>();
      if (bes)
        remaining += bes->GetRemainingEnergy();
    }
  }

  Ptr<OranReportLteEnergyEfficiency> report = CreateObject<OranReportLteEnergyEfficiency>();
  report->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
  report->SetAttribute("Time",             TimeValue(Simulator::Now()));
  report->SetAttribute("EnergyRemaining",  DoubleValue(remaining));

  m_reports.push_back(report);
}

std::vector<Ptr<OranReport>>
OranReporterLteEnergyEfficiency::GenerateReports()
{
  NS_LOG_FUNCTION(this);
  std::vector<Ptr<OranReport>> out;
  ReportEnergyEfficiency();
  if (m_active)
  {
    out = m_reports;
    m_reports.clear();
  }
  return out;
}

} // namespace ns3
