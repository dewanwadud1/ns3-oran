/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Implementation of the LTE energy‐efficiency reporter.
 */

 #include "oran-reporter-lte-energy-efficiency.h"
 #include "oran-report-lte-energy-efficiency.h"
 
 #include <ns3/abort.h>
 #include <ns3/double.h>
 #include <ns3/log.h>
 #include <ns3/simulator.h>
 #include <ns3/uinteger.h>
 #include <ns3/basic-energy-source.h>
 
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
     {
       return;
     }
     
   NS_ABORT_MSG_IF(m_terminator == nullptr,
                   "Reporter has no E2 terminator set");
                   
   Ptr<BasicEnergySource> basicSource = nullptr;
   Ptr<EnergySourceContainer> container = nullptr;

   container = m_terminator->GetNode()->GetObject<EnergySourceContainer>();

   NS_ABORT_MSG_IF(container == nullptr, "Unable to find appropriate energy container");

   double remaining = 0.0;
   for (auto i = container->Begin(); i != container->End(); i++)
   {
      basicSource = (*i)->GetObject<BasicEnergySource>();

      NS_ABORT_MSG_IF(basicSource == nullptr, "Unable to find appropriate energy source");

      remaining += basicSource->GetRemainingEnergy();
   }

 
   // Build the report object with the correct attributes
   Ptr<OranReportLteEnergyEfficiency> report =
     CreateObject<OranReportLteEnergyEfficiency>();
   report->SetAttribute("ReporterE2NodeId",
                        UintegerValue(m_terminator->GetE2NodeId()));
   report->SetAttribute("Time",
                        TimeValue(Simulator::Now()));
   report->SetAttribute("EnergyRemaining",
                        DoubleValue(remaining));
 
   m_reports.push_back(report);
 }
 
 std::vector<Ptr<OranReport>>
 OranReporterLteEnergyEfficiency::GenerateReports()
 {
   NS_LOG_FUNCTION(this);
   std::vector<Ptr<OranReport>> reports;
   ReportEnergyEfficiency();
   if (m_active)
     {
       reports = m_reports;
       m_reports.clear();
     }
   return reports;
 }
 
 } // namespace ns3
 
