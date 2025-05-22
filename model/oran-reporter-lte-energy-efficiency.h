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
 #include <vector>
 
 namespace ns3 {
 
 /**
  * \brief Captures and forwards energy‐efficiency samples from the VTM layer
  *        into the ORAN reporting pipeline.
  */
 class OranReporterLteEnergyEfficiency : public OranReporter
 {
 public:
   /**
    * \brief Get the TypeId of the OranReporterLteEnergyEfficiency class.
    * \return The TypeId.
    */
   static TypeId GetTypeId ();
 
   /**
    * \brief Constructor.
    */
   OranReporterLteEnergyEfficiency ();
 
   /**
    * \brief Destructor.
    */
   ~OranReporterLteEnergyEfficiency () override;
 
   /**
    * \brief Enqueue an energy‐efficiency measurement for reporting.
    * \param efficiency Energy‐efficiency KPI (e.g., bits per joule).
    */
   void ReportEnergyEfficiency (double efficiency);
 
 protected:
   /**
    * \brief Called by the framework to retrieve pending reports.
    * \return Vector of OranReport objects to be sent upstream.
    */
   std::vector<Ptr<OranReport>> GenerateReports () override;
 
 private:
   std::vector<Ptr<OranReport>> m_reports; //!< Accumulated reports awaiting dispatch
 };
 
 } // namespace ns3
 
 #endif // ORAN_REPORTER_LTE_ENERGY_EFFICIENCY_H
 