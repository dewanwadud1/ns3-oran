#include "oran-reporter-lte-ue-rsrp.h"
#include "oran-report.h"

#include <ns3/log.h>
#include <ns3/uinteger.h>
#include <ns3/double.h>  // Include for DoubleValue
#include <ns3/lte-ue-net-device.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-rrc-sap.h>
#include <ns3/simulator.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterLteUeRsrp");

NS_OBJECT_ENSURE_REGISTERED(OranReporterLteUeRsrp);

TypeId
OranReporterLteUeRsrp::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranReporterLteUeRsrp")
            .SetParent<OranReporter>()
            .AddConstructor<OranReporterLteUeRsrp>();

    return tid;
}

OranReporterLteUeRsrp::OranReporterLteUeRsrp(void)
    : OranReporter()
{
    NS_LOG_FUNCTION(this);
}

OranReporterLteUeRsrp::~OranReporterLteUeRsrp(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranReport>>
OranReporterLteUeRsrp::GenerateReports(void)
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranReport>> reports;

    if (m_active)
    {
        // Ensure the E2 Terminator is valid
        NS_ABORT_MSG_IF(m_terminator == nullptr,
                        "Attempting to generate reports in reporter with NULL E2 Terminator");

        Ptr<LteUeNetDevice> lteUeNetDev = nullptr;
        Ptr<Node> node = m_terminator->GetNode();

        // Fetch the correct LTE UE NetDevice from the node
        for (uint32_t idx = 0; lteUeNetDev == nullptr && idx < node->GetNDevices(); idx++)
        {
            lteUeNetDev = node->GetDevice(idx)->GetObject<LteUeNetDevice>();
        }

        NS_ABORT_MSG_IF(lteUeNetDev == nullptr, "Unable to find appropriate network device");

        // Get the UE RRC to fetch measurement results
        Ptr<LteUeRrc> lteUeRrc = lteUeNetDev->GetRrc();

        // Create and populate measurement parameters
        LteUeCphySapUser::UeMeasurementsParameters params;
        // Populate params with real measurements (e.g., RSRP, cell ID, etc.)
        // Example: params.m_componentCarrierId = 0; (adjust as necessary)

        // Trigger the reporting of UE measurements
        lteUeRrc->DoReportUeMeasurements(params);
        
        // You could also manually call SaveUeMeasurements if needed
        LteRrcSap::ReportConfigEutra reportConfig;
    	reportConfig.eventId = LteRrcSap::ReportConfigEutra::EVENT_A3;
    	reportConfig.a3Offset = 0;
    	//reportConfig.hysteresis = hysteresisIeValue;
    	reportConfig.timeToTrigger = m_timeToTrigger.GetMilliSeconds();
    	reportConfig.reportOnLeave = false;
    	reportConfig.triggerQuantity = LteRrcSap::ReportConfigEutra::RSRP;
    	reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS1024;
    	m_measIds = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfig);
    	
    	LteRrcSap::MeasResults measResults;
        
        
        
        auto it = measResults.measResultListEutra.begin();
        uint16_t cellId = lteUeRrc->GetCellId();  // Get the serving cell ID
        double rsrp = 0.0;  // Replace with actual RSRP value if needed
        double rsrq = 0.0;  // Replace with actual RSRQ value if needed
        bool isConnected = true;  // Adjust based on the context
        uint8_t measId = 0;  // Measurement ID (adjust if necessary)
        
        
        if (measResults.haveMeasResultNeighCells && !measResults.measResultListEutra.empty())
        {
        	//uint16_t bestNeighbourCellId = 0;
        	//uint8_t bestNeighbourRsrp = 0;

        	for (auto it = measResults.measResultListEutra.begin();
        	it != measResults.measResultListEutra.end();
             	++it)
        	{
            		if (it->haveRsrpResult)
            		{
                		cellId = lteUeRrc->GetCellId();
        			rsrp = it->rsrpResult;
        			rsrq = it->rsrqResult;
        			isConnected = true;
        			measId = (uint16_t)measResults.measId;
            		}
            		else
            		{
                		NS_LOG_WARN("RSRP measurement is missing from cell ID " << it->physCellId);
            		}
        	}

        	/*
        	if (bestNeighbourCellId > 0)
        	{
            		NS_LOG_LOGIC("Trigger Handover to cellId " << bestNeighbourCellId);
            		NS_LOG_LOGIC("target cell RSRP " << (uint16_t)bestNeighbourRsrp);
            		NS_LOG_LOGIC("serving cell RSRP " << (uint16_t)measResults.measResultPCell.rsrpResult);

            		// Inform eNodeB RRC about handover
            		m_handoverManagementSapUser->TriggerHandover(rnti, bestNeighbourCellId);
        	} */
    	}


        lteUeRrc->SaveUeMeasurements(cellId, rsrp, rsrq, isConnected, measId);

        // Now you can create a report with the saved RSRP value
        Ptr<OranReport> rsrpReport = CreateObject<OranReport>();
        
        //NS_LOG_INFO("UE E2 Node ID: " << m_terminator->GetE2NodeId() << ", RSRP: " << rsrp);
        
        // Print the RSRP to the command prompt before passing it to the report
        NS_LOG_INFO("RSRP recorded: " << rsrp << " dBm");

        // Additional details to print for debugging
        NS_LOG_INFO("UE E2 Node ID: " << m_terminator->GetE2NodeId());
        NS_LOG_INFO("Time: " << Simulator::Now().GetSeconds() << " seconds");

        // Set attributes in the report for RSRP and other data
        rsrpReport->SetAttribute("Rsrp", DoubleValue(rsrp));
        rsrpReport->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
        rsrpReport->SetAttribute("Time", TimeValue(Simulator::Now()));

        // Add the report to the list of reports
        reports.push_back(rsrpReport);
    }

    return reports;
}

}  // namespace ns3

