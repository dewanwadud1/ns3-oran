#ifndef ORAN_REPORTER_LTE_UE_RSRP_H
#define ORAN_REPORTER_LTE_UE_RSRP_H

#include "oran-report.h"
#include "oran-reporter.h"

#include "ns3/lte-handover-algorithm.h"
#include "ns3/lte-handover-management-sap.h"
#include "ns3/lte-rrc-sap.h"

#include <ns3/ptr.h>

#include <vector>

namespace ns3
{

/**
 * \ingroup oran
 *
 * Reporter that attaches to an LTE UE and captures the RSRP
 * (Reference Signal Received Power) value for the UE.
 */
class OranReporterLteUeRsrp : public OranReporter
{
  public:
    /**
     * Get the TypeId of the OranReporterLteUeRsrp class.
     *
     * \return The TypeId.
     */
    static TypeId GetTypeId(void);

    /**
     * Constructor of the OranReporterLteUeRsrp class.
     */
    OranReporterLteUeRsrp(void);
    /**
     * Destructor of the OranReporterLteUeRsrp class.
     */
    ~OranReporterLteUeRsrp(void) override;

    // inherited from LteHandoverAlgorithm
    //void SetLteHandoverManagementSapUser(LteHandoverManagementSapUser* s) override;
    //LteHandoverManagementSapProvider* GetLteHandoverManagementSapProvider() override;

    /// let the forwarder class access the protected and private members
    //friend class MemberLteHandoverManagementSapProvider<OranReporterLteUeRsrp>;

  protected:
    /**
     * Get the RSRP value of the attached LTE UE and generate an
     * OranReportLteUeRsrp.
     *
     * \return The generated Report.
     */
     
    std::vector<Ptr<NetDevice>> GetUeDevices(void) const;

    std::vector<Ptr<OranReport>> GenerateReports(void) override;
    
    // inherited from LteHandoverAlgorithm as a Handover Management SAP implementation
    //void DoReportUeMeas(uint16_t rnti, LteRrcSap::MeasResults measResults) override;
    
    private:
    /**
     * Determines if a neighbour cell is a valid destination for handover.
     * Currently always return true.
     *
     * \param cellId The cell ID of the neighbour cell.
     * \return True if the cell is a valid destination for handover.
     */
    //bool IsValidNeighbour(uint16_t cellId);

    /// The expected measurement identity for A3 measurements.
    std::vector<uint8_t> m_measIds;

    /**
     * The `Hysteresis` attribute. Handover margin (hysteresis) in dB (rounded to
     * the nearest multiple of 0.5 dB).
     */
    double m_hysteresisDb;
    /**
     * The `TimeToTrigger` attribute. Time during which neighbour cell's RSRP
     * must continuously higher than serving cell's RSRP "
     */
    Time m_timeToTrigger;

    /// Interface to the eNodeB RRC instance.
    LteHandoverManagementSapUser* m_handoverManagementSapUser;
    /// Receive API calls from the eNodeB RRC instance.
    LteHandoverManagementSapProvider* m_handoverManagementSapProvider;
}; // class OranReporterLteUeRsrp

} // namespace ns3

#endif /* ORAN_REPORTER_LTE_UE_RSRP_H */

