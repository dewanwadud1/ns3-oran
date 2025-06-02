/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * NIST-developed software is provided by NIST as a public service. You may
 * use, copy and distribute copies of the software in any medium, provided that
 * you keep intact this entire notice. You may improve, modify and create
 * derivative works of the software or any portion of the software, and you may
 * copy and distribute such modifications or works. Modified works should carry
 * a notice stating that you changed the software and should note the date and
 * nature of any such change. Please explicitly acknowledge the National
 * Institute of Standards and Technology as the source of the software.
 *
 * NIST-developed software is expressly provided "AS IS." NIST MAKES NO
 * WARRANTY OF ANY KIND, EXPRESS, IMPLIED, IN FACT OR ARISING BY OPERATION OF
 * LAW, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, NON-INFRINGEMENT AND DATA ACCURACY. NIST
 * NEITHER REPRESENTS NOR WARRANTS THAT THE OPERATION OF THE SOFTWARE WILL BE
 * UNINTERRUPTED OR ERROR-FREE, OR THAT ANY DEFECTS WILL BE CORRECTED. NIST
 * DOES NOT WARRANT OR MAKE ANY REPRESENTATIONS REGARDING THE USE OF THE
 * SOFTWARE OR THE RESULTS THEREOF, INCLUDING BUT NOT LIMITED TO THE
 * CORRECTNESS, ACCURACY, RELIABILITY, OR USEFULNESS OF THE SOFTWARE.
 *
 * You are solely responsible for determining the appropriateness of using and
 * distributing the software and you assume all risks associated with its use,
 * including but not limited to the risks and costs of program errors,
 * compliance with applicable laws, damage to or loss of data, programs or
 * equipment, and the unavailability or interruption of operation. This
 * software is not intended to be used in any situation where a failure could
 * cause risk of injury or damage to property. The software developed by NIST
 * employees is not subject to copyright protection within the United States.
 */

#ifndef ORAN_REPORT_APPLOSS
#define ORAN_REPORT_APPLOSS

#include "oran-report.h"

#include <string>

namespace ns3
{

/**
 * \ingroup oran
 *
 * Report with the application packet loss of a node at a given time.
 */
class OranReportAppLoss : public OranReport
{
  public:
    /**
     * Get the TypeId of the OranReportAppLoss class.
     *
     * \return The TypeId.
     */
    static TypeId GetTypeId(void);
    /**
     * Constructor of the OranReportAppLoss class.
     */
    OranReportAppLoss(void);
    /**
     * Destructor of the OranReportAppLoss class.
     */
    ~OranReportAppLoss(void) override;
    /**
     * Get a string representation of this Report
     *
     * \return A string representation of this Report.
     */
    std::string ToString(void) const override;
    /**
     * Gets the reported application packet loss.
     *
     * \return The reported application packet loss.
     */
    double GetLoss(void) const;
    
    /**
     * Gets the number of TX bytes
     * \return The number of TX bytes.
     */
    uint32_t GetTx(void) const;
    /**
     * Gets the number of RX bytes
     * \return The number of RX bytes.
     */
    uint32_t GetRx(void) const;

  private:
    /**
     * The application packet loss.
     */
    double m_loss;
    uint32_t m_tx; //!< Number of TX bytes
    uint32_t m_rx; //!< Number of RX bytes
}; // class OranReportAppLoss

} // namespace ns3

#endif // ORAN_REPORT_APPLOSS
