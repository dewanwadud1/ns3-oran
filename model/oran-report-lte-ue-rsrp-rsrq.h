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

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_REPORT_LTE_UE_RSRP_RSRQ
#define ORAN_REPORT_LTE_UE_RSRP_RSRQ

#include "oran-report.h"
#include <string>

namespace ns3 {

/**
 * \ingroup oran
 * UE RSRP/RSRQ report.
 */
class OranReportLteUeRsrpRsrq : public OranReport
{
public:
  static TypeId GetTypeId();

  OranReportLteUeRsrpRsrq();
  ~OranReportLteUeRsrpRsrq() override;

  std::string ToString() const override;

  // Getters
  uint16_t GetRnti() const;
  uint16_t GetCellId() const;
  double   GetRsrp()  const;
  double   GetRsrq()  const;   // <-- returns m_rsrq (not m_rsrp)
  bool     GetIsServingCell() const;
  uint16_t GetComponentCarrierId() const;

  // Typed setters (use these instead of SetAttribute)
  void SetRnti(uint16_t v)               { m_rnti = v; }
  void SetCellId(uint16_t v)             { m_cellId = v; }
  void SetRsrp(double v)                 { m_rsrp = v; }
  void SetRsrq(double v)                 { m_rsrq = v; }
  void SetIsServingCell(bool v)          { m_isServingCell = v; }
  void SetComponentCarrierId(uint16_t v) { m_componentCarrierId = v; }

private:
  uint16_t m_rnti{0};
  uint16_t m_cellId{0};
  double   m_rsrp{0.0};
  double   m_rsrq{0.0};
  bool     m_isServingCell{false};
  uint16_t m_componentCarrierId{0};
};

} // namespace ns3

#endif // ORAN_REPORT_LTE_UE_RSRP_RSRQ
