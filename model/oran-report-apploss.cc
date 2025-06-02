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

#include "oran-report-apploss.h"

#include "oran-report.h"

#include <ns3/abort.h>
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/uinteger.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportAppLoss");
NS_OBJECT_ENSURE_REGISTERED(OranReportAppLoss);

TypeId
OranReportAppLoss::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::OranReportAppLoss")
                            .SetParent<OranReport>()
                            .AddConstructor<OranReportAppLoss>()
                            .AddAttribute("Loss",
                                          "App Loss Rate",
                                          DoubleValue(),
                              MakeDoubleAccessor(&OranReportAppLoss::m_loss),
                                          MakeDoubleChecker<double>())
                             .AddAttribute("Tx",
                                          "App TX bytes",
                                          UintegerValue(),
                                                               MakeUintegerAccessor(&OranReportAppLoss::m_tx),
                                          MakeUintegerChecker<uint32_t>())
                             .AddAttribute("Rx",
                                          "App RX bytes",
                                          UintegerValue(),
                              MakeUintegerAccessor(&OranReportAppLoss::m_rx),
                                          MakeUintegerChecker<uint32_t>())
                            ;

    return tid;
}

OranReportAppLoss::OranReportAppLoss(void)
{
    NS_LOG_FUNCTION(this);
}

OranReportAppLoss::~OranReportAppLoss(void)
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportAppLoss::ToString(void) const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportAppLoss("
       << "E2NodeId=" << GetReporterE2NodeId()
       << ";Time=" << time.As(Time::S)
       << ";Loss=" << m_loss
       << ";Tx=" << m_tx
       << ";Rx=" << m_rx
       << ")";

    return ss.str();
}

double
OranReportAppLoss::GetLoss(void) const
{
    NS_LOG_FUNCTION(this);

    return m_loss;
}

uint32_t
OranReportAppLoss::GetTx(void) const
{
  NS_LOG_FUNCTION(this);

  return m_tx;
}

uint32_t
OranReportAppLoss::GetRx(void) const
{
  NS_LOG_FUNCTION(this);

  return m_rx;
}
} // namespace ns3
