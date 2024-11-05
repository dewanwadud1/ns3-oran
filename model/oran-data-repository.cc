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

#include "oran-data-repository.h"

#include <ns3/log.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranDataRepository");

NS_OBJECT_ENSURE_REGISTERED(OranDataRepository);

TypeId
OranDataRepository::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::OranDataRepository").SetParent<Object>();

    return tid;
}

OranDataRepository::OranDataRepository(void)
    : Object(),
      m_active(false)
{
    NS_LOG_FUNCTION(this);
}

OranDataRepository::~OranDataRepository(void)
{
    NS_LOG_FUNCTION(this);
}

void
OranDataRepository::Activate(void)
{
    NS_LOG_FUNCTION(this);

    m_active = true;
}

void
OranDataRepository::Deactivate(void)
{
    NS_LOG_FUNCTION(this);

    m_active = false;
}

bool
OranDataRepository::IsActive(void) const
{
    NS_LOG_FUNCTION(this);

    return m_active;
}

// New Methods for RSRP Reporting
void
OranDataRepository::SaveUeRsrp(uint64_t e2NodeId, uint16_t cellId, double rsrp, Time t)
{
    NS_LOG_FUNCTION(this << e2NodeId << cellId << rsrp << t);
    if (m_active)
    {
        // Store the RSRP value for the UE at the specified time
        m_rsrpTable[e2NodeId][t][cellId] = rsrp;
    }
}

std::map<Time, std::map<uint16_t, double>>
OranDataRepository::GetUeRsrp(uint64_t e2NodeId, Time fromTime, Time toTime)
{
    NS_LOG_FUNCTION(this << e2NodeId << fromTime << toTime);
    std::map<Time, std::map<uint16_t, double>> result;

    if (m_active)
    {
        auto ueIt = m_rsrpTable.find(e2NodeId);
        if (ueIt != m_rsrpTable.end())
        {
            for (auto it = ueIt->second.lower_bound(fromTime); it != ueIt->second.end() && it->first <= toTime; ++it)
            {
                result[it->first] = it->second;
            }
        }
    }

    return result;
}

void
OranDataRepository::DoDispose(void)
{
    NS_LOG_FUNCTION(this);

    Object::DoDispose();
}

} // namespace ns3
