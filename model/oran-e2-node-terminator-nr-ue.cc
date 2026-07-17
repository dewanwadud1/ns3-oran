/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-e2-node-terminator-nr-ue.h"

#include "oran-command-nr-2-nr-handover.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranE2NodeTerminatorNrUe");

NS_OBJECT_ENSURE_REGISTERED(OranE2NodeTerminatorNrUe);

TypeId
OranE2NodeTerminatorNrUe::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranE2NodeTerminatorNrUe")
                            .SetParent<OranE2NodeTerminator>()
                            .AddConstructor<OranE2NodeTerminatorNrUe>();

    return tid;
}

OranE2NodeTerminatorNrUe::OranE2NodeTerminatorNrUe()
    : OranE2NodeTerminator()
{
    NS_LOG_FUNCTION(this);
}

OranE2NodeTerminatorNrUe::~OranE2NodeTerminatorNrUe()
{
    NS_LOG_FUNCTION(this);
}

OranNearRtRic::NodeType
OranE2NodeTerminatorNrUe::GetNodeType() const
{
    NS_LOG_FUNCTION(this);

    return OranNearRtRic::NRUE;
}

void
OranE2NodeTerminatorNrUe::ReceiveCommand(Ptr<OranCommand> command)
{
    NS_LOG_FUNCTION(this << command);

    if (m_active)
    {
        // No supported commands yet.
    }
}

Ptr<NrUeNetDevice>
OranE2NodeTerminatorNrUe::GetNetDevice() const
{
    NS_LOG_FUNCTION(this);

    Ptr<NrUeNetDevice> nrUeNetDev =
        GetNode()->GetDevice(GetNetDeviceIndex())->GetObject<NrUeNetDevice>();

    NS_ABORT_MSG_IF(nrUeNetDev == nullptr, "Unable to find appropriate network device");

    return nrUeNetDev;
}

} // namespace ns3
