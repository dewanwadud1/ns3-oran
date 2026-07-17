/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-e2-node-terminator-nr-gnb.h"

#include "oran-command-nr-2-nr-cell-parameter.h"
#include "oran-command-nr-2-nr-handover.h"
#include "oran-command-nr-2-nr-tx-power.h"
#include "oran-nr-cell-control-state.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-gnb-phy.h"
#include "ns3/nr-gnb-rrc.h"
#include "ns3/node.h"
#include "ns3/pointer.h"
#include "ns3/string.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranE2NodeTerminatorNrGnb");

NS_OBJECT_ENSURE_REGISTERED(OranE2NodeTerminatorNrGnb);

TypeId
OranE2NodeTerminatorNrGnb::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranE2NodeTerminatorNrGnb")
                            .SetParent<OranE2NodeTerminator>()
                            .AddConstructor<OranE2NodeTerminatorNrGnb>();

    return tid;
}

OranE2NodeTerminatorNrGnb::OranE2NodeTerminatorNrGnb()
    : OranE2NodeTerminator()
{
    NS_LOG_FUNCTION(this);
}

OranE2NodeTerminatorNrGnb::~OranE2NodeTerminatorNrGnb()
{
    NS_LOG_FUNCTION(this);
}

OranNearRtRic::NodeType
OranE2NodeTerminatorNrGnb::GetNodeType() const
{
    NS_LOG_FUNCTION(this);

    return OranNearRtRic::NodeType::NRGNB;
}

void
OranE2NodeTerminatorNrGnb::ReceiveCommand(Ptr<OranCommand> command)
{
    NS_LOG_FUNCTION(this << command);

    if (m_active)
    {
        if (command->GetInstanceTypeId() == OranCommandNr2NrHandover::GetTypeId())
        {
            Ptr<OranCommandNr2NrHandover> handoverCommand =
                command->GetObject<OranCommandNr2NrHandover>();
            Ptr<NrGnbRrc> nrGnbRrc = GetNetDevice()->GetRrc();
            nrGnbRrc->SendHandoverRequest(handoverCommand->GetTargetRnti(),
                                          handoverCommand->GetTargetCellId());
        }
        else if (command->GetInstanceTypeId() == OranCommandNr2NrTxPower::GetTypeId())
        {
            Ptr<OranCommandNr2NrTxPower> txCmd =
                command->GetObject<OranCommandNr2NrTxPower>();
            // NR gNBs may have multiple bandwidth parts (BWPs), each with
            // its own PHY/TxPower; this scenario config uses a single band
            // (BWP index 0), so that is the only PHY that needs updating.
            Ptr<NrGnbPhy> phy = GetNetDevice()->GetPhy(0);
            NS_ABORT_MSG_IF(phy == nullptr, "No NrGnbPhy on gNB; dropping TxPower cmd");

            double deltaDb = txCmd->GetPowerDeltaDb();
            double curDbm  = phy->GetTxPower();
            double newDbm  = curDbm + deltaDb;
            // NR small/macro-cell gNB TxPower is a much narrower range than
            // LTE macro eNBs (0-70 dBm there); clamp to a generic sub-6GHz
            // gNB range instead.
            if (newDbm < 0.0) newDbm = 0.0;
            if (newDbm > 50.0) newDbm = 50.0;
            phy->SetTxPower(newDbm);

            NS_LOG_INFO("gNB[E2=" << GetE2NodeId()
                        << "] TxPower: " << curDbm << " dBm -> " << newDbm
                        << " dBm (delta=" << deltaDb << " dB)");
        }
        else if (command->GetInstanceTypeId() == OranCommandNr2NrCellParameter::GetTypeId())
        {
            Ptr<OranCommandNr2NrCellParameter> paramCmd =
                command->GetObject<OranCommandNr2NrCellParameter>();
            const bool applied = ApplyNrCellControlParameter(GetE2NodeId(),
                                                             paramCmd->GetParameterName(),
                                                             paramCmd->GetValue(),
                                                             paramCmd->IsDelta());
            if (applied)
            {
                NS_LOG_INFO("gNB[E2=" << GetE2NodeId() << "] cell-control "
                                      << paramCmd->GetParameterName() << "="
                                      << paramCmd->GetValue()
                                      << " isDelta=" << paramCmd->IsDelta());
            }
            else
            {
                NS_LOG_WARN("gNB[E2=" << GetE2NodeId()
                                      << "] unknown cell-control parameter "
                                      << paramCmd->GetParameterName());
            }
        }
    }
}

Ptr<NrGnbNetDevice>
OranE2NodeTerminatorNrGnb::GetNetDevice() const
{
    NS_LOG_FUNCTION(this);

    Ptr<NrGnbNetDevice> nrGnbNetDev =
        GetNode()->GetDevice(GetNetDeviceIndex())->GetObject<NrGnbNetDevice>();

    NS_ABORT_MSG_IF(nrGnbNetDev == nullptr, "Unable to find appropriate network device");

    return nrGnbNetDev;
}

} // namespace ns3
