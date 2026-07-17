/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_E2_NODE_TERMINATOR_NR_GNB_H
#define ORAN_E2_NODE_TERMINATOR_NR_GNB_H

#include "oran-e2-node-terminator.h"

#include "ns3/nr-gnb-net-device.h"

namespace ns3
{

/**
 * @ingroup oran
 *
 * E2 Node Terminator for NR gNBs. This Terminator can process NR Handover,
 * TxPower, and cell-control-parameter Commands (mirrors
 * OranE2NodeTerminatorLteEnb).
 */
class OranE2NodeTerminatorNrGnb : public OranE2NodeTerminator
{
  public:
    static TypeId GetTypeId();
    OranE2NodeTerminatorNrGnb();
    ~OranE2NodeTerminatorNrGnb() override;
    OranNearRtRic::NodeType GetNodeType() const override;
    void ReceiveCommand(Ptr<OranCommand> command) override;
    /**
     * Get the NetDevice of the NR gNB.
     *
     * @return The net device.
     */
    virtual Ptr<NrGnbNetDevice> GetNetDevice() const;
}; // class OranE2NodeTerminatorNrGnb

} // namespace ns3

#endif /* ORAN_E2_NODE_TERMINATOR_NR_GNB_H */
