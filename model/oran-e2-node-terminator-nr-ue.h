/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_E2_NODE_TERMINATOR_NR_UE_H
#define ORAN_E2_NODE_TERMINATOR_NR_UE_H

#include "oran-e2-node-terminator.h"

#include "ns3/nr-ue-net-device.h"

namespace ns3
{

/**
 * @ingroup oran
 *
 * E2 Node Terminator for NR UEs. This Terminator does not process any Commands.
 */
class OranE2NodeTerminatorNrUe : public OranE2NodeTerminator
{
  public:
    static TypeId GetTypeId();
    OranE2NodeTerminatorNrUe();
    ~OranE2NodeTerminatorNrUe() override;
    OranNearRtRic::NodeType GetNodeType() const override;
    void ReceiveCommand(Ptr<OranCommand> command) override;
    /**
     * Get the NetDevice of the NR UE.
     *
     * @return The net device.
     */
    virtual Ptr<NrUeNetDevice> GetNetDevice() const;
}; // class OranE2NodeTerminatorNrUe

} // namespace ns3

#endif /* ORAN_E2_NODE_TERMINATOR_NR_UE_H */
