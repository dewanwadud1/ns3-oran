/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 */

#include "oran-e2-node-terminator-lte-enb.h"

#include "oran-command-lte-2-lte-handover.h"
#include "oran-command-lte-2-lte-tx-power.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/pointer.h>
#include <ns3/string.h>

#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-enb-phy.h>
#include <ns3/lte-enb-rrc.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranE2NodeTerminatorLteEnb");
NS_OBJECT_ENSURE_REGISTERED(OranE2NodeTerminatorLteEnb);

TypeId
OranE2NodeTerminatorLteEnb::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OranE2NodeTerminatorLteEnb")
                        .SetParent<OranE2NodeTerminator>()
                        .AddConstructor<OranE2NodeTerminatorLteEnb>();
  return tid;
}

OranE2NodeTerminatorLteEnb::OranE2NodeTerminatorLteEnb(void)
  : OranE2NodeTerminator()
{
  NS_LOG_FUNCTION(this);
}

OranE2NodeTerminatorLteEnb::~OranE2NodeTerminatorLteEnb(void)
{
  NS_LOG_FUNCTION(this);
}

OranNearRtRic::NodeType
OranE2NodeTerminatorLteEnb::GetNodeType(void) const
{
  NS_LOG_FUNCTION(this);
  return OranNearRtRic::NodeType::LTEENB;
}

/* ---- Safe helpers ---- */
static inline Ptr<LteEnbNetDevice>
GetEnbDevSafe(Ptr<Node> node, uint32_t idx)
{
  if (node == nullptr) return nullptr;
  if (idx >= node->GetNDevices()) return nullptr;
  return node->GetDevice(idx)->GetObject<LteEnbNetDevice>();
}

void
OranE2NodeTerminatorLteEnb::ReceiveCommand(Ptr<OranCommand> command)
{
  NS_LOG_FUNCTION(this << command);

  if (!m_active)
  {
    NS_LOG_DEBUG("Terminator not active; dropping command.");
    return;
  }
  if (command == nullptr)
  {
    NS_LOG_WARN("Null command; dropping.");
    return;
  }

  // We assume the RIC only delivers commands addressed to this E2 node.
  const TypeId t = command->GetInstanceTypeId();

  // ------------------ HO command ------------------
  if (t == OranCommandLte2LteHandover::GetTypeId())
  {
    Ptr<OranCommandLte2LteHandover> ho = DynamicCast<OranCommandLte2LteHandover>(command);
    if (!ho)
    {
      NS_LOG_WARN("TypeId matched HO, but DynamicCast failed; dropping.");
      return;
    }

    Ptr<Node> node = GetNode();
    Ptr<LteEnbNetDevice> dev = GetEnbDevSafe(node, GetNetDeviceIndex());
    if (!dev)
    {
      NS_LOG_WARN("No LteEnbNetDevice @ index " << GetNetDeviceIndex()
                   << " for eNB E2=" << GetE2NodeId() << "; dropping HO.");
      return;
    }

    Ptr<LteEnbRrc> rrc = dev->GetRrc();
    if (!rrc)
    {
      NS_LOG_WARN("No LteEnbRrc on eNB E2=" << GetE2NodeId() << "; dropping HO.");
      return;
    }

    const uint16_t rnti   = ho->GetTargetRnti();
    const uint16_t cellId = ho->GetTargetCellId();

    NS_LOG_INFO("eNB[E2=" << GetE2NodeId()
                << "] HO request: RNTI=" << rnti
                << " → CellId=" << cellId);

    rrc->SendHandoverRequest(rnti, cellId);
    return;
  }

  // ------------------ Tx Power command ------------------
  if (t == OranCommandLte2LteTxPower::GetTypeId())
  {
    Ptr<OranCommandLte2LteTxPower> tx = DynamicCast<OranCommandLte2LteTxPower>(command);
    if (!tx)
    {
      NS_LOG_WARN("TypeId matched TxPower, but DynamicCast failed; dropping.");
      return;
    }

    Ptr<Node> node = GetNode();
    Ptr<LteEnbNetDevice> dev = GetEnbDevSafe(node, GetNetDeviceIndex());
    if (!dev)
    {
      NS_LOG_WARN("No LteEnbNetDevice @ index " << GetNetDeviceIndex()
                   << " for eNB E2=" << GetE2NodeId() << "; dropping TxPower cmd.");
      return;
    }

    Ptr<LteEnbPhy> phy = dev->GetPhy();
    if (!phy)
    {
      NS_LOG_WARN("No LteEnbPhy on eNB E2=" << GetE2NodeId() << "; dropping TxPower cmd.");
      return;
    }

    double deltaDb = tx->GetPowerDeltaDb();
    double curDbm  = phy->GetTxPower();
    double newDbm  = curDbm + deltaDb;

    // optional clamp; adjust if you want broader range
    if (newDbm < 0.0)  newDbm = 0.0;
    if (newDbm > 70.0) newDbm = 70.0;

    phy->SetTxPower(newDbm);

    NS_LOG_INFO("eNB[E2=" << GetE2NodeId()
                << "] applied TxPower: " << curDbm << " dBm → " << newDbm
                << " (Δ=" << deltaDb << " dB)");
    return;
  }

  // ------------------ Unknown command ------------------
  NS_LOG_WARN("Unknown command type " << t.GetName() << "; dropping.");
}

Ptr<LteEnbNetDevice>
OranE2NodeTerminatorLteEnb::GetNetDevice(void) const
{
  NS_LOG_FUNCTION(this);

  Ptr<Node> node = GetNode();
  if (!node)
  {
    NS_LOG_WARN("GetNetDevice(): no Node bound; returning nullptr.");
    return nullptr;
  }
  if (GetNetDeviceIndex() >= node->GetNDevices())
  {
    NS_LOG_WARN("GetNetDevice(): index " << GetNetDeviceIndex()
                 << " out of range (" << node->GetNDevices() << "); returning nullptr.");
    return nullptr;
  }

  Ptr<LteEnbNetDevice> dev = node->GetDevice(GetNetDeviceIndex())->GetObject<LteEnbNetDevice>();
  if (!dev)
  {
    NS_LOG_WARN("GetNetDevice(): device at index is not LteEnbNetDevice; returning nullptr.");
  }
  return dev;
}

} // namespace ns3

