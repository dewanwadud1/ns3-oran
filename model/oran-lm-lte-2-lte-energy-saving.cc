/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Implementation of the LTE UE energy‐saving logic module,
 * which directly tweaks eNB TxPower to steer energy‐efficiency.
 */

#include "oran-lm-lte-2-lte-energy-saving.h"
#include "oran-near-rt-ric.h"
#include "oran-data-repository.h"
#include "oran-e2-node-terminator-lte-enb.h"
#include "oran-command-lte-2-lte-tx-power.h"

#include <ns3/log.h>
#include <ns3/oran-module.h>
#include <ns3/simulator.h>
#include <ns3/double.h>           // for DoubleValue, MakeDoubleAccessor, MakeDoubleChecker
#include <ns3/uinteger.h>         // for UintegerValue
#include <ns3/lte-enb-net-device.h>// for LteEnbNetDevice
#include <ns3/lte-enb-phy.h>       // for LteEnbPhy

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteEnergySaving");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteEnergySaving);

TypeId
OranLmLte2LteEnergySaving::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OranLmLte2LteEnergySaving")
    .SetParent<OranLm>()
    .AddConstructor<OranLmLte2LteEnergySaving>()
    .AddAttribute("TargetEfficiency",
                  "Desired energy‐efficiency (bits per joule)",
                  DoubleValue(1e6),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_targetEfficiency),
                  MakeDoubleChecker<double>())
    .AddAttribute("StepSize",
                  "Transmit power adjustment step (dB)",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_stepSize),
                  MakeDoubleChecker<double>());
  return tid;
}

OranLmLte2LteEnergySaving::OranLmLte2LteEnergySaving()
  : OranLm()
{
  NS_LOG_FUNCTION(this);
  m_name = "OranLmLte2LteEnergySaving";
}

OranLmLte2LteEnergySaving::~OranLmLte2LteEnergySaving()
{
  NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteEnergySaving::Run(void)
{
  NS_LOG_FUNCTION(this);
  std::vector<Ptr<OranCommand>> commands; // we won't actually enqueue any OranCommands

  NS_LOG_INFO("Energy‐Saving LM starting; target=" << m_targetEfficiency
               << " step=" << m_stepSize);

  if (!m_active)
  {
    NS_LOG_WARN("  → inactive, skipping");
    return commands;
  }
  if (m_nearRtRic == nullptr)
  {
    NS_LOG_WARN("  → no RIC, skipping");
    return commands;
  }

  Ptr<OranDataRepository> repo = m_nearRtRic->Data();

  // For each UE, compute a delta and apply to all eNBs
  for (auto ueId : repo->GetLteUeE2NodeIds())
  {
    double eff   = repo->GetLteEnergyEfficiency(ueId);
    double delta = 0;
    if      (eff < m_targetEfficiency)
    {
      delta =  m_stepSize;
      NS_LOG_INFO("  UE " << ueId << ": eff=" << eff << " < target, ↑" << delta);
    }
    else if (eff > m_targetEfficiency)
    {
      delta = -m_stepSize;
      NS_LOG_INFO("  UE " << ueId << ": eff=" << eff << " > target, ↓" << -delta);
    }
    else
    {
      NS_LOG_INFO("  UE " << ueId << ": eff=" << eff << " == target, no change");
      continue;
    }

    // Instead, emit one TxPower command per eNB:
    for (auto enbId : repo->GetLteEnbE2NodeIds())
    {
      Ptr<OranCommandLte2LteTxPower> cmd =
      CreateObject<OranCommandLte2LteTxPower>();
      cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
      cmd->SetAttribute("PowerDeltaDb",   DoubleValue(delta));

      // Log & queue it
      repo->LogCommandLm(m_name, cmd);
      commands.push_back(cmd);
    }
  }

  return commands;
}

} // namespace ns3

