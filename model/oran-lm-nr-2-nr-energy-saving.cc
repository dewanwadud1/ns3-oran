/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Energy-Saving Logic Module for NR gNBs. Mirrors OranLmLte2LteEnergySaving.
 */

#include "oran-lm-nr-2-nr-energy-saving.h"

#include "oran-near-rt-ric.h"
#include "oran-data-repository.h"
#include "oran-command-nr-2-nr-tx-power.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"

#include <unordered_map>
#include <cmath>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrEnergySaving");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrEnergySaving);

namespace {
  std::unordered_map<uint64_t, double> g_prevGnbRemainingJ;
  double g_prevTimeSec = std::numeric_limits<double>::quiet_NaN();
}

TypeId
OranLmNr2NrEnergySaving::GetTypeId (void)
{
  static TypeId tid = TypeId("ns3::OranLmNr2NrEnergySaving")
    .SetParent<OranLm>()
    .AddConstructor<OranLmNr2NrEnergySaving>()
    .AddAttribute("TargetPowerW",
                  "Target average power consumption per gNB (W). "
                  "LM increases TxPower when below target and decreases it when above.",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&OranLmNr2NrEnergySaving::m_targetPowerW),
                  MakeDoubleChecker<double>(0.0))
    .AddAttribute("StepSize",
                  "TxPower adjustment step (dB) per LM invocation.",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&OranLmNr2NrEnergySaving::m_stepSize),
                  MakeDoubleChecker<double>(0.0))
    .AddAttribute("LmIntervalSec",
                  "Expected LM query interval (s). Used to convert delta-J to average power (W).",
                  DoubleValue(5.0),
                  MakeDoubleAccessor(&OranLmNr2NrEnergySaving::m_lmIntervalSec),
                  MakeDoubleChecker<double>(1e-3));
  return tid;
}

OranLmNr2NrEnergySaving::OranLmNr2NrEnergySaving ()
  : OranLm ()
{
  NS_LOG_FUNCTION(this);
  m_name = "OranLmNr2NrEnergySaving";
}

OranLmNr2NrEnergySaving::~OranLmNr2NrEnergySaving ()
{
  NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrEnergySaving::Run (void)
{
  NS_LOG_FUNCTION(this);

  std::vector<Ptr<OranCommand>> commands;

  if (!m_active)
  {
    NS_LOG_WARN("Energy-Saving LM inactive; skipping.");
    return commands;
  }
  NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmNr2NrEnergySaving: no Near-RT RIC");

  Ptr<OranDataRepository> repo = m_nearRtRic->Data();
  const double nowSec = Simulator::Now().GetSeconds();

  const bool firstRun = !std::isfinite(g_prevTimeSec);
  if (firstRun)
  {
    g_prevTimeSec = nowSec;
    for (auto gnbId : repo->GetNrGnbE2NodeIds())
    {
      g_prevGnbRemainingJ[gnbId] = repo->GetNrEnergyRemaining(gnbId);
    }
    NS_LOG_INFO("Energy-Saving LM warm-up complete; no commands this tick.");
    return commands;
  }

  double dt = nowSec - g_prevTimeSec;
  g_prevTimeSec = nowSec;
  if (dt <= 0.0) dt = m_lmIntervalSec;

  constexpr double eps = 1e-9;

  for (auto gnbId : repo->GetNrGnbE2NodeIds())
  {
    const double remNow  = repo->GetNrEnergyRemaining(gnbId);
    double& remPrev      = g_prevGnbRemainingJ[gnbId];
    const double deltaJ  = remPrev - remNow;
    remPrev              = remNow;

    if (deltaJ < 0.0)
    {
      NS_LOG_INFO("gNB " << gnbId << ": negative deltaJ (" << deltaJ << " J); skipping.");
      continue;
    }

    const double avgPowerW = deltaJ / dt;

    double deltaDb = 0.0;
    if (avgPowerW > m_targetPowerW + eps)
      deltaDb = -m_stepSize;
    else if (avgPowerW < m_targetPowerW - eps && avgPowerW > eps)
      deltaDb = +m_stepSize;

    if (std::abs(deltaDb) < eps)
    {
      NS_LOG_INFO("gNB " << gnbId
                  << ": avgPower=" << avgPowerW << " W"
                  << " ~ target=" << m_targetPowerW << " W (dead-band); no change.");
      continue;
    }

    Ptr<OranCommandNr2NrTxPower> cmd = CreateObject<OranCommandNr2NrTxPower>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(gnbId));
    cmd->SetAttribute("PowerDeltaDb",   DoubleValue(deltaDb));

    repo->LogCommandLm(m_name, cmd);
    commands.push_back(cmd);

    NS_LOG_INFO("gNB " << gnbId
                << ": deltaJ=" << deltaJ << " J"
                << " dt=" << dt << " s"
                << " avgPower=" << avgPowerW << " W"
                << " target=" << m_targetPowerW << " W"
                << " -> deltaTx=" << deltaDb << " dB");
  }

  return commands;
}

} // namespace ns3
