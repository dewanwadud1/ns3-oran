/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Energy-Saving Logic Module for LTE eNBs.
 *
 * Policy: for each eNB, estimate the average power consumption (W) over the
 * last LM window from the change in remaining energy stored in the data
 * repository.  If the measured power exceeds a configurable target, reduce
 * TxPower; if it is well below the target, increase TxPower (up to a limit).
 *
 * First invocation warms up baselines — no commands are issued.
 */

#include "oran-lm-lte-2-lte-energy-saving.h"

#include "oran-near-rt-ric.h"
#include "oran-data-repository.h"
#include "oran-command-lte-2-lte-tx-power.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"

#include <unordered_map>
#include <cmath>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteEnergySaving");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteEnergySaving);

namespace {
  // Per-eNB: remaining energy (J) at the last run
  std::unordered_map<uint64_t, double> g_prevEnbRemainingJ;
  // Simulation time (s) at the last run
  double g_prevTimeSec = std::numeric_limits<double>::quiet_NaN();
}

TypeId
OranLmLte2LteEnergySaving::GetTypeId (void)
{
  static TypeId tid = TypeId("ns3::OranLmLte2LteEnergySaving")
    .SetParent<OranLm>()
    .AddConstructor<OranLmLte2LteEnergySaving>()
    .AddAttribute("TargetPowerW",
                  "Target average power consumption per eNB (W). "
                  "LM increases TxPower when below target and decreases it when above.",
                  DoubleValue(10.0),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_targetPowerW),
                  MakeDoubleChecker<double>(0.0))
    .AddAttribute("StepSize",
                  "TxPower adjustment step (dB) per LM invocation.",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_stepSize),
                  MakeDoubleChecker<double>(0.0))
    .AddAttribute("LmIntervalSec",
                  "Expected LM query interval (s). Used to convert ΔJ → average power (W).",
                  DoubleValue(5.0),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_lmIntervalSec),
                  MakeDoubleChecker<double>(1e-3));
  return tid;
}

OranLmLte2LteEnergySaving::OranLmLte2LteEnergySaving ()
  : OranLm ()
{
  NS_LOG_FUNCTION(this);
  m_name = "OranLmLte2LteEnergySaving";
}

OranLmLte2LteEnergySaving::~OranLmLte2LteEnergySaving ()
{
  NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteEnergySaving::Run (void)
{
  NS_LOG_FUNCTION(this);

  std::vector<Ptr<OranCommand>> commands;

  if (!m_active)
  {
    NS_LOG_WARN("Energy-Saving LM inactive; skipping.");
    return commands;
  }
  NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmLte2LteEnergySaving: no Near-RT RIC");

  Ptr<OranDataRepository> repo = m_nearRtRic->Data();
  const double nowSec = Simulator::Now().GetSeconds();

  // Warm-up: record baselines on first call
  const bool firstRun = !std::isfinite(g_prevTimeSec);
  if (firstRun)
  {
    g_prevTimeSec = nowSec;
    for (auto enbId : repo->GetLteEnbE2NodeIds())
    {
      g_prevEnbRemainingJ[enbId] = repo->GetLteEnergyRemaining(enbId);
    }
    NS_LOG_INFO("Energy-Saving LM warm-up complete; no commands this tick.");
    return commands;
  }

  // Elapsed time since last call (use actual sim clock for accuracy)
  double dt = nowSec - g_prevTimeSec;
  g_prevTimeSec = nowSec;
  if (dt <= 0.0) dt = m_lmIntervalSec; // guard clock stall

  constexpr double eps = 1e-9;

  for (auto enbId : repo->GetLteEnbE2NodeIds())
  {
    const double remNow  = repo->GetLteEnergyRemaining(enbId);
    double& remPrev      = g_prevEnbRemainingJ[enbId]; // creates 0 if missing
    const double deltaJ  = remPrev - remNow;            // energy consumed this window (J)
    remPrev              = remNow;                       // update baseline

    if (deltaJ < 0.0)
    {
      // Energy recharged / model artefact — skip
      NS_LOG_INFO("eNB " << enbId << ": negative ΔJ (" << deltaJ << " J); skipping.");
      continue;
    }

    const double avgPowerW = deltaJ / dt;  // average power over window (W)

    double deltaDb = 0.0;
    if (avgPowerW > m_targetPowerW + eps)
      deltaDb = -m_stepSize;   // over budget → lower power
    else if (avgPowerW < m_targetPowerW - eps && avgPowerW > eps)
      deltaDb = +m_stepSize;   // under budget and active → raise power
    // else within dead-band or idle → no change

    if (std::abs(deltaDb) < eps)
    {
      NS_LOG_INFO("eNB " << enbId
                  << ": avgPower=" << avgPowerW << " W"
                  << " ~ target=" << m_targetPowerW << " W (dead-band); no change.");
      continue;
    }

    Ptr<OranCommandLte2LteTxPower> cmd = CreateObject<OranCommandLte2LteTxPower>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
    cmd->SetAttribute("PowerDeltaDb",   DoubleValue(deltaDb));

    repo->LogCommandLm(m_name, cmd);
    commands.push_back(cmd);

    NS_LOG_INFO("eNB " << enbId
                << ": ΔJ=" << deltaJ << " J"
                << " dt=" << dt << " s"
                << " avgPower=" << avgPowerW << " W"
                << " target=" << m_targetPowerW << " W"
                << " → ΔTx=" << deltaDb << " dB");
  }

  return commands;
}

} // namespace ns3
