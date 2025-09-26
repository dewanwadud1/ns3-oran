/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Energy-saving LM for LTE eNBs.
 * Adjusts eNB TxPower to steer energy efficiency toward a target.
 *
 * Fixes over the original:
 *  - Uses delta efficiency (Δbits / ΔJ) between invocations; no hard-coded initial energy.
 *  - Emits at most one TxPower command per eNB per tick (no per-UE amplification).
 *  - First invocation warms up state and sends no commands (avoids division-by-zero / jitters).
 */

#include "oran-lm-lte-2-lte-energy-saving.h"
#include "oran-near-rt-ric.h"
#include "oran-data-repository.h"
#include "oran-e2-node-terminator-lte-enb.h"
#include "oran-command-lte-2-lte-tx-power.h"

#include <ns3/log.h>
#include <ns3/oran-module.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>

#include <unordered_map>
#include <limits>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteEnergySaving");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteEnergySaving);

TypeId
OranLmLte2LteEnergySaving::GetTypeId (void)
{
  static TypeId tid = TypeId("ns3::OranLmLte2LteEnergySaving")
    .SetParent<OranLm>()
    .AddConstructor<OranLmLte2LteEnergySaving>()
    .AddAttribute("TargetEfficiency",
                  "Desired energy efficiency (bits per Joule) measured over the last LM window.",
                  DoubleValue(1e6),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_targetEfficiency),
                  MakeDoubleChecker<double>(0.0))
    .AddAttribute("StepSize",
                  "Transmit-power adjustment step (dB) per LM invocation. Positive raises power, negative lowers.",
                  DoubleValue(1.0),
                  MakeDoubleAccessor(&OranLmLte2LteEnergySaving::m_stepSize),
                  MakeDoubleChecker<double>(0.0));
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

/* ---------- Local LM state (per process) -------------------------
 * We keep prior counters to compute deltas between invocations.
 * Stored here to avoid changing the header/API.
 */
namespace {
  // Total RX bits across all UEs on the last run
  double g_prevTotalBits = std::numeric_limits<double>::quiet_NaN();

  // Per-eNB remaining energy at the last run (J)
  std::unordered_map<uint32_t, double> g_prevEnbRemainingJ;
} // anonymous

std::vector< Ptr<OranCommand> >
OranLmLte2LteEnergySaving::Run (void)
{
  NS_LOG_FUNCTION(this);

  std::vector< Ptr<OranCommand> > commands;

  if (!m_active)
    {
      NS_LOG_WARN("Energy-Saving LM inactive; skipping.");
      return commands;
    }
  if (m_nearRtRic == nullptr)
    {
      NS_LOG_WARN("No Near-RT RIC; skipping.");
      return commands;
    }

  Ptr<OranDataRepository> repo = m_nearRtRic->Data ();

  // 1) Aggregate RX across all UEs (bits)
  double totalBitsNow = 0.0;
  for (auto ueId : repo->GetLteUeE2NodeIds())
    {
      // repo->GetAppRx() returns bytes (as used in your original LM)
      const uint64_t rxBytes = repo->GetAppRx(ueId);
      totalBitsNow += static_cast<double>(rxBytes) * 8.0;
    }

  // 2) Warm-up on first invocation: capture baselines, emit no commands
  const bool firstRun = !std::isfinite(g_prevTotalBits);
  if (firstRun)
    {
      g_prevTotalBits = totalBitsNow;
      for (auto enbId : repo->GetLteEnbE2NodeIds())
        {
          g_prevEnbRemainingJ[enbId] = repo->GetLteEnergyRemaining(enbId);
        }
      NS_LOG_INFO("Energy-Saving LM warm-up; no commands this tick.");
      return commands;
    }

  // 3) Compute Δbits since last run
  double deltaBits = totalBitsNow - g_prevTotalBits;
  g_prevTotalBits  = totalBitsNow;

  // Guard tiny/negative deltas (e.g., clock skew or repo lag)
  if (deltaBits < 0.0) deltaBits = 0.0;

  // Small epsilon to avoid too-aggressive toggling when very close to target
  const double eps = 1e-6;

  // 4) For each eNB, compute ΔJ and decide a single ±StepSize command
  for (auto enbId : repo->GetLteEnbE2NodeIds())
    {
      const double remNow = repo->GetLteEnergyRemaining(enbId);
      double &remPrevRef  = g_prevEnbRemainingJ[enbId]; // creates if missing
      double deltaJ       = remPrevRef - remNow;        // energy consumed in this LM window
      remPrevRef          = remNow;                     // update baseline

      // Guards
      if (deltaJ <= 0.0 || deltaBits <= 0.0)
        {
          NS_LOG_INFO("eNB " << enbId
                      << ": ΔJ=" << deltaJ << " J, Δbits=" << deltaBits
                      << " → no command (insufficient delta).");
          continue;
        }

      const double eff = deltaBits / deltaJ; // bits per Joule in last window

      double deltaDb = 0.0;
      if      (eff < m_targetEfficiency - eps) deltaDb =  m_stepSize; // below target → add power
      else if (eff > m_targetEfficiency + eps) deltaDb = -m_stepSize; // above target → reduce power
      else                                     deltaDb =  0.0;         // within dead-band

      if (std::abs(deltaDb) < eps)
        {
          NS_LOG_INFO("eNB " << enbId << ": eff=" << eff
                        << " ~ target=" << m_targetEfficiency
                        << " (dead-band), no change.");
          continue;
        }

      // Emit exactly one command per eNB
      Ptr<OranCommandLte2LteTxPower> cmd = CreateObject<OranCommandLte2LteTxPower>();
      cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
      cmd->SetAttribute("PowerDeltaDb",   DoubleValue(deltaDb));

      repo->LogCommandLm(m_name, cmd);
      commands.push_back(cmd);

      NS_LOG_INFO("eNB " << enbId
                  << ": Δbits=" << deltaBits
                  << " ΔJ=" << deltaJ
                  << " eff=" << eff
                  << " target=" << m_targetEfficiency
                  << " → cmd ΔTx=" << deltaDb << " dB");
    }

  return commands;
}

} // namespace ns3

