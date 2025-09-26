/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * RSRP-driven LTE→LTE handover LM with robust guards.
 *
 * Fixes:
 *  - Map LTE cellId → eNB E2NodeId and verify presence using the Data Repository (no RIC lookup).
 *  - Guard against missing serving eNB mapping (no uninitialized ids).
 *  - Skip if RSRP measurements are absent or non-finite.
 *  - Add per-UE handover hold-off and small hysteresis to avoid ping-pong.
 */

#include "oran-lm-lte-2-lte-rsrp-handover.h"

#include "oran-command-lte-2-lte-handover.h"
#include "oran-data-repository.h"
// No need to include RIC/E2 terminator headers for this LM
// #include "oran-near-rt-ric.h"
// #include "oran-e2-node-terminator.h"
// #include "oran-e2-node-terminator-lte-enb.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteRsrpHandover");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteRsrpHandover);

/* ---------- Simple per-UE debounce (no header change) ---------- */
namespace {
  constexpr double kHoHoldoffSec = 1.5;   // seconds between HOs per UE
  constexpr double kRsrpHystDb   = 1.0;   // require +1 dB better to trigger
  std::unordered_map<uint64_t,double> g_lastHoTime; // ueE2Id -> last HO sim time (s)
}

TypeId
OranLmLte2LteRsrpHandover::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OranLmLte2LteRsrpHandover")
                        .SetParent<OranLm>()
                        .AddConstructor<OranLmLte2LteRsrpHandover>();
  return tid;
}

OranLmLte2LteRsrpHandover::OranLmLte2LteRsrpHandover(void)
  : OranLm()
{
  m_name = "OranLmLte2LteRsrpHandover";
}

OranLmLte2LteRsrpHandover::~OranLmLte2LteRsrpHandover(void) = default;

std::vector<Ptr<OranCommand>>
OranLmLte2LteRsrpHandover::Run(void)
{
  std::vector<Ptr<OranCommand>> commands;

  if (!m_active)
  {
    NS_LOG_INFO("RSRP LM inactive; skipping.");
    return commands;
  }

  NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                  "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

  Ptr<OranDataRepository> data = m_nearRtRic->Data();

  auto ueInfos  = GetUeInfos(data);
  auto enbInfos = GetEnbInfos(data);

  commands = GetHandoverCommands(data, ueInfos, enbInfos);
  return commands;
}

std::vector<OranLmLte2LteRsrpHandover::UeInfo>
OranLmLte2LteRsrpHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
  std::vector<UeInfo> ueInfos;
  for (auto ueId : data->GetLteUeE2NodeIds())
  {
    UeInfo ueInfo;
    ueInfo.nodeId = ueId;

    bool found;
    std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetLteUeCellInfo(ueInfo.nodeId);
    if (!found)
    {
      NS_LOG_INFO("No UE cell info for E2 UE " << ueInfo.nodeId);
      continue;
    }

    auto nodePositions = data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());
    if (nodePositions.empty())
    {
      NS_LOG_INFO("No UE position for E2 UE " << ueInfo.nodeId);
      continue;
    }

    ueInfo.position = nodePositions.rbegin()->second;
    ueInfos.push_back(ueInfo);
  }
  return ueInfos;
}

std::vector<OranLmLte2LteRsrpHandover::EnbInfo>
OranLmLte2LteRsrpHandover::GetEnbInfos(Ptr<OranDataRepository> data) const
{
  std::vector<EnbInfo> enbInfos;
  for (auto enbId : data->GetLteEnbE2NodeIds())
  {
    EnbInfo enbInfo;
    enbInfo.nodeId = enbId;

    bool found;
    std::tie(found, enbInfo.cellId) = data->GetLteEnbCellInfo(enbInfo.nodeId);
    if (!found)
    {
      NS_LOG_INFO("No eNB cell info for E2 eNB " << enbInfo.nodeId);
      continue;
    }

    auto nodePositions = data->GetNodePositions(enbInfo.nodeId, Seconds(0), Simulator::Now());
    if (nodePositions.empty())
    {
      NS_LOG_INFO("No eNB position for E2 eNB " << enbInfo.nodeId);
      continue;
    }

    enbInfo.position = nodePositions.rbegin()->second;
    enbInfos.push_back(enbInfo);
  }
  return enbInfos;
}

/** Validate cellId→E2 mapping using the Data Repository only; return 0 if invalid. */
static uint64_t
SafeCellIdToEnbE2(Ptr<ns3::OranDataRepository> repo,
                  const std::unordered_map<uint16_t, uint64_t>& cellToE2,
                  uint16_t cellId)
{
  auto it = cellToE2.find(cellId);
  if (it == cellToE2.end())
  {
    return 0; // no mapping for this cell
  }
  const uint64_t e2 = it->second;

  // Confirm the E2 node is currently known by the repository
  auto enbIds = repo->GetLteEnbE2NodeIds();
  if (std::find(enbIds.begin(), enbIds.end(), e2) == enbIds.end())
  {
    return 0; // not registered/known at this time
  }
  return e2;
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteRsrpHandover::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<UeInfo> ueInfos,
    std::vector<EnbInfo> enbInfos) const
{
  std::vector<Ptr<OranCommand>> commands;

  // Map LTE cellId -> eNB E2NodeId
  std::unordered_map<uint16_t, uint64_t> cellIdToEnbE2;
  cellIdToEnbE2.reserve(enbInfos.size());
  for (const auto& e : enbInfos)
    cellIdToEnbE2.emplace(e.cellId, e.nodeId);

  const double now = Simulator::Now().GetSeconds();

  for (const auto& ueInfo : ueInfos)
  {
    // Debounce repeated HOs per UE
    auto itLast = g_lastHoTime.find(ueInfo.nodeId);
    if (itLast != g_lastHoTime.end() && (now - itLast->second) < kHoHoldoffSec)
    {
      NS_LOG_INFO("UE " << ueInfo.nodeId << ": within HO hold-off; skipping.");
      continue;
    }

    // Must have a valid RNTI
    if (ueInfo.rnti == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId << ": RNTI=0; suppressing HO.");
      continue;
    }

    // Pull latest RSRP/RSRQ
    auto meas = data->GetLteUeRsrpRsrq(ueInfo.nodeId);
    if (meas.empty())
    {
      NS_LOG_INFO("UE " << ueInfo.nodeId << ": no RSRP/RSRQ measurements; skipping.");
      continue;
    }

    // Track best & current-cell RSRP
    double   bestRsrp = -DBL_MAX;
    uint16_t bestCell = ueInfo.cellId;
    double   currRsrp = -DBL_MAX;

    for (const auto& m : meas)
    {
      uint16_t rnti, cellId; double rsrp, rsrq; bool serving; uint8_t ccid;
      std::tie(rnti, cellId, rsrp, rsrq, serving, ccid) = m;

      if (!std::isfinite(rsrp)) continue;     // guard against NaN/Inf

      if (cellId == ueInfo.cellId && rsrp > currRsrp)
        currRsrp = rsrp;

      if (rsrp > bestRsrp)
      {
        bestRsrp = rsrp;
        bestCell = cellId;
      }
    }

    // If we never saw a finite current-cell RSRP, be conservative
    if (!std::isfinite(currRsrp))
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId << ": no finite current-cell RSRP; skipping.");
      continue;
    }

    // If best is current (or not better than hysteresis), skip
    if (bestCell == ueInfo.cellId || (bestRsrp - currRsrp) < kRsrpHystDb)
      continue;

    // Resolve **serving** and **target** eNB E2 ids and verify they exist (repo-based)
    const uint64_t servingE2 = SafeCellIdToEnbE2(data, cellIdToEnbE2, ueInfo.cellId);
    if (servingE2 == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId
                        << ": serving cellId " << ueInfo.cellId
                        << " has no valid eNB E2 node; suppressing HO.");
      continue;
    }

    const uint64_t targetE2  = SafeCellIdToEnbE2(data, cellIdToEnbE2, bestCell);
    if (targetE2 == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId
                        << ": target cellId " << bestCell
                        << " has no valid eNB E2 node; suppressing HO.");
      continue;
    }

    // Build and log command
    // Address the **serving eNB** (it executes the HO to target cellId).
    Ptr<OranCommandLte2LteHandover> cmd = CreateObject<OranCommandLte2LteHandover>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(servingE2)); // execution terminator
    cmd->SetAttribute("TargetRnti",     UintegerValue(ueInfo.rnti));
    cmd->SetAttribute("TargetCellId",   UintegerValue(bestCell));

    data->LogCommandLm(m_name, cmd);
    commands.push_back(cmd);

    g_lastHoTime[ueInfo.nodeId] = now;

    NS_LOG_INFO("UE " << ueInfo.nodeId << ": HO requested "
                 << ueInfo.cellId << " → " << bestCell
                 << " (RSRP " << currRsrp << "→" << bestRsrp
                 << ", servingE2 " << servingE2
                 << ", targetE2 "  << targetE2
                 << ", RNTI " << ueInfo.rnti << ")");
  }

  return commands;
}

} // namespace ns3

