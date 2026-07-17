/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * RSRP-driven NR->NR handover LM. Mirrors OranLmLte2LteRsrpHandover.
 */

#include "oran-lm-nr-2-nr-rsrp-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"
#include "oran-nr-cell-control-state.h"

#include <ns3/abort.h>
#include <ns3/boolean.h>
#include <ns3/double.h>
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

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrRsrpHandover");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrRsrpHandover);

TypeId
OranLmNr2NrRsrpHandover::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::OranLmNr2NrRsrpHandover")
                        .SetParent<OranLm>()
                        .AddConstructor<OranLmNr2NrRsrpHandover>()
                        .AddAttribute("HandoverHoldoffSec",
                                      "Minimum time between handovers per UE.",
                                      DoubleValue(1.5),
                                      MakeDoubleAccessor(&OranLmNr2NrRsrpHandover::m_handoverHoldoffSec),
                                      MakeDoubleChecker<double>(0.0))
                        .AddAttribute("RsrpHysteresisDb",
                                      "Base RSRP margin required before triggering handover.",
                                      DoubleValue(1.0),
                                      MakeDoubleAccessor(&OranLmNr2NrRsrpHandover::m_rsrpHysteresisDb),
                                      MakeDoubleChecker<double>(0.0))
                        .AddAttribute("TimeToTriggerSec",
                                      "Base time that a target must remain preferred before handover.",
                                      DoubleValue(0.0),
                                      MakeDoubleAccessor(&OranLmNr2NrRsrpHandover::m_timeToTriggerSec),
                                      MakeDoubleChecker<double>(0.0))
                        .AddAttribute("EnableCellControlBias",
                                      "Use cell-control CIO/TTT/HYS state in the RSRP handover decision.",
                                      BooleanValue(true),
                                      MakeBooleanAccessor(&OranLmNr2NrRsrpHandover::m_enableCellControlBias),
                                      MakeBooleanChecker());
  return tid;
}

OranLmNr2NrRsrpHandover::OranLmNr2NrRsrpHandover(void)
  : OranLm()
{
  m_name = "OranLmNr2NrRsrpHandover";
  m_handoverHoldoffSec = 1.5;
  m_rsrpHysteresisDb = 1.0;
  m_timeToTriggerSec = 0.0;
  m_enableCellControlBias = true;
}

OranLmNr2NrRsrpHandover::~OranLmNr2NrRsrpHandover(void) = default;

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandover::Run(void)
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
  auto gnbInfos = GetGnbInfos(data);

  commands = GetHandoverCommands(data, ueInfos, gnbInfos);
  return commands;
}

std::vector<OranLmNr2NrRsrpHandover::UeInfo>
OranLmNr2NrRsrpHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
  std::vector<UeInfo> ueInfos;
  for (auto ueId : data->GetNrUeE2NodeIds())
  {
    UeInfo ueInfo;
    ueInfo.nodeId = ueId;

    bool found;
    std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetNrUeCellInfo(ueInfo.nodeId);
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

std::vector<OranLmNr2NrRsrpHandover::GnbInfo>
OranLmNr2NrRsrpHandover::GetGnbInfos(Ptr<OranDataRepository> data) const
{
  std::vector<GnbInfo> gnbInfos;
  for (auto gnbId : data->GetNrGnbE2NodeIds())
  {
    GnbInfo gnbInfo;
    gnbInfo.nodeId = gnbId;

    bool found;
    std::tie(found, gnbInfo.cellId) = data->GetNrGnbCellInfo(gnbInfo.nodeId);
    if (!found)
    {
      NS_LOG_INFO("No gNB cell info for E2 gNB " << gnbInfo.nodeId);
      continue;
    }

    auto nodePositions = data->GetNodePositions(gnbInfo.nodeId, Seconds(0), Simulator::Now());
    if (nodePositions.empty())
    {
      NS_LOG_INFO("No gNB position for E2 gNB " << gnbInfo.nodeId);
      continue;
    }

    gnbInfo.position = nodePositions.rbegin()->second;
    gnbInfos.push_back(gnbInfo);
  }
  return gnbInfos;
}

/** Validate cellId→E2 mapping using the Data Repository only; return 0 if invalid. */
static uint64_t
SafeCellIdToGnbE2(Ptr<ns3::OranDataRepository> repo,
                  const std::unordered_map<uint16_t, uint64_t>& cellToE2,
                  uint16_t cellId)
{
  auto it = cellToE2.find(cellId);
  if (it == cellToE2.end())
  {
    return 0; // no mapping for this cell
  }
  const uint64_t e2 = it->second;

  auto gnbIds = repo->GetNrGnbE2NodeIds();
  if (std::find(gnbIds.begin(), gnbIds.end(), e2) == gnbIds.end())
  {
    return 0; // not registered/known at this time
  }
  return e2;
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandover::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<UeInfo> ueInfos,
    std::vector<GnbInfo> gnbInfos) const
{
  std::vector<Ptr<OranCommand>> commands;

  std::unordered_map<uint16_t, uint64_t> cellIdToGnbE2;
  cellIdToGnbE2.reserve(gnbInfos.size());
  for (const auto& e : gnbInfos)
    cellIdToGnbE2.emplace(e.cellId, e.nodeId);

  const double now = Simulator::Now().GetSeconds();

  for (const auto& ueInfo : ueInfos)
  {
    auto itLast = m_lastHoTime.find(ueInfo.nodeId);
    if (itLast != m_lastHoTime.end() && (now - itLast->second) < m_handoverHoldoffSec)
    {
      NS_LOG_INFO("UE " << ueInfo.nodeId << ": within HO hold-off; skipping.");
      continue;
    }

    if (ueInfo.rnti == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId << ": RNTI=0; suppressing HO.");
      continue;
    }

    const uint64_t servingE2 = SafeCellIdToGnbE2(data, cellIdToGnbE2, ueInfo.cellId);
    if (servingE2 == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId
                        << ": serving cellId " << ueInfo.cellId
                        << " has no valid gNB E2 node; suppressing HO.");
      continue;
    }
    const OranNrCellControlParams servingParams =
        m_enableCellControlBias ? GetNrCellControlParameters(servingE2)
                                : OranNrCellControlParams();

    auto meas = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
    if (meas.empty())
    {
      NS_LOG_INFO("UE " << ueInfo.nodeId << ": no RSRP/RSRQ measurements; skipping.");
      continue;
    }

    double   bestRsrp = -DBL_MAX;
    uint16_t bestCell = ueInfo.cellId;
    double   currRsrp = -DBL_MAX;

    for (const auto& m : meas)
    {
      uint16_t rnti, cellId; double rsrp, rsrq; bool serving; uint8_t ccid;
      std::tie(rnti, cellId, rsrp, rsrq, serving, ccid) = m;

      if (!std::isfinite(rsrp)) continue;

      double adjustedRsrp = rsrp;
      if (m_enableCellControlBias)
      {
        const uint64_t cellE2 = SafeCellIdToGnbE2(data, cellIdToGnbE2, cellId);
        if (cellE2 != 0)
        {
          adjustedRsrp += GetNrCellControlParameters(cellE2).cioDb;
        }
      }

      if (cellId == ueInfo.cellId && adjustedRsrp > currRsrp)
        currRsrp = adjustedRsrp;

      if (adjustedRsrp > bestRsrp)
      {
        bestRsrp = adjustedRsrp;
        bestCell = cellId;
      }
    }

    if (!std::isfinite(currRsrp))
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId << ": no finite current-cell RSRP; skipping.");
      continue;
    }

    const double requiredMarginDb = m_rsrpHysteresisDb + servingParams.hysDb;
    if (bestCell == ueInfo.cellId || (bestRsrp - currRsrp) < requiredMarginDb)
    {
      m_pendingTargetSince.erase(ueInfo.nodeId);
      continue;
    }

    const double requiredTttSec = std::max(m_timeToTriggerSec, servingParams.tttSec);
    if (requiredTttSec > 0.0)
    {
      auto itPending = m_pendingTargetSince.find(ueInfo.nodeId);
      if (itPending == m_pendingTargetSince.end() || itPending->second.first != bestCell)
      {
        m_pendingTargetSince[ueInfo.nodeId] = std::make_pair(bestCell, now);
        continue;
      }
      if ((now - itPending->second.second) < requiredTttSec)
      {
        continue;
      }
    }

    const uint64_t targetE2  = SafeCellIdToGnbE2(data, cellIdToGnbE2, bestCell);
    if (targetE2 == 0)
    {
      NS_LOG_WARN("UE " << ueInfo.nodeId
                        << ": target cellId " << bestCell
                        << " has no valid gNB E2 node; suppressing HO.");
      continue;
    }

    Ptr<OranCommandNr2NrHandover> cmd = CreateObject<OranCommandNr2NrHandover>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(servingE2));
    cmd->SetAttribute("TargetRnti",     UintegerValue(ueInfo.rnti));
    cmd->SetAttribute("TargetCellId",   UintegerValue(bestCell));

    data->LogCommandLm(m_name, cmd);
    commands.push_back(cmd);

    m_lastHoTime[ueInfo.nodeId] = now;
    m_pendingTargetSince.erase(ueInfo.nodeId);

    NS_LOG_INFO("UE " << ueInfo.nodeId << ": HO requested "
                 << ueInfo.cellId << " -> " << bestCell
                 << " (adjusted RSRP " << currRsrp << " -> " << bestRsrp
                 << ", margin " << requiredMarginDb
                 << ", ttt " << requiredTttSec
                 << ", servingE2 " << servingE2
                 << ", targetE2 "  << targetE2
                 << ", RNTI " << ueInfo.rnti << ")");
  }

  return commands;
}

} // namespace ns3
