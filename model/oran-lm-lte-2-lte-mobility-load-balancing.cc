/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "oran-lm-lte-2-lte-mobility-load-balancing.h"

#include "oran-command-lte-2-lte-cell-parameter.h"
#include "oran-data-repository.h"
#include "oran-lte-cell-control-state.h"
#include "oran-near-rt-ric.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <tuple>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteMobilityLoadBalancing");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteMobilityLoadBalancing);

namespace
{

Ptr<OranCommandLte2LteCellParameter>
CreateCellParameterCommand(uint64_t e2NodeId, const std::string& name, double value)
{
    Ptr<OranCommandLte2LteCellParameter> cmd = CreateObject<OranCommandLte2LteCellParameter>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(e2NodeId));
    cmd->SetAttribute("ParameterName", StringValue(name));
    cmd->SetAttribute("Value", DoubleValue(value));
    cmd->SetAttribute("IsDelta", BooleanValue(false));
    return cmd;
}

} // namespace

TypeId
OranLmLte2LteMobilityLoadBalancing::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmLte2LteMobilityLoadBalancing")
            .SetParent<OranLm>()
            .AddConstructor<OranLmLte2LteMobilityLoadBalancing>()
            .AddAttribute("LoadImbalanceThreshold",
                          "Fractional UE-count imbalance around the average that triggers MLB.",
                          DoubleValue(0.20),
                          MakeDoubleAccessor(
                              &OranLmLte2LteMobilityLoadBalancing::m_loadImbalanceThreshold),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("CioStep",
                          "CIO adjustment step in dB.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmLte2LteMobilityLoadBalancing::m_cioStepDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MaxAbsCio",
                          "Absolute CIO clamp in dB.",
                          DoubleValue(6.0),
                          MakeDoubleAccessor(&OranLmLte2LteMobilityLoadBalancing::m_maxAbsCioDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("HotCellTttSec",
                          "TTT value requested for overloaded cells when TTT control is enabled.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranLmLte2LteMobilityLoadBalancing::m_hotCellTttSec),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ControlTtt",
                          "Whether MLB also writes TTT for overloaded cells.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranLmLte2LteMobilityLoadBalancing::m_controlTtt),
                          MakeBooleanChecker())
            .AddAttribute("EnbCapacityMbps",
                          "Fixed per-eNB bandwidth capacity that demand is balanced against "
                          "(uniform across all eNBs). Should match the scenario's "
                          "--enb-capacity-mbps.",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&OranLmLte2LteMobilityLoadBalancing::m_enbCapacityMbps),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

OranLmLte2LteMobilityLoadBalancing::OranLmLte2LteMobilityLoadBalancing()
    : OranLm(),
      m_loadImbalanceThreshold(0.20),
      m_cioStepDb(1.0),
      m_maxAbsCioDb(6.0),
      m_hotCellTttSec(0.0),
      m_controlTtt(false),
      m_enbCapacityMbps(50.0)
{
    m_name = "OranLmLte2LteMobilityLoadBalancing";
}

OranLmLte2LteMobilityLoadBalancing::~OranLmLte2LteMobilityLoadBalancing() = default;

std::vector<Ptr<OranCommand>>
OranLmLte2LteMobilityLoadBalancing::Run()
{
    std::vector<Ptr<OranCommand>> commands;
    if (!m_active)
    {
        return commands;
    }

    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run MLB LM with NULL Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();

    std::map<uint16_t, uint64_t> cellToE2;
    std::map<uint16_t, double> demandMbpsByCell;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbId);
        if (found)
        {
            cellToE2[cellId] = enbId;
            demandMbpsByCell[cellId] = 0.0;
        }
    }

    // Aggregate REAL observed per-UE demand (OranReporterLteUeAppDemand,
    // fed by actual eMBB/URLLC/mMTC/V2X application traffic) by each UE's
    // current serving cell -- replaces the previous raw UE-count tally, so
    // MLB balances actual bandwidth demand, not just how many UEs happen to
    // be attached.
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        uint16_t rnti = 0;
        std::tie(found, cellId, rnti) = data->GetLteUeCellInfo(ueId);
        if (found && demandMbpsByCell.find(cellId) != demandMbpsByCell.end())
        {
            demandMbpsByCell[cellId] += data->GetLteUeAppDemand(ueId);
        }
    }

    if (demandMbpsByCell.empty())
    {
        return commands;
    }

    double totalDemandMbps = 0.0;
    for (const auto& item : demandMbpsByCell)
    {
        totalDemandMbps += item.second;
    }
    const double avgDemandMbps = totalDemandMbps / demandMbpsByCell.size();
    if (totalDemandMbps <= 0.0)
    {
        return commands;
    }

    for (const auto& item : demandMbpsByCell)
    {
        const uint16_t cellId = item.first;
        const double demandMbps = item.second;
        const uint64_t e2NodeId = cellToE2[cellId];
        const double utilization = demandMbps / m_enbCapacityMbps;
        const double normalizedError =
            (avgDemandMbps > 0.0) ? (demandMbps - avgDemandMbps) / avgDemandMbps : 0.0;

        // Two independent triggers: relative imbalance (as before, now over
        // demand instead of UE count) OR a cell literally exceeding its
        // fixed capacity ceiling -- the latter is an absolute problem worth
        // shedding load for even if it isn't far from the network average
        // (e.g. every cell running hot together).
        const bool overloaded = utilization > 1.0;
        if (std::abs(normalizedError) < m_loadImbalanceThreshold && !overloaded)
        {
            continue;
        }

        OranLteCellControlParams params = GetLteCellControlParameters(e2NodeId);
        const double direction = (normalizedError > 0.0 || overloaded) ? -1.0 : 1.0;
        const double newCio =
            std::max(-m_maxAbsCioDb,
                     std::min(m_maxAbsCioDb, params.cioDb + direction * m_cioStepDb));

        Ptr<OranCommandLte2LteCellParameter> cioCmd =
            CreateCellParameterCommand(e2NodeId, "CIO", newCio);
        data->LogCommandLm(m_name, cioCmd);
        commands.push_back(cioCmd);

        if (m_controlTtt && (normalizedError > 0.0 || overloaded))
        {
            Ptr<OranCommandLte2LteCellParameter> tttCmd =
                CreateCellParameterCommand(e2NodeId, "TTT", m_hotCellTttSec);
            data->LogCommandLm(m_name, tttCmd);
            commands.push_back(tttCmd);
        }

        std::ostringstream msg;
        msg << "MLB cellId=" << cellId << " demandMbps=" << demandMbps
            << " avgDemandMbps=" << avgDemandMbps << " utilization=" << utilization
            << " normalizedError=" << normalizedError << " newCio=" << newCio;
        data->LogActionLm(m_name, msg.str());
    }

    return commands;
}

} // namespace ns3
