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
                          MakeBooleanChecker());
    return tid;
}

OranLmLte2LteMobilityLoadBalancing::OranLmLte2LteMobilityLoadBalancing()
    : OranLm(),
      m_loadImbalanceThreshold(0.20),
      m_cioStepDb(1.0),
      m_maxAbsCioDb(6.0),
      m_hotCellTttSec(0.0),
      m_controlTtt(false)
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
    std::map<uint16_t, uint32_t> ueCountByCell;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbId);
        if (found)
        {
            cellToE2[cellId] = enbId;
            ueCountByCell[cellId] = 0;
        }
    }

    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        uint16_t rnti = 0;
        std::tie(found, cellId, rnti) = data->GetLteUeCellInfo(ueId);
        if (found && ueCountByCell.find(cellId) != ueCountByCell.end())
        {
            ueCountByCell[cellId]++;
        }
    }

    if (ueCountByCell.empty())
    {
        return commands;
    }

    uint32_t totalUes = 0;
    for (const auto& item : ueCountByCell)
    {
        totalUes += item.second;
    }
    const double avgLoad = static_cast<double>(totalUes) / ueCountByCell.size();
    if (avgLoad <= 0.0)
    {
        return commands;
    }

    for (const auto& item : ueCountByCell)
    {
        const uint16_t cellId = item.first;
        const uint32_t load = item.second;
        const uint64_t e2NodeId = cellToE2[cellId];
        const double normalizedError = (static_cast<double>(load) - avgLoad) / avgLoad;

        if (std::abs(normalizedError) < m_loadImbalanceThreshold)
        {
            continue;
        }

        OranLteCellControlParams params = GetLteCellControlParameters(e2NodeId);
        const double direction = normalizedError > 0.0 ? -1.0 : 1.0;
        const double newCio =
            std::max(-m_maxAbsCioDb,
                     std::min(m_maxAbsCioDb, params.cioDb + direction * m_cioStepDb));

        Ptr<OranCommandLte2LteCellParameter> cioCmd =
            CreateCellParameterCommand(e2NodeId, "CIO", newCio);
        data->LogCommandLm(m_name, cioCmd);
        commands.push_back(cioCmd);

        if (m_controlTtt && normalizedError > 0.0)
        {
            Ptr<OranCommandLte2LteCellParameter> tttCmd =
                CreateCellParameterCommand(e2NodeId, "TTT", m_hotCellTttSec);
            data->LogCommandLm(m_name, tttCmd);
            commands.push_back(tttCmd);
        }

        std::ostringstream msg;
        msg << "MLB cellId=" << cellId << " load=" << load << " avgLoad=" << avgLoad
            << " normalizedError=" << normalizedError << " newCio=" << newCio;
        data->LogActionLm(m_name, msg.str());
    }

    return commands;
}

} // namespace ns3
