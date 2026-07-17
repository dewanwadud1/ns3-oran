/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-lm-nr-2-nr-mobility-load-balancing.h"

#include "oran-command-nr-2-nr-cell-parameter.h"
#include "oran-data-repository.h"
#include "oran-nr-cell-control-state.h"
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

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrMobilityLoadBalancing");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrMobilityLoadBalancing);

namespace
{

Ptr<OranCommandNr2NrCellParameter>
CreateCellParameterCommand(uint64_t e2NodeId, const std::string& name, double value)
{
    Ptr<OranCommandNr2NrCellParameter> cmd = CreateObject<OranCommandNr2NrCellParameter>();
    cmd->SetAttribute("TargetE2NodeId", UintegerValue(e2NodeId));
    cmd->SetAttribute("ParameterName", StringValue(name));
    cmd->SetAttribute("Value", DoubleValue(value));
    cmd->SetAttribute("IsDelta", BooleanValue(false));
    return cmd;
}

} // namespace

TypeId
OranLmNr2NrMobilityLoadBalancing::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrMobilityLoadBalancing")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrMobilityLoadBalancing>()
            .AddAttribute("LoadImbalanceThreshold",
                          "Fractional demand imbalance around the average that triggers MLB.",
                          DoubleValue(0.20),
                          MakeDoubleAccessor(
                              &OranLmNr2NrMobilityLoadBalancing::m_loadImbalanceThreshold),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("CioStep",
                          "CIO adjustment step in dB.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmNr2NrMobilityLoadBalancing::m_cioStepDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MaxAbsCio",
                          "Absolute CIO clamp in dB.",
                          DoubleValue(6.0),
                          MakeDoubleAccessor(&OranLmNr2NrMobilityLoadBalancing::m_maxAbsCioDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("HotCellTttSec",
                          "TTT value requested for overloaded cells when TTT control is enabled.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranLmNr2NrMobilityLoadBalancing::m_hotCellTttSec),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("ControlTtt",
                          "Whether MLB also writes TTT for overloaded cells.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranLmNr2NrMobilityLoadBalancing::m_controlTtt),
                          MakeBooleanChecker())
            .AddAttribute("EnbCapacityMbps",
                          "Fixed per-gNB bandwidth capacity that demand is balanced against "
                          "(uniform across all gNBs).",
                          DoubleValue(50.0),
                          MakeDoubleAccessor(&OranLmNr2NrMobilityLoadBalancing::m_enbCapacityMbps),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

OranLmNr2NrMobilityLoadBalancing::OranLmNr2NrMobilityLoadBalancing()
    : OranLm(),
      m_loadImbalanceThreshold(0.20),
      m_cioStepDb(1.0),
      m_maxAbsCioDb(6.0),
      m_hotCellTttSec(0.0),
      m_controlTtt(false),
      m_enbCapacityMbps(50.0)
{
    m_name = "OranLmNr2NrMobilityLoadBalancing";
}

OranLmNr2NrMobilityLoadBalancing::~OranLmNr2NrMobilityLoadBalancing() = default;

std::vector<Ptr<OranCommand>>
OranLmNr2NrMobilityLoadBalancing::Run()
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
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetNrGnbCellInfo(gnbId);
        if (found)
        {
            cellToE2[cellId] = gnbId;
            demandMbpsByCell[cellId] = 0.0;
        }
    }

    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        uint16_t rnti = 0;
        std::tie(found, cellId, rnti) = data->GetNrUeCellInfo(ueId);
        if (found && demandMbpsByCell.find(cellId) != demandMbpsByCell.end())
        {
            demandMbpsByCell[cellId] += data->GetNrUeAppDemand(ueId);
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

        const bool overloaded = utilization > 1.0;
        if (std::abs(normalizedError) < m_loadImbalanceThreshold && !overloaded)
        {
            continue;
        }

        OranNrCellControlParams params = GetNrCellControlParameters(e2NodeId);
        const double direction = (normalizedError > 0.0 || overloaded) ? -1.0 : 1.0;
        const double newCio =
            std::max(-m_maxAbsCioDb,
                     std::min(m_maxAbsCioDb, params.cioDb + direction * m_cioStepDb));

        Ptr<OranCommandNr2NrCellParameter> cioCmd =
            CreateCellParameterCommand(e2NodeId, "CIO", newCio);
        data->LogCommandLm(m_name, cioCmd);
        commands.push_back(cioCmd);

        if (m_controlTtt && (normalizedError > 0.0 || overloaded))
        {
            Ptr<OranCommandNr2NrCellParameter> tttCmd =
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
