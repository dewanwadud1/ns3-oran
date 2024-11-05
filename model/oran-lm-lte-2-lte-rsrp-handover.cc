#include "oran-lm-lte-2-lte-rsrp-handover.h"

#include "oran-command-lte-2-lte-handover.h"
#include "oran-data-repository.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/double.h>
#include <ns3/uinteger.h>

#include <cfloat>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteRsrpHandover");

NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteRsrpHandover);

TypeId
OranLmLte2LteRsrpHandover::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::OranLmLte2LteRsrpHandover")
                            .SetParent<OranLm>()
                            .AddConstructor<OranLmLte2LteRsrpHandover>()
                            .AddAttribute("HysteresisMargin",
                                          "The hysteresis margin in dB to avoid ping-pong handovers.",
                                          DoubleValue(3.0),
                                          MakeDoubleAccessor(&OranLmLte2LteRsrpHandover::m_hysteresisMargin),
                                          MakeDoubleChecker<double>())
                            .AddAttribute("TimeWindow",
                                          "Time window in seconds to consider RSRP measurements.",
                                          TimeValue(Seconds(1.0)),
                                          MakeTimeAccessor(&OranLmLte2LteRsrpHandover::m_timeWindow),
                                          MakeTimeChecker());
    return tid;
}

OranLmLte2LteRsrpHandover::OranLmLte2LteRsrpHandover(void)
    : OranLm(),
      m_hysteresisMargin(3.0),
      m_timeWindow(Seconds(1.0))
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmLte2LteRsrpHandover";
}

OranLmLte2LteRsrpHandover::~OranLmLte2LteRsrpHandover(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteRsrpHandover::Run(void)
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<UeInfo> ueInfos = GetUeInfos(data);
        std::vector<EnbInfo> enbInfos = GetEnbInfos(data);
        commands = GetHandoverCommands(data, ueInfos, enbInfos);
    }

    // Return the commands.
    return commands;
}

std::vector<OranLmLte2LteRsrpHandover::UeInfo>
OranLmLte2LteRsrpHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;
        // Get the current cell ID and RNTI of the UE and record it.
        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetLteUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            ueInfos.push_back(ueInfo);
        }
        else
        {
            NS_LOG_INFO("Could not find LTE UE cell info for E2 Node ID = " << ueInfo.nodeId);
        }
    }
    return ueInfos;
}

std::vector<OranLmLte2LteRsrpHandover::EnbInfo>
OranLmLte2LteRsrpHandover::GetEnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<EnbInfo> enbInfos;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        EnbInfo enbInfo;
        enbInfo.nodeId = enbId;
        // Get the cell ID of this eNB and record it.
        bool found;
        std::tie(found, enbInfo.cellId) = data->GetLteEnbCellInfo(enbInfo.nodeId);
        if (found)
        {
            enbInfos.push_back(enbInfo);
        }
        else
        {
            NS_LOG_INFO("Could not find LTE eNB cell info for E2 Node ID = " << enbInfo.nodeId);
        }
    }
    return enbInfos;
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteRsrpHandover::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<OranLmLte2LteRsrpHandover::UeInfo> ueInfos,
    std::vector<OranLmLte2LteRsrpHandover::EnbInfo> enbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<Ptr<OranCommand>> commands;

    // For each UE, analyze RSRP measurements.
    for (auto ueInfo : ueInfos)
    {
        // Retrieve RSRP measurements for this UE within the time window.
        std::map<Time, std::map<uint16_t, double>> rsrpMeasurements =
            data->GetUeRsrp(ueInfo.nodeId, Simulator::Now() - m_timeWindow, Simulator::Now());

        if (rsrpMeasurements.empty())
        {
            NS_LOG_INFO("No RSRP measurements found for UE E2 Node ID = " << ueInfo.nodeId);
            continue;
        }

        // Aggregate RSRP measurements to find average RSRP per cell.
        std::map<uint16_t, std::vector<double>> cellRsrpValues;
        for (const auto& timeEntry : rsrpMeasurements)
        {
            for (const auto& cellEntry : timeEntry.second)
            {
                cellRsrpValues[cellEntry.first].push_back(cellEntry.second);
            }
        }

        // Compute average RSRP per cell.
        std::map<uint16_t, double> cellAvgRsrp;
        for (const auto& cellEntry : cellRsrpValues)
        {
            double sum = 0.0;
            for (double val : cellEntry.second)
            {
                sum += val;
            }
            cellAvgRsrp[cellEntry.first] = sum / cellEntry.second.size();
        }

        // Identify the cell with the highest average RSRP.
        uint16_t bestCellId = ueInfo.cellId;
        double bestRsrp = -DBL_MAX;

        for (const auto& cellEntry : cellAvgRsrp)
        {
            NS_LOG_INFO("UE " << ueInfo.nodeId << " average RSRP from CellID "
                              << cellEntry.first << " is " << cellEntry.second << " dBm");

            if (cellEntry.second > bestRsrp)
            {
                bestRsrp = cellEntry.second;
                bestCellId = cellEntry.first;
            }
        }

        // Check if the best cell is different from the current serving cell and the RSRP difference exceeds hysteresis.
        if (bestCellId != ueInfo.cellId &&
            (bestRsrp - cellAvgRsrp[ueInfo.cellId] > m_hysteresisMargin))
        {
            // Find the nodeId of the best eNB.
            uint64_t targetEnbNodeId = 0;
            for (const auto& enbInfo : enbInfos)
            {
                if (enbInfo.cellId == bestCellId)
                {
                    targetEnbNodeId = enbInfo.nodeId;
                    break;
                }
            }

            if (targetEnbNodeId == 0)
            {
                NS_LOG_WARN("Could not find eNB with CellID = " << bestCellId);
                continue;
            }

            // Issue handover command.
            Ptr<OranCommandLte2LteHandover> handoverCommand =
                CreateObject<OranCommandLte2LteHandover>();
            // Send the command to the current serving eNB.
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(ueInfo.nodeId));
            // Use the RNTI that the current cell is using to identify the UE.
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));
            // Give the current cell the ID of the new cell to handover to.
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(bestCellId));
            // Log the command to the storage
            data->LogCommandLm(m_name, handoverCommand);
            // Add the command to send.
            commands.push_back(handoverCommand);

            NS_LOG_INFO("Issuing handover command for UE " << ueInfo.nodeId
                                                           << " from CellID " << ueInfo.cellId
                                                           << " to CellID " << bestCellId
                                                           << " due to better RSRP (" << bestRsrp
                                                           << " dBm) exceeding hysteresis margin.");
        }
    }
    return commands;
}

} // namespace ns3

