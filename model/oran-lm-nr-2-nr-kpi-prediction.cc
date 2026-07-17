/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-lm-nr-2-nr-kpi-prediction.h"

#include "oran-command-nr-2-nr-tx-power.h"
#include "oran-data-repository.h"
#include "oran-near-rt-ric.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <map>
#include <sstream>
#include <tuple>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrKpiPrediction");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrKpiPrediction);

TypeId
OranLmNr2NrKpiPrediction::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrKpiPrediction")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrKpiPrediction>()
            .AddAttribute("ProactiveThresholdDbm",
                          "Predicted RSRP below which a preemptive TxPower increase is issued.",
                          DoubleValue(-65.0),
                          MakeDoubleAccessor(&OranLmNr2NrKpiPrediction::m_proactiveThreshDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReactiveThresholdDbm",
                          "Current RSRP below which no preemptive action is taken "
                          "(already in reactive territory, let CCO handle it).",
                          DoubleValue(-70.0),
                          MakeDoubleAccessor(&OranLmNr2NrKpiPrediction::m_reactiveThreshDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("EmaAlpha",
                          "EMA smoothing factor in (0,1): higher = more responsive to recent KPIs.",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&OranLmNr2NrKpiPrediction::m_emaAlpha),
                          MakeDoubleChecker<double>(0.01, 0.99))
            .AddAttribute("StepSizeDb",
                          "TxPower increase per proactive action (dB).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmNr2NrKpiPrediction::m_stepSizeDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("PredictionHorizon",
                          "Number of LM-query cycles ahead to project the RSRP trend.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&OranLmNr2NrKpiPrediction::m_predictionHorizon),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MinRsrpSamples",
                          "Minimum RSRP samples per gNB before the predictor activates.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&OranLmNr2NrKpiPrediction::m_minRsrpSamples),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

OranLmNr2NrKpiPrediction::OranLmNr2NrKpiPrediction()
    : OranLm(),
      m_proactiveThreshDbm(-65.0),
      m_reactiveThreshDbm(-70.0),
      m_emaAlpha(0.4),
      m_stepSizeDb(1.0),
      m_predictionHorizon(2),
      m_minRsrpSamples(1)
{
    m_name = "OranLmNr2NrKpiPrediction";
}

OranLmNr2NrKpiPrediction::~OranLmNr2NrKpiPrediction() = default;

std::vector<Ptr<OranCommand>>
OranLmNr2NrKpiPrediction::Run()
{
    std::vector<Ptr<OranCommand>> commands;
    if (!m_active)
        return commands;

    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run KPI prediction LM with NULL Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();

    std::map<uint16_t, uint64_t> cellToE2;
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetNrGnbCellInfo(gnbId);
        if (found)
            cellToE2[cellId] = gnbId;
    }

    struct CellStats
    {
        double minRsrp = 0.0;
        bool hasMin = false;
        uint32_t n = 0;
    };
    std::map<uint16_t, CellStats> cellStats;

    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool found = false;
        uint16_t servCell = 0;
        uint16_t servRnti = 0;
        std::tie(found, servCell, servRnti) = data->GetNrUeCellInfo(ueId);
        if (!found)
            continue;

        for (const auto& meas : data->GetNrUeRsrpRsrq(ueId))
        {
            uint16_t rnti = 0, cellId = 0;
            double rsrp = 0.0, rsrq = 0.0;
            bool serving = false;
            uint8_t ccid = 0;
            std::tie(rnti, cellId, rsrp, rsrq, serving, ccid) = meas;
            if (cellId != servCell)
                continue;
            CellStats& cs = cellStats[cellId];
            if (!cs.hasMin || rsrp < cs.minRsrp)
            {
                cs.minRsrp = rsrp;
                cs.hasMin = true;
            }
            cs.n++;
        }
    }

    for (const auto& cellEntry : cellStats)
    {
        uint16_t cellId = cellEntry.first;
        const CellStats& cs = cellEntry.second;

        if (cs.n < m_minRsrpSamples)
            continue;
        if (cellToE2.find(cellId) == cellToE2.end())
            continue;

        uint64_t gnbE2Id = cellToE2[cellId];
        double minRsrp = cs.minRsrp;

        EmaState& ema = m_emaState[gnbE2Id];
        bool wasInitialized = ema.initialized;
        if (!ema.initialized)
        {
            ema.ema = minRsrp;
            ema.prevEma = minRsrp;
            ema.initialized = true;
        }
        else
        {
            ema.prevEma = ema.ema;
            ema.ema = m_emaAlpha * minRsrp + (1.0 - m_emaAlpha) * ema.ema;
        }

        double slope = wasInitialized ? (ema.ema - ema.prevEma) : 0.0;

        double rsrpPredicted = ema.ema + slope * static_cast<double>(m_predictionHorizon);

        std::ostringstream msg;
        msg << "KPI-predict e2=" << gnbE2Id << " cell=" << cellId << " samples=" << cs.n
            << " minRsrp=" << minRsrp << " dBm"
            << " ema=" << ema.ema << " dBm"
            << " slope=" << slope << " dBm/cycle"
            << " predicted=" << rsrpPredicted << " dBm"
            << " proactiveThr=" << m_proactiveThreshDbm << " dBm";

        if (rsrpPredicted < m_proactiveThreshDbm && minRsrp > m_reactiveThreshDbm)
        {
            msg << " action=PROACTIVE_TXP_INCREASE stepDb=" << m_stepSizeDb;
            NS_LOG_INFO(msg.str());
            data->LogActionLm(m_name, msg.str());

            Ptr<OranCommandNr2NrTxPower> cmd = CreateObject<OranCommandNr2NrTxPower>();
            cmd->SetAttribute("TargetE2NodeId", UintegerValue(gnbE2Id));
            cmd->SetAttribute("PowerDeltaDb", DoubleValue(m_stepSizeDb));
            data->LogCommandLm(m_name, cmd);
            commands.push_back(cmd);

            ema.actionCount++;
        }
        else
        {
            msg << " action=NONE";
            NS_LOG_INFO(msg.str());
        }
    }

    return commands;
}

} // namespace ns3
