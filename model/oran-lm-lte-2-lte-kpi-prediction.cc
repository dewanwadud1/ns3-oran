/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "oran-lm-lte-2-lte-kpi-prediction.h"

#include "oran-command-lte-2-lte-tx-power.h"
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

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteKpiPrediction");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteKpiPrediction);

TypeId
OranLmLte2LteKpiPrediction::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmLte2LteKpiPrediction")
            .SetParent<OranLm>()
            .AddConstructor<OranLmLte2LteKpiPrediction>()
            .AddAttribute("ProactiveThresholdDbm",
                          "Predicted RSRP below which a preemptive TxPower increase is issued.",
                          DoubleValue(-65.0),
                          MakeDoubleAccessor(&OranLmLte2LteKpiPrediction::m_proactiveThreshDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("ReactiveThresholdDbm",
                          "Current RSRP below which no preemptive action is taken "
                          "(already in reactive territory, let CCO handle it).",
                          DoubleValue(-70.0),
                          MakeDoubleAccessor(&OranLmLte2LteKpiPrediction::m_reactiveThreshDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("EmaAlpha",
                          "EMA smoothing factor in (0,1): higher = more responsive to recent KPIs.",
                          DoubleValue(0.4),
                          MakeDoubleAccessor(&OranLmLte2LteKpiPrediction::m_emaAlpha),
                          MakeDoubleChecker<double>(0.01, 0.99))
            .AddAttribute("StepSizeDb",
                          "TxPower increase per proactive action (dB).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmLte2LteKpiPrediction::m_stepSizeDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("PredictionHorizon",
                          "Number of LM-query cycles ahead to project the RSRP trend.",
                          UintegerValue(2),
                          MakeUintegerAccessor(&OranLmLte2LteKpiPrediction::m_predictionHorizon),
                          MakeUintegerChecker<uint32_t>(1))
            .AddAttribute("MinRsrpSamples",
                          "Minimum RSRP samples per eNB before the predictor activates.",
                          UintegerValue(1),
                          MakeUintegerAccessor(&OranLmLte2LteKpiPrediction::m_minRsrpSamples),
                          MakeUintegerChecker<uint32_t>(1));
    return tid;
}

OranLmLte2LteKpiPrediction::OranLmLte2LteKpiPrediction()
    : OranLm(),
      m_proactiveThreshDbm(-65.0),
      m_reactiveThreshDbm(-70.0),
      m_emaAlpha(0.4),
      m_stepSizeDb(1.0),
      m_predictionHorizon(2),
      m_minRsrpSamples(1)
{
    m_name = "OranLmLte2LteKpiPrediction";
}

OranLmLte2LteKpiPrediction::~OranLmLte2LteKpiPrediction() = default;

std::vector<Ptr<OranCommand>>
OranLmLte2LteKpiPrediction::Run()
{
    std::vector<Ptr<OranCommand>> commands;
    if (!m_active)
        return commands;

    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run KPI prediction LM with NULL Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();

    // ── Step 1: build cellId → eNB e2NodeId map ──────────────────────────────
    std::map<uint16_t, uint64_t> cellToE2;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbId);
        if (found)
            cellToE2[cellId] = enbId;
    }

    // ── Step 2: compute per-serving-cell worst-case (minimum) RSRP ───────────
    // Using minimum instead of average so the predictor is sensitive to the
    // cell-edge UE whose RSRP is at risk, not masked by close UEs with high RSRP.
    struct CellStats
    {
        double minRsrp    = 0.0;
        bool   hasMin     = false;
        uint32_t n        = 0;
    };
    std::map<uint16_t, CellStats> cellStats;

    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool found = false;
        uint16_t servCell = 0;
        uint16_t servRnti = 0;
        std::tie(found, servCell, servRnti) = data->GetLteUeCellInfo(ueId);
        if (!found)
            continue;

        for (const auto& meas : data->GetLteUeRsrpRsrq(ueId))
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
                cs.hasMin  = true;
            }
            cs.n++;
        }
    }

    // ── Step 3: update EMA per eNB, predict, and act proactively ─────────────
    for (const auto& cellEntry : cellStats)
    {
        uint16_t cellId = cellEntry.first;
        const CellStats& cs = cellEntry.second;

        if (cs.n < m_minRsrpSamples)
            continue;
        if (cellToE2.find(cellId) == cellToE2.end())
            continue;

        uint64_t enbE2Id  = cellToE2[cellId];
        double   minRsrp  = cs.minRsrp;

        // Update EMA — fix initialization: seed prevEma = minRsrp so slope=0 on first call
        EmaState& ema = m_emaState[enbE2Id];
        bool wasInitialized = ema.initialized;
        if (!ema.initialized)
        {
            ema.ema         = minRsrp;
            ema.prevEma     = minRsrp;
            ema.initialized = true;
        }
        else
        {
            ema.prevEma = ema.ema;
            ema.ema     = m_emaAlpha * minRsrp + (1.0 - m_emaAlpha) * ema.ema;
        }

        // Estimate slope (dBm per cycle); zero on first call (no history yet)
        double slope = wasInitialized ? (ema.ema - ema.prevEma) : 0.0;

        // Project forward by prediction_horizon cycles
        double rsrpPredicted = ema.ema + slope * static_cast<double>(m_predictionHorizon);

        std::ostringstream msg;
        msg << "KPI-predict e2=" << enbE2Id
            << " cell=" << cellId
            << " samples=" << cs.n
            << " minRsrp=" << minRsrp << " dBm"
            << " ema=" << ema.ema << " dBm"
            << " slope=" << slope << " dBm/cycle"
            << " predicted=" << rsrpPredicted << " dBm"
            << " proactiveThr=" << m_proactiveThreshDbm << " dBm";

        // Proactive action: act if PREDICTED RSRP will drop below proactive_threshold,
        // but CURRENT RSRP is still above reactive_threshold (let CCO handle if already low)
        if (rsrpPredicted < m_proactiveThreshDbm && minRsrp > m_reactiveThreshDbm)
        {
            msg << " action=PROACTIVE_TXP_INCREASE stepDb=" << m_stepSizeDb;
            NS_LOG_INFO(msg.str());
            data->LogActionLm(m_name, msg.str());

            Ptr<OranCommandLte2LteTxPower> cmd = CreateObject<OranCommandLte2LteTxPower>();
            cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbE2Id));
            cmd->SetAttribute("PowerDeltaDb",   DoubleValue(m_stepSizeDb));
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
