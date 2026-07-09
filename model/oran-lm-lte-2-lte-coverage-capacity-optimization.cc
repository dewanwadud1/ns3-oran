/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "oran-lm-lte-2-lte-coverage-capacity-optimization.h"

#include "oran-command-lte-2-lte-cell-parameter.h"
#include "oran-command-lte-2-lte-tx-power.h"
#include "oran-data-repository.h"
#include "oran-lte-cell-control-state.h"
#include "oran-near-rt-ric.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <map>
#include <sstream>
#include <tuple>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteCoverageCapacityOptimization");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteCoverageCapacityOptimization);

namespace
{

struct CellRsrpStats
{
    uint32_t samples         = 0;
    uint32_t lowSamples      = 0;
    uint32_t criticalSamples = 0;
    double   sumRsrpDbm      = 0.0;
};

} // namespace

TypeId
OranLmLte2LteCoverageCapacityOptimization::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmLte2LteCoverageCapacityOptimization")
            .SetParent<OranLm>()
            .AddConstructor<OranLmLte2LteCoverageCapacityOptimization>()
            .AddAttribute("LowRsrpThresholdDbm",
                          "Serving-cell RSRP threshold below which CCO raises TxPower.",
                          DoubleValue(-110.0),
                          MakeDoubleAccessor(
                              &OranLmLte2LteCoverageCapacityOptimization::m_lowRsrpThresholdDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("LowRsrpFractionThreshold",
                          "Fraction of low-RSRP samples required before acting.",
                          DoubleValue(0.10),
                          MakeDoubleAccessor(&OranLmLte2LteCoverageCapacityOptimization::
                                                 m_lowRsrpFractionThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("StepSize",
                          "TxPower increase requested by CCO in dB.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmLte2LteCoverageCapacityOptimization::m_stepSizeDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MinSamplesPerCell",
                          "Minimum serving-RSRP samples per cell before CCO acts.",
                          UintegerValue(1),
                          MakeUintegerAccessor(
                              &OranLmLte2LteCoverageCapacityOptimization::m_minSamplesPerCell),
                          MakeUintegerChecker<uint32_t>(1))
            // ── RET (antenna tilt) adjustment ────────────────────────────────
            // When coverage is critically poor, CCO also reduces tilt (increases
            // coverage radius). This issues an OranCommandLte2LteCellParameter
            // for "RET", creating an Indirect conflict with MRO: RET ∈ P2K[CDR,HSR]
            // and MRO manages CDR/HSR but MRO ∉ P2X[RET].
            .AddAttribute("CriticalRsrpThresholdDbm",
                          "RSRP below which a UE counts as critically covered; "
                          "exceeding CriticalFractionThreshold also issues a RET command.",
                          DoubleValue(-120.0),
                          MakeDoubleAccessor(&OranLmLte2LteCoverageCapacityOptimization::
                                                 m_criticalRsrpThresholdDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("CriticalFractionThreshold",
                          "Fraction of critically-low RSRP UEs required to trigger RET adjustment.",
                          DoubleValue(0.30),
                          MakeDoubleAccessor(&OranLmLte2LteCoverageCapacityOptimization::
                                                 m_criticalFractionThreshold),
                          MakeDoubleChecker<double>(0.0, 1.0))
            .AddAttribute("RetStepDeg",
                          "Antenna tilt change per CCO cycle (degrees). "
                          "Positive = more tilt (concentrates beam, improves SINR); negative = less tilt.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(
                              &OranLmLte2LteCoverageCapacityOptimization::m_retStepDeg),
                          MakeDoubleChecker<double>());
    return tid;
}

OranLmLte2LteCoverageCapacityOptimization::OranLmLte2LteCoverageCapacityOptimization()
    : OranLm(),
      m_lowRsrpThresholdDbm(-110.0),
      m_lowRsrpFractionThreshold(0.10),
      m_stepSizeDb(1.0),
      m_minSamplesPerCell(1),
      m_criticalRsrpThresholdDbm(-120.0),
      m_criticalFractionThreshold(0.30),
      m_retStepDeg(1.0)
{
    m_name = "OranLmLte2LteCoverageCapacityOptimization";
}

OranLmLte2LteCoverageCapacityOptimization::~OranLmLte2LteCoverageCapacityOptimization() = default;

std::vector<Ptr<OranCommand>>
OranLmLte2LteCoverageCapacityOptimization::Run()
{
    std::vector<Ptr<OranCommand>> commands;
    if (!m_active)
    {
        return commands;
    }

    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run CCO LM with NULL Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    std::map<uint16_t, uint64_t> cellToE2;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        bool found = false;
        uint16_t cellId = 0;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbId);
        if (found)
            cellToE2[cellId] = enbId;
    }

    std::map<uint16_t, CellRsrpStats> stats;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool found = false;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;
        std::tie(found, servingCellId, servingRnti) = data->GetLteUeCellInfo(ueId);
        if (!found)
            continue;

        for (const auto& meas : data->GetLteUeRsrpRsrq(ueId))
        {
            uint16_t rnti = 0, cellId = 0;
            double rsrp = 0.0, rsrq = 0.0;
            bool serving = false;
            uint8_t ccid = 0;
            std::tie(rnti, cellId, rsrp, rsrq, serving, ccid) = meas;
            if (cellId != servingCellId)
                continue;

            CellRsrpStats& s = stats[cellId];
            s.samples++;
            s.sumRsrpDbm += rsrp;
            if (rsrp < m_lowRsrpThresholdDbm)
                s.lowSamples++;
            if (rsrp < m_criticalRsrpThresholdDbm)
                s.criticalSamples++;
        }
    }

    for (const auto& item : stats)
    {
        const uint16_t cellId = item.first;
        const CellRsrpStats& s = item.second;
        if (s.samples < m_minSamplesPerCell || cellToE2.find(cellId) == cellToE2.end())
            continue;

        const uint64_t e2id       = cellToE2[cellId];
        const double lowFraction  = static_cast<double>(s.lowSamples) / s.samples;
        const double critFraction = static_cast<double>(s.criticalSamples) / s.samples;
        const double avgRsrp      = s.sumRsrpDbm / s.samples;

        // ── TxPower increase when low-RSRP fraction exceeds threshold ────────
        if (lowFraction >= m_lowRsrpFractionThreshold)
        {
            Ptr<OranCommandLte2LteTxPower> txCmd = CreateObject<OranCommandLte2LteTxPower>();
            txCmd->SetAttribute("TargetE2NodeId", UintegerValue(e2id));
            txCmd->SetAttribute("PowerDeltaDb",   DoubleValue(m_stepSizeDb));
            data->LogCommandLm(m_name, txCmd);
            commands.push_back(txCmd);

            std::ostringstream msg;
            msg << "CCO TXP cell=" << cellId
                << " lowFrac=" << lowFraction
                << " avgRsrp=" << avgRsrp
                << " thr=" << m_lowRsrpThresholdDbm
                << " delta=+" << m_stepSizeDb << "dB";
            data->LogActionLm(m_name, msg.str());
            NS_LOG_INFO(msg.str());
        }

        // ── RET adjustment when critically-low RSRP fraction exceeds threshold.
        // Positive RetStepDeg increases the logical tilt state used for conflict tracking.
        // Indirect conflict: RET ∈ P2K[CDR,HSR]; K2X[CDR]=MRO; MRO ∉ P2X[RET].
        if (critFraction >= m_criticalFractionThreshold && m_retStepDeg != 0.0)
        {
            // Read current RET to compute absolute new value
            OranLteCellControlParams cp = GetLteCellControlParameters(e2id);
            double newRet = cp.retDeg + m_retStepDeg;
            newRet = std::max(0.0, std::min(15.0, newRet)); // clamp [0,15] deg

            Ptr<OranCommandLte2LteCellParameter> retCmd =
                CreateObject<OranCommandLte2LteCellParameter>();
            retCmd->SetAttribute("TargetE2NodeId", UintegerValue(e2id));
            retCmd->SetAttribute("ParameterName",  StringValue("RET"));
            retCmd->SetAttribute("Value",          DoubleValue(newRet));
            retCmd->SetAttribute("IsDelta",        BooleanValue(false));
            data->LogCommandLm(m_name, retCmd);
            commands.push_back(retCmd);

            std::ostringstream msg;
            msg << "CCO RET cell=" << cellId
                << " critFrac=" << critFraction
                << " avgRsrp=" << avgRsrp
                << " critThr=" << m_criticalRsrpThresholdDbm
                << " ret=" << cp.retDeg << "->" << newRet << "deg";
            data->LogActionLm(m_name, msg.str());
            NS_LOG_INFO(msg.str());
        }
    }

    return commands;
}

} // namespace ns3
