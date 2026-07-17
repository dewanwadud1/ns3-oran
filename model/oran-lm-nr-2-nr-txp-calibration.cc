/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Diagnostic-only TxPower square-wave calibration LM. See the header for
 * the full rationale (measuring parameter-change-to-KPI-effect lag with an
 * unambiguous, large-amplitude signal instead of trying to extract the lag
 * from real xApps' small, noisy increments).
 */

#include "oran-lm-nr-2-nr-txp-calibration.h"

#include "oran-command-nr-2-nr-tx-power.h"
#include "oran-data-repository.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrTxpCalibration");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrTxpCalibration);

TypeId
OranLmNr2NrTxpCalibration::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrTxpCalibration")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrTxpCalibration>()
            .AddAttribute("LowDbm",
                          "Low extreme of the TxPower square wave (dBm).",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&OranLmNr2NrTxpCalibration::m_lowDbm),
                          MakeDoubleChecker<double>(0.0, 50.0))
            .AddAttribute("HighDbm",
                          "High extreme of the TxPower square wave (dBm).",
                          DoubleValue(45.0),
                          MakeDoubleAccessor(&OranLmNr2NrTxpCalibration::m_highDbm),
                          MakeDoubleChecker<double>(0.0, 50.0))
            .AddAttribute("PeriodCycles",
                          "Number of LM cycles spent at each extreme before toggling.",
                          UintegerValue(6),
                          MakeUintegerAccessor(&OranLmNr2NrTxpCalibration::m_periodCycles),
                          MakeUintegerChecker<uint32_t>(1));

    return tid;
}

OranLmNr2NrTxpCalibration::OranLmNr2NrTxpCalibration()
    : m_lowDbm(10.0),
      m_highDbm(45.0),
      m_periodCycles(6)
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmNr2NrTxpCalibration";
}

OranLmNr2NrTxpCalibration::~OranLmNr2NrTxpCalibration()
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrTxpCalibration::Run()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmNr2NrTxpCalibration: no Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();

    for (auto enbId : data->GetNrGnbE2NodeIds())
    {
        if (!m_isHigh.count(enbId))
        {
            // First time seeing this gNB: start the count, assume it's
            // sitting at the scenario's initial TxPower (m_lowDbm is just a
            // bookkeeping placeholder here -- the very first toggle's delta
            // may not land exactly on m_highDbm as a result, so discard the
            // first transition when analyzing lag; every later toggle is
            // exact since we track our own prior commands).
            m_isHigh[enbId] = false;
            m_cycleCount[enbId] = 0;
            m_believedDbm[enbId] = m_lowDbm;
            continue;
        }

        m_cycleCount[enbId]++;
        if (m_cycleCount[enbId] < m_periodCycles)
        {
            continue;
        }

        m_cycleCount[enbId] = 0;
        bool goHigh = !m_isHigh[enbId];
        double targetDbm = goHigh ? m_highDbm : m_lowDbm;
        double delta = targetDbm - m_believedDbm[enbId];

        Ptr<OranCommandNr2NrTxPower> cmd = CreateObject<OranCommandNr2NrTxPower>();
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
        cmd->SetAttribute("PowerDeltaDb", DoubleValue(delta));
        data->LogCommandLm(m_name, cmd);
        commands.push_back(cmd);

        m_isHigh[enbId] = goHigh;
        m_believedDbm[enbId] = targetDbm;

        NS_LOG_INFO("gNB " << enbId << ": TxPower toggled to " << targetDbm
                    << " dBm (delta=" << delta << ")");
    }

    return commands;
}

} // namespace ns3
