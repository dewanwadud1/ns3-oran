/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-cmm-lte-2-lte-es-mro.h"

#include "oran-command-lte-2-lte-handover.h"
#include "oran-command-lte-2-lte-tx-power.h"
#include "oran-command.h"
#include "oran-data-repository.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <tuple>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranCmmLte2LteEsMro");
NS_OBJECT_ENSURE_REGISTERED(OranCmmLte2LteEsMro);

namespace
{

std::vector<double>
ZScore(const std::vector<double>& values)
{
    double sum = 0.0;
    for (double v : values)
    {
        sum += v;
    }
    const double mean = values.empty() ? 0.0 : sum / values.size();

    double ss = 0.0;
    for (double v : values)
    {
        const double d = v - mean;
        ss += d * d;
    }
    const double stddev = values.size() > 1 ? std::sqrt(ss / (values.size() - 1)) : 0.0;

    std::vector<double> out(values.size(), 0.0);
    if (stddev < 1e-9)
    {
        return out;
    }
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        out[i] = (values[i] - mean) / stddev;
    }
    return out;
}

double
DistanceFromThreshold(double utility, double threshold)
{
    return utility >= threshold ? 0.0 : threshold - utility;
}

} // namespace

TypeId
OranCmmLte2LteEsMro::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranCmmLte2LteEsMro")
            .SetParent<OranCmm>()
            .AddConstructor<OranCmmLte2LteEsMro>()
            .AddAttribute("MitigationMethod",
                          "TxPower conflict method: none, cancel, dampen, priority, nswf, eg, qacm.",
                          StringValue("qacm"),
                          MakeStringAccessor(&OranCmmLte2LteEsMro::m_method),
                          MakeStringChecker())
            .AddAttribute("SweepPoints",
                          "Number of TxPower-fraction points for game-theoretic methods.",
                          UintegerValue(51),
                          MakeUintegerAccessor(&OranCmmLte2LteEsMro::m_sweepPoints),
                          MakeUintegerChecker<uint32_t>(3))
            .AddAttribute("EsPriority",
                          "Priority weight for the Energy Saving xApp.",
                          DoubleValue(0.70),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_esPriority),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MroPriority",
                          "Priority weight for the MRO xApp.",
                          DoubleValue(1.00),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_mroPriority),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("UtilitySigma",
                          "Gaussian utility width over the normalized TxPower-fraction domain.",
                          DoubleValue(0.25),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_utilitySigma),
                          MakeDoubleChecker<double>(1e-6))
            .AddAttribute("QosThreshold",
                          "Normalized utility threshold used by QACM.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_qosThreshold),
                          MakeDoubleChecker<double>())
            .AddAttribute("QacmZeta",
                          "Weighted-distance scale used by the QACM objective.",
                          DoubleValue(1000.0),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_qacmZeta),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("HandoverHoldoffSec",
                          "Minimum time between forwarded LTE handover commands per UE.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranCmmLte2LteEsMro::m_handoverHoldoffSec),
                          MakeDoubleChecker<double>(0.0));

    return tid;
}

OranCmmLte2LteEsMro::OranCmmLte2LteEsMro()
    : OranCmm()
{
    NS_LOG_FUNCTION(this);
    m_name = "CmmLte2LteEsMro";
}

OranCmmLte2LteEsMro::~OranCmmLte2LteEsMro()
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranCmmLte2LteEsMro::Filter(
    std::map<std::tuple<std::string, bool>, std::vector<Ptr<OranCommand>>> inputCommands)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run ES/MRO CMM with NULL Near-RT RIC");

    std::vector<Ptr<OranCommand>> commands;
    for (const auto& commandSet : inputCommands)
    {
        for (auto command : commandSet.second)
        {
            Ptr<OranCommandLte2LteHandover> handover =
                command->GetObject<OranCommandLte2LteHandover>();
            if (handover != nullptr)
            {
                if (ShouldForwardHandover(command))
                {
                    commands.push_back(command);
                }
                continue;
            }

            Ptr<OranCommandLte2LteTxPower> txPower =
                command->GetObject<OranCommandLte2LteTxPower>();
            if (!m_active || txPower == nullptr || txPower->GetPowerDeltaDb() >= 0.0)
            {
                commands.push_back(command);
                continue;
            }

            const double requestedDeltaDb = txPower->GetPowerDeltaDb();
            const double fraction = SelectFraction(requestedDeltaDb);
            const double mitigatedDeltaDb = requestedDeltaDb * fraction;

            std::ostringstream msg;
            msg << "ES/MRO mitigation method=" << m_method << " targetE2="
                << command->GetTargetE2NodeId() << " requestedDeltaDb=" << requestedDeltaDb
                << " fraction=" << fraction << " mitigatedDeltaDb=" << mitigatedDeltaDb;
            LogLogicToStorage(msg.str());
            NS_LOG_INFO(msg.str());

            if (std::abs(mitigatedDeltaDb) < 1e-9)
            {
                continue;
            }

            Ptr<OranCommandLte2LteTxPower> mitigated =
                CreateObject<OranCommandLte2LteTxPower>();
            mitigated->SetAttribute("TargetE2NodeId",
                                    UintegerValue(command->GetTargetE2NodeId()));
            mitigated->SetAttribute("PowerDeltaDb", DoubleValue(mitigatedDeltaDb));
            commands.push_back(mitigated);
        }
    }

    return commands;
}

bool
OranCmmLte2LteEsMro::ShouldForwardHandover(Ptr<OranCommand> command)
{
    if (!m_active)
    {
        return true;
    }

    Ptr<OranCommandLte2LteHandover> handover =
        command->GetObject<OranCommandLte2LteHandover>();
    if (handover == nullptr)
    {
        return true;
    }

    bool found = false;
    uint16_t servingCellId = 0;
    std::tie(found, servingCellId) =
        m_nearRtRic->Data()->GetLteEnbCellInfo(command->GetTargetE2NodeId());
    if (!found)
    {
        LogLogicToStorage("Dropping handover command because serving eNB cell info is unknown");
        return false;
    }

    const uint64_t ueE2NodeId =
        m_nearRtRic->Data()->GetLteUeE2NodeIdFromCellInfo(servingCellId,
                                                          handover->GetTargetRnti());
    if (ueE2NodeId == 0)
    {
        LogLogicToStorage("Dropping handover command because UE E2 node is unknown");
        return false;
    }

    const double now = Simulator::Now().GetSeconds();
    auto it = m_lastHandoverByUe.find(ueE2NodeId);
    if (it != m_lastHandoverByUe.end() && (now - it->second) < m_handoverHoldoffSec)
    {
        std::ostringstream msg;
        msg << "Dropping handover for UE E2=" << ueE2NodeId
            << " during CMM holdoff; age=" << (now - it->second) << "s";
        LogLogicToStorage(msg.str());
        NS_LOG_INFO(msg.str());
        return false;
    }

    m_lastHandoverByUe[ueE2NodeId] = now;
    return true;
}

double
OranCmmLte2LteEsMro::SelectFraction(double requestedDeltaDb) const
{
    if (requestedDeltaDb >= 0.0 || m_method == "none")
    {
        return 1.0;
    }
    if (m_method == "cancel")
    {
        return 0.0;
    }
    if (m_method == "dampen")
    {
        return 0.5;
    }
    if (m_method == "priority")
    {
        return m_mroPriority >= m_esPriority ? 0.0 : 1.0;
    }
    if (m_method == "nswf" || m_method == "eg" || m_method == "qacm")
    {
        return SolveGame(m_method);
    }

    NS_LOG_WARN("Unknown ES/MRO mitigation method '" << m_method << "'; passing command through.");
    return 1.0;
}

double
OranCmmLte2LteEsMro::SolveGame(const std::string& method) const
{
    const uint32_t n = std::max<uint32_t>(m_sweepPoints, 3);
    const double wSum = std::max(1e-9, m_esPriority + m_mroPriority);
    const double wEs = m_esPriority / wSum;
    const double wMro = m_mroPriority / wSum;

    std::vector<double> p(n);
    std::vector<double> esRaw(n);
    std::vector<double> mroRaw(n);

    for (uint32_t i = 0; i < n; ++i)
    {
        p[i] = static_cast<double>(i) / static_cast<double>(n - 1);
        esRaw[i] = m_esPriority *
                   std::exp(-std::pow(p[i] - 1.0, 2.0) / (2.0 * m_utilitySigma * m_utilitySigma));
        mroRaw[i] = m_mroPriority *
                    std::exp(-std::pow(p[i], 2.0) / (2.0 * m_utilitySigma * m_utilitySigma));
    }

    const std::vector<double> es = ZScore(esRaw);
    const std::vector<double> mro = ZScore(mroRaw);

    double bestScore = method == "qacm" ? std::numeric_limits<double>::infinity()
                                        : -std::numeric_limits<double>::infinity();
    uint32_t bestIdx = 0;

    for (uint32_t i = 0; i < n; ++i)
    {
        double score = 0.0;
        if (method == "nswf")
        {
            const double esPos = es[i] - *std::min_element(es.begin(), es.end()) + 1e-6;
            const double mroPos = mro[i] - *std::min_element(mro.begin(), mro.end()) + 1e-6;
            score = esPos * mroPos;
            if (score > bestScore)
            {
                bestScore = score;
                bestIdx = i;
            }
        }
        else if (method == "eg")
        {
            score = wEs * es[i] + wMro * mro[i];
            if (score > bestScore)
            {
                bestScore = score;
                bestIdx = i;
            }
        }
        else
        {
            const double esDist = DistanceFromThreshold(es[i], m_qosThreshold);
            const double mroDist = DistanceFromThreshold(mro[i], m_qosThreshold);
            const double esSat = esDist == 0.0 ? 1.0 : 0.0;
            const double mroSat = mroDist == 0.0 ? 1.0 : 0.0;
            score = (wEs * esDist + wMro * mroDist) * m_qacmZeta -
                    std::pow(esSat + mroSat, 2.0);
            if (score < bestScore)
            {
                bestScore = score;
                bestIdx = i;
            }
        }
    }

    return p[bestIdx];
}

} // namespace ns3
