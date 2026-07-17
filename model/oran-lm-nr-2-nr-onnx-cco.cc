/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * DQN Coverage & Capacity Optimization Logic Module for NR, using an ONNX
 * model for inference. See oran-lm-nr-2-nr-onnx-cco.h for the state/action
 * layout, and workspace/ml/cco_dqn/train_cco.py for how the model was
 * trained (calibrated against workspace/ml/real_data/cco_nr_ptx_sweep_*.csv).
 */

#include "oran-lm-nr-2-nr-onnx-cco.h"

#include "oran-command-nr-2-nr-tx-power.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrOnnxCco");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrOnnxCco);

namespace
{
// Matches PTX_LEVELS in workspace/ml/cco_dqn/train_cco.py's NR variant --
// kept inside the NR gNB TxPower clamp of [0,50] dBm (see
// OranE2NodeTerminatorNrGnb::ReceiveCommand), bracketing the scenario's
// 35 dBm default.
constexpr double kPtxMinDbm = 20.0;
constexpr double kPtxMaxDbm = 45.0;
constexpr std::size_t kNumPtxLevels = 10;

double
PtxLevel(std::size_t idx)
{
    return kPtxMinDbm + static_cast<double>(idx) * (kPtxMaxDbm - kPtxMinDbm) /
                            static_cast<double>(kNumPtxLevels - 1);
}
} // namespace

TypeId
OranLmNr2NrOnnxCco::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrOnnxCco")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrOnnxCco>()
            .AddAttribute("DqnPath",
                          "ONNX path for the CCO PTX-selection DQN.",
                          StringValue("cco_dqn_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxCco::SetDqnPath),
                          MakeStringChecker())
            .AddAttribute("NominalTxPowerDbm",
                          "Believed initial PTX (dBm) for gNBs not seen yet. Should "
                          "match the scenario's initial --tx-power.",
                          DoubleValue(35.0),
                          MakeDoubleAccessor(&OranLmNr2NrOnnxCco::m_nominalTxPowerDbm),
                          MakeDoubleChecker<double>(0.0, 50.0));

    return tid;
}

OranLmNr2NrOnnxCco::OranLmNr2NrOnnxCco()
    : m_nominalTxPowerDbm(35.0)
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmNr2NrOnnxCco";
}

OranLmNr2NrOnnxCco::~OranLmNr2NrOnnxCco()
{
    NS_LOG_FUNCTION(this);
}

void
OranLmNr2NrOnnxCco::SetDqnPath(const std::string& path)
{
    std::ifstream f(path.c_str());
    NS_ABORT_MSG_IF(!f.good(),
                    "ONNX model file \""
                        << path << "\" not found."
                        << " Sample model \"cco_dqn_nr.onnx\" can be copied from"
                        << " contrib/oran/examples/ to the working directory.");
    f.close();

    m_dqnSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

std::vector<OranLmNr2NrOnnxCco::CcoStats>
OranLmNr2NrOnnxCco::GetCcoStats(Ptr<OranDataRepository> data,
                                const std::vector<uint64_t>& enbIds) const
{
    std::map<uint16_t, std::size_t> cellIdToIdx;
    for (std::size_t i = 0; i < enbIds.size(); ++i)
    {
        bool found;
        uint16_t cellId;
        std::tie(found, cellId) = data->GetNrGnbCellInfo(enbIds[i]);
        if (found)
        {
            cellIdToIdx[cellId] = i;
        }
    }

    std::vector<std::vector<double>> servedRsrp(enbIds.size());
    std::vector<std::vector<double>> ifcRsrp(enbIds.size());

    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        bool found;
        uint16_t servingCellId;
        uint16_t rnti;
        std::tie(found, servingCellId, rnti) = data->GetNrUeCellInfo(ueId);
        if (!found)
        {
            continue;
        }
        auto servingIt = cellIdToIdx.find(servingCellId);
        if (servingIt == cellIdToIdx.end())
        {
            continue;
        }
        std::size_t n = servingIt->second;

        double ownRsrp = 0.0;
        bool haveOwn = false;
        std::vector<double> otherRsrp;
        for (const auto& tup : data->GetNrUeRsrpRsrq(ueId))
        {
            uint16_t cellId = std::get<1>(tup);
            double rsrp = std::get<2>(tup);
            if (cellId == servingCellId)
            {
                ownRsrp = rsrp;
                haveOwn = true;
            }
            else
            {
                otherRsrp.push_back(rsrp);
            }
        }
        if (haveOwn)
        {
            servedRsrp[n].push_back(ownRsrp);
        }
        if (!otherRsrp.empty())
        {
            double sum = 0.0;
            for (double r : otherRsrp)
            {
                sum += r;
            }
            ifcRsrp[n].push_back(sum / otherRsrp.size());
        }
    }

    // Fallback matches ES's empty-measurement convention (no visible UEs yet
    // shouldn't look artificially strong or weak to the model).
    constexpr double kNoDataRsrp = -95.0;
    constexpr double kNoDataIfc = -115.0;

    std::vector<CcoStats> stats(enbIds.size());
    for (std::size_t i = 0; i < enbIds.size(); ++i)
    {
        CcoStats& s = stats[i];
        s.load = static_cast<uint32_t>(servedRsrp[i].size());

        if (servedRsrp[i].empty())
        {
            s.meanRsrpServed = kNoDataRsrp;
            s.stdRsrpServed = 0.0;
        }
        else
        {
            double sum = 0.0;
            for (double r : servedRsrp[i])
            {
                sum += r;
            }
            s.meanRsrpServed = sum / servedRsrp[i].size();
            double sq = 0.0;
            for (double r : servedRsrp[i])
            {
                sq += (r - s.meanRsrpServed) * (r - s.meanRsrpServed);
            }
            s.stdRsrpServed = std::sqrt(sq / servedRsrp[i].size());
        }

        if (ifcRsrp[i].empty())
        {
            s.meanIfc = kNoDataIfc;
        }
        else
        {
            double sum = 0.0;
            for (double r : ifcRsrp[i])
            {
                sum += r;
            }
            s.meanIfc = sum / ifcRsrp[i].size();
        }
    }
    return stats;
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrOnnxCco::Run()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmNr2NrOnnxCco: no Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    std::vector<uint64_t> enbIds = data->GetNrGnbE2NodeIds();
    std::sort(enbIds.begin(), enbIds.end());

    const std::size_t numRus = enbIds.size();
    if (numRus == 0)
    {
        return commands;
    }

    std::vector<CcoStats> stats = GetCcoStats(data, enbIds);

    uint32_t totalUes = 0;
    for (const auto& s : stats)
    {
        totalUes += s.load;
    }
    if (totalUes == 0)
    {
        return commands;
    }
    const double avgLoad = static_cast<double>(totalUes) / numRus;

    for (std::size_t i = 0; i < numRus; ++i)
    {
        uint64_t enbId = enbIds[i];
        double currentPtx = m_currentPtxDbm.count(enbId) ? m_currentPtxDbm[enbId]
                                                          : m_nominalTxPowerDbm;

        std::array<float, 7> state{
            static_cast<float>(stats[i].meanRsrpServed / 100.0),
            static_cast<float>(stats[i].stdRsrpServed / 20.0),
            static_cast<float>(stats[i].meanIfc / 100.0),
            static_cast<float>(stats[i].load / avgLoad),
            static_cast<float>((kPtxMaxDbm - currentPtx) / 20.0),
            0.5f, // t/T: no bounded episode horizon at inference time, same
                  // fixed-midpoint rationale as OranLmNr2NrOnnxEnergySaving.
            static_cast<float>(i) / static_cast<float>(numRus),
        };

        std::array<int64_t, 2> shape{1, static_cast<int64_t>(state.size())};
        Ort::Value tensor = Ort::Value::CreateTensor<float>(m_memoryInfo,
                                                             state.data(),
                                                             state.size(),
                                                             shape.data(),
                                                             shape.size());
        const auto inputName = m_dqnSession.GetInputNameAllocated(0UL, m_allocator);
        std::array<const char*, 1> inputNames{inputName.get()};
        const auto outputName = m_dqnSession.GetOutputNameAllocated(0UL, m_allocator);
        std::array<const char*, 1> outputNames{outputName.get()};
        auto output = m_dqnSession.Run(Ort::RunOptions{},
                                       inputNames.data(),
                                       &tensor,
                                       1,
                                       outputNames.data(),
                                       1);
        const float* q = output[0].GetTensorData<float>();

        std::size_t bestIdx = 0;
        float bestQ = q[0];
        for (std::size_t k = 1; k < kNumPtxLevels; ++k)
        {
            if (q[k] > bestQ)
            {
                bestQ = q[k];
                bestIdx = k;
            }
        }

        double targetPtx = PtxLevel(bestIdx);
        double delta = targetPtx - currentPtx;
        if (std::abs(delta) < 0.01)
        {
            continue;
        }

        Ptr<OranCommandNr2NrTxPower> cmd = CreateObject<OranCommandNr2NrTxPower>();
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
        cmd->SetAttribute("PowerDeltaDb", DoubleValue(delta));
        data->LogCommandLm(m_name, cmd);
        commands.push_back(cmd);
        m_currentPtxDbm[enbId] = targetPtx;

        NS_LOG_INFO("gNB " << enbId << ": PTX " << currentPtx << " -> " << targetPtx
                    << " dBm (Δ=" << delta << ")");
    }

    return commands;
}

} // namespace ns3
