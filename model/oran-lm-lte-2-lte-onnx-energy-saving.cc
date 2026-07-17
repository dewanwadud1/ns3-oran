/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * DQN Energy-Saving Logic Module, using an ONNX model for inference.
 * See oran-lm-lte-2-lte-onnx-energy-saving.h for the state/action layout,
 * and workspace/ml/es_dqn/train_es_dqn.py for how the model was trained.
 */

#include "oran-lm-lte-2-lte-onnx-energy-saving.h"

#include "oran-command-lte-2-lte-tx-power.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteOnnxEnergySaving");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteOnnxEnergySaving);

TypeId
OranLmLte2LteOnnxEnergySaving::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmLte2LteOnnxEnergySaving")
            .SetParent<OranLm>()
            .AddConstructor<OranLmLte2LteOnnxEnergySaving>()
            .AddAttribute("OnnxModelPath",
                          "The file path of the trained ES DQN ONNX model.",
                          StringValue("es_dqn.onnx"),
                          MakeStringAccessor(&OranLmLte2LteOnnxEnergySaving::SetOnnxModelPath),
                          MakeStringChecker())
            .AddAttribute("NominalTxPowerDbm",
                          "The 'awake' TxPower (dBm); 'asleep' eNBs are dropped to 0 dBm. "
                          "Should match the scenario's initial --tx-power.",
                          DoubleValue(35.0),
                          MakeDoubleAccessor(&OranLmLte2LteOnnxEnergySaving::m_nominalTxPowerDbm),
                          MakeDoubleChecker<double>(0.0, 70.0));

    return tid;
}

OranLmLte2LteOnnxEnergySaving::OranLmLte2LteOnnxEnergySaving()
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmLte2LteOnnxEnergySaving";
}

OranLmLte2LteOnnxEnergySaving::~OranLmLte2LteOnnxEnergySaving()
{
    NS_LOG_FUNCTION(this);
}

void
OranLmLte2LteOnnxEnergySaving::SetOnnxModelPath(const std::string& onnxModelPath)
{
    std::ifstream f(onnxModelPath.c_str());
    NS_ABORT_MSG_IF(!f.good(),
                    "ONNX model file \""
                        << onnxModelPath << "\" not found."
                        << " Sample model \"es_dqn.onnx\" can be copied from"
                        << " contrib/oran/examples/ to the working directory.");
    f.close();

    m_session = Ort::Session(m_env, onnxModelPath.c_str(), Ort::SessionOptions{});
}

std::vector<OranLmLte2LteOnnxEnergySaving::EnbRsrpStats>
OranLmLte2LteOnnxEnergySaving::GetEnbRsrpStats(Ptr<OranDataRepository> data,
                                               const std::vector<uint64_t>& enbIds) const
{
    std::map<uint16_t, std::size_t> cellIdToIdx;
    for (std::size_t i = 0; i < enbIds.size(); ++i)
    {
        bool found;
        uint16_t cellId;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbIds[i]);
        if (found)
        {
            cellIdToIdx[cellId] = i;
        }
    }

    std::vector<std::vector<double>> samples(enbIds.size());
    std::size_t totalUes = 0;

    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        totalUes++;
        for (const auto& tup : data->GetLteUeRsrpRsrq(ueId))
        {
            uint16_t cellId = std::get<1>(tup);
            double rsrp = std::get<2>(tup);
            auto it = cellIdToIdx.find(cellId);
            if (it != cellIdToIdx.end())
            {
                samples[it->second].push_back(rsrp);
            }
        }
    }

    // Fallback matches the empty-measurement case in train_es_dqn.py's
    // build_state (RMIN_DBM = -95.0), so an eNB with no visible UEs yet
    // doesn't look like a strong-signal cell to the model.
    constexpr double kNoDataRsrp = -95.0;

    std::vector<EnbRsrpStats> stats(enbIds.size());
    for (std::size_t i = 0; i < enbIds.size(); ++i)
    {
        const auto& v = samples[i];
        if (v.empty())
        {
            stats[i] = EnbRsrpStats{kNoDataRsrp, 0.0, kNoDataRsrp, kNoDataRsrp, 0.0};
            continue;
        }
        double sum = 0.0;
        double mx = v.front();
        double mn = v.front();
        for (double x : v)
        {
            sum += x;
            mx = std::max(mx, x);
            mn = std::min(mn, x);
        }
        double mean = sum / v.size();
        double sq = 0.0;
        for (double x : v)
        {
            sq += (x - mean) * (x - mean);
        }
        double sd = std::sqrt(sq / v.size());
        double reachableFrac = totalUes > 0 ? static_cast<double>(v.size()) / totalUes : 0.0;
        stats[i] = EnbRsrpStats{mean, sd, mx, mn, reachableFrac};
    }
    return stats;
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteOnnxEnergySaving::Run()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmLte2LteOnnxEnergySaving: no Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    std::vector<uint64_t> enbIds = data->GetLteEnbE2NodeIds();
    std::sort(enbIds.begin(), enbIds.end());

    const std::size_t numRus = enbIds.size();
    NS_ABORT_MSG_IF(numRus != 3,
                    "OranLmLte2LteOnnxEnergySaving: the ONNX model was trained for exactly"
                        << " 3 eNBs (see workspace/ml/es_dqn/train_es_dqn.py), got " << numRus);

    std::vector<EnbRsrpStats> stats = GetEnbRsrpStats(data, enbIds);

    std::vector<float> input(6 * numRus + 1);
    for (std::size_t i = 0; i < numRus; ++i)
    {
        bool awake = !m_enbAwake.count(enbIds[i]) || m_enbAwake[enbIds[i]];
        input[i * 6 + 0] = static_cast<float>(stats[i].mean / 100.0);
        input[i * 6 + 1] = static_cast<float>(stats[i].stdDev / 20.0);
        input[i * 6 + 2] = static_cast<float>(stats[i].max / 100.0);
        input[i * 6 + 3] = static_cast<float>(stats[i].min / 100.0);
        input[i * 6 + 4] = static_cast<float>(stats[i].reachableFrac);
        input[i * 6 + 5] = awake ? 1.0f : 0.0f;
    }
    // t/T: training varied this over a bounded 200-slot episode, but a
    // continuously-running LM has no equivalent bounded horizon, and the
    // learned policy's decisions were empirically load-driven rather than
    // schedule-driven (see workspace/ml/es_dqn/eval_policy.py) -- so a fixed
    // mid-point placeholder is used here at inference time.
    input[6 * numRus] = 0.5f;

    std::array<int64_t, 2> inputShape{1, static_cast<int64_t>(input.size())};
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(m_memoryInfo,
                                                             input.data(),
                                                             input.size(),
                                                             inputShape.data(),
                                                             inputShape.size());

    const auto inputName = m_session.GetInputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> inputNames{inputName.get()};
    const auto outputName = m_session.GetOutputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> outputNames{outputName.get()};

    const auto output = m_session.Run(Ort::RunOptions{},
                                      inputNames.data(),
                                      &inputTensor,
                                      1UL,
                                      outputNames.data(),
                                      1);
    const float* q = output[0].GetTensorData<float>();

    // q layout: [Q_off(1..numRus), Q_on(1..numRus)] -- see DQN.forward() in
    // train_es_dqn.py.
    std::vector<bool> wantAwake(numRus);
    std::size_t bestMarginIdx = 0;
    float bestMargin = -std::numeric_limits<float>::infinity();
    bool anyAwake = false;
    for (std::size_t i = 0; i < numRus; ++i)
    {
        float margin = q[numRus + i] - q[i]; // Q_on - Q_off
        wantAwake[i] = margin > 0.0f;
        anyAwake = anyAwake || wantAwake[i];
        if (margin > bestMargin)
        {
            bestMargin = margin;
            bestMarginIdx = i;
        }
    }
    if (!anyAwake)
    {
        // Matches the training-time safeguard: never let every eNB sleep at once.
        wantAwake[bestMarginIdx] = true;
    }

    for (std::size_t i = 0; i < numRus; ++i)
    {
        uint64_t enbId = enbIds[i];
        bool currentlyAwake = !m_enbAwake.count(enbId) || m_enbAwake[enbId];
        if (wantAwake[i] == currentlyAwake)
        {
            continue;
        }

        Ptr<OranCommandLte2LteTxPower> cmd = CreateObject<OranCommandLte2LteTxPower>();
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
        cmd->SetAttribute(
            "PowerDeltaDb",
            DoubleValue(wantAwake[i] ? m_nominalTxPowerDbm : -m_nominalTxPowerDbm));
        data->LogCommandLm(m_name, cmd);
        commands.push_back(cmd);
        m_enbAwake[enbId] = wantAwake[i];

        NS_LOG_INFO("eNB " << enbId << (wantAwake[i] ? ": waking" : ": sleeping") << " (Δ="
                    << (wantAwake[i] ? "+" : "-") << m_nominalTxPowerDbm << " dB)");
    }

    return commands;
}

} // namespace ns3
