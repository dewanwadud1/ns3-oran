/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * DQN Mobility Load Balancing Logic Module, using an ONNX model for
 * inference. See oran-lm-lte-2-lte-onnx-mlb.h for the state/action layout
 * and CIO-aggregation rationale, and workspace/ml/mlb_dqn/train_mlb.py for
 * how the model was trained.
 */

#include "oran-lm-lte-2-lte-onnx-mlb.h"

#include "oran-command-lte-2-lte-cell-parameter.h"
#include "oran-lte-cell-control-state.h"
#include "oran-near-rt-ric.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmLte2LteOnnxMlb");
NS_OBJECT_ENSURE_REGISTERED(OranLmLte2LteOnnxMlb);

TypeId
OranLmLte2LteOnnxMlb::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmLte2LteOnnxMlb")
            .SetParent<OranLm>()
            .AddConstructor<OranLmLte2LteOnnxMlb>()
            .AddAttribute("DqnPath",
                          "ONNX path for the load-balancing DQN.",
                          StringValue("mlb_dqn.onnx"),
                          MakeStringAccessor(&OranLmLte2LteOnnxMlb::SetDqnPath),
                          MakeStringChecker())
            .AddAttribute("CioStep",
                          "CIO adjustment step in dB.",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&OranLmLte2LteOnnxMlb::m_cioStepDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("MaxAbsCio",
                          "Absolute CIO clamp in dB.",
                          DoubleValue(6.0),
                          MakeDoubleAccessor(&OranLmLte2LteOnnxMlb::m_maxAbsCioDb),
                          MakeDoubleChecker<double>(0.0))
            .AddAttribute("VoteImbalanceThreshold",
                          "Fractional (votes-load)/totalUes mismatch, per cell, that "
                          "triggers a CIO nudge.",
                          DoubleValue(0.15),
                          MakeDoubleAccessor(&OranLmLte2LteOnnxMlb::m_voteImbalanceThreshold),
                          MakeDoubleChecker<double>(0.0));

    return tid;
}

OranLmLte2LteOnnxMlb::OranLmLte2LteOnnxMlb()
    : m_cioStepDb(1.0),
      m_maxAbsCioDb(6.0),
      m_voteImbalanceThreshold(0.15)
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmLte2LteOnnxMlb";
}

OranLmLte2LteOnnxMlb::~OranLmLte2LteOnnxMlb()
{
    NS_LOG_FUNCTION(this);
}

void
OranLmLte2LteOnnxMlb::SetDqnPath(const std::string& path)
{
    std::ifstream f(path.c_str());
    NS_ABORT_MSG_IF(!f.good(),
                    "ONNX model file \""
                        << path << "\" not found."
                        << " Sample model \"mlb_dqn.onnx\" can be copied from"
                        << " contrib/oran/examples/ to the working directory.");
    f.close();

    m_dqnSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

double
OranLmLte2LteOnnxMlb::SyntheticDemand(uint64_t ueE2NodeId)
{
    // Stable per-UE value in [1, 10] Mbps -- real per-user traffic demand
    // isn't tracked by the data repository, so (matching how training in
    // workspace/ml/mlb_dqn/train_mlb.py handled the same gap) a fixed value
    // derived from the E2 node ID is used instead of a per-Run() random draw.
    std::size_t h = std::hash<uint64_t>{}(ueE2NodeId);
    return 1.0 + static_cast<double>(h % 9001) / 1000.0;
}

std::vector<Ptr<OranCommand>>
OranLmLte2LteOnnxMlb::Run()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmLte2LteOnnxMlb: no Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    std::vector<uint64_t> enbIds = data->GetLteEnbE2NodeIds();
    std::sort(enbIds.begin(), enbIds.end());

    const std::size_t numRus = enbIds.size();
    NS_ABORT_MSG_IF(numRus != 3,
                    "OranLmLte2LteOnnxMlb: the ONNX model was trained for exactly 3 eNBs"
                        << " (see workspace/ml/mlb_dqn/train_mlb.py), got " << numRus);

    std::map<uint16_t, uint64_t> cellIdToEnbId;
    for (auto enbId : enbIds)
    {
        bool found;
        uint16_t cellId;
        std::tie(found, cellId) = data->GetLteEnbCellInfo(enbId);
        if (found)
        {
            NS_ABORT_MSG_IF(cellId < 1 || cellId > numRus,
                            "OranLmLte2LteOnnxMlb: expected cellIds in [1,"
                                << numRus << "], got " << cellId);
            cellIdToEnbId[cellId] = enbId;
        }
    }
    if (cellIdToEnbId.size() != numRus)
    {
        return commands; // not all eNBs attached yet
    }

    // Pass 1: current per-cell load, same convention as the rule-based MLB.
    std::vector<uint32_t> currentLoad(numRus, 0);
    std::map<uint64_t, uint16_t> ueServingCell;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        bool found;
        uint16_t cellId;
        uint16_t rnti;
        std::tie(found, cellId, rnti) = data->GetLteUeCellInfo(ueId);
        if (found && cellId >= 1 && cellId <= numRus)
        {
            currentLoad[cellId - 1]++;
            ueServingCell[ueId] = cellId;
        }
    }
    const double totalUes = static_cast<double>(ueServingCell.size());
    if (totalUes <= 0.0)
    {
        return commands;
    }

    std::vector<double> loadFrac(numRus);
    for (std::size_t i = 0; i < numRus; ++i)
    {
        loadFrac[i] = currentLoad[i] / totalUes;
    }

    // Pass 2: per-UE DQN vote for its best cell.
    std::vector<double> votes(numRus, 0.0);
    for (const auto& kv : ueServingCell)
    {
        uint64_t ueId = kv.first;

        std::vector<double> rsrp(numRus, -140.0);
        for (const auto& tup : data->GetLteUeRsrpRsrq(ueId))
        {
            uint16_t cellId = std::get<1>(tup);
            double rsrpVal = std::get<2>(tup);
            if (cellId >= 1 && cellId <= numRus)
            {
                rsrp[cellId - 1] = rsrpVal;
            }
        }

        double demand = SyntheticDemand(ueId);

        std::vector<float> state(2 * numRus + 2);
        for (std::size_t i = 0; i < numRus; ++i)
        {
            state[i] = static_cast<float>(rsrp[i] / 100.0);
            state[numRus + i] = static_cast<float>(loadFrac[i]);
        }
        state[2 * numRus] = static_cast<float>(demand / 10.0);
        // t/T: no bounded episode horizon at inference time -- same
        // fixed-midpoint rationale as OranLmLte2LteOnnxEnergySaving.
        state[2 * numRus + 1] = 0.5f;

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

        std::size_t best = 0;
        float bestQ = q[0];
        for (std::size_t i = 1; i < numRus; ++i)
        {
            if (q[i] > bestQ)
            {
                bestQ = q[i];
                best = i;
            }
        }
        votes[best] += 1.0;
    }

    // Translate per-cell vote/load mismatch into a CIO nudge -- cells the
    // DQN wants to gain load get more attractive (CIO up), cells it wants
    // to shed load get less attractive (CIO down). Keeps the actuator type
    // the same as the rule-based MLB so CDC keeps recognizing this as MLB.
    for (std::size_t i = 0; i < numRus; ++i)
    {
        double normalizedVoteError = (votes[i] - currentLoad[i]) / totalUes;
        if (std::abs(normalizedVoteError) < m_voteImbalanceThreshold)
        {
            continue;
        }

        uint64_t enbId = cellIdToEnbId[static_cast<uint16_t>(i + 1)];
        OranLteCellControlParams params = GetLteCellControlParameters(enbId);
        double direction = normalizedVoteError > 0.0 ? 1.0 : -1.0;
        double newCio = std::max(-m_maxAbsCioDb,
                                 std::min(m_maxAbsCioDb, params.cioDb + direction * m_cioStepDb));

        Ptr<OranCommandLte2LteCellParameter> cmd = CreateObject<OranCommandLte2LteCellParameter>();
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(enbId));
        cmd->SetAttribute("ParameterName", StringValue("CIO"));
        cmd->SetAttribute("Value", DoubleValue(newCio));
        cmd->SetAttribute("IsDelta", BooleanValue(false));
        data->LogCommandLm(m_name, cmd);
        commands.push_back(cmd);

        NS_LOG_INFO("MLB cellId=" << (i + 1) << " votes=" << votes[i] << " load="
                    << currentLoad[i] << " totalUes=" << totalUes << " newCio=" << newCio);
    }

    return commands;
}

} // namespace ns3
