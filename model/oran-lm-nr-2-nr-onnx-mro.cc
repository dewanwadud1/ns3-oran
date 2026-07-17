/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * DQN Mobility Robustness Optimisation (handover) Logic Module for NR, using
 * ONNX models for inference. See oran-lm-nr-2-nr-onnx-mro.h for the
 * pipeline, and workspace/ml/mro_dqn/train_mro.py for how the models were
 * trained.
 */

#include "oran-lm-nr-2-nr-onnx-mro.h"

#include "oran-command-nr-2-nr-handover.h"
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

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrOnnxMro");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrOnnxMro);

namespace
{
constexpr std::size_t kNumModes = 7; // PED, CYCLIST, CAR, BUS, TRAIN, DRONE, UAV
} // namespace

TypeId
OranLmNr2NrOnnxMro::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrOnnxMro")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrOnnxMro>()
            .AddAttribute("ModeClassifierPath",
                          "ONNX path for the mobility mode classifier.",
                          StringValue("mode_classifier_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxMro::SetModeClassifierPath),
                          MakeStringChecker())
            .AddAttribute("TrajXPath",
                          "ONNX path for the trajectory X regressor.",
                          StringValue("traj_x_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxMro::SetTrajXPath),
                          MakeStringChecker())
            .AddAttribute("TrajYPath",
                          "ONNX path for the trajectory Y regressor.",
                          StringValue("traj_y_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxMro::SetTrajYPath),
                          MakeStringChecker())
            .AddAttribute("RsrpPath",
                          "ONNX path for the per-gNB RSRP regressor.",
                          StringValue("rsrp_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxMro::SetRsrpPath),
                          MakeStringChecker())
            .AddAttribute("DqnPath",
                          "ONNX path for the handover-target DQN head.",
                          StringValue("mro_dqn_nr.onnx"),
                          MakeStringAccessor(&OranLmNr2NrOnnxMro::SetDqnPath),
                          MakeStringChecker())
            .AddAttribute("HandoverHoldoffSec",
                          "Minimum time between consecutive handover commands for the same "
                          "UE, to let NrGnbRrc's state machine settle (see the member "
                          "comment in the header for the crash this prevents).",
                          DoubleValue(1.5),
                          MakeDoubleAccessor(&OranLmNr2NrOnnxMro::m_handoverHoldoffSec),
                          MakeDoubleChecker<double>(0.0));

    return tid;
}

OranLmNr2NrOnnxMro::OranLmNr2NrOnnxMro()
    : m_handoverHoldoffSec(1.5)
{
    NS_LOG_FUNCTION(this);

    m_name = "OranLmNr2NrOnnxMro";
}

OranLmNr2NrOnnxMro::~OranLmNr2NrOnnxMro()
{
    NS_LOG_FUNCTION(this);
}

namespace
{
void
CheckOnnxFile(const std::string& path)
{
    std::ifstream f(path.c_str());
    NS_ABORT_MSG_IF(!f.good(),
                    "ONNX model file \"" << path << "\" not found."
                        << " Sample models can be copied from"
                        << " contrib/oran/examples/ to the working directory.");
}
} // namespace

void
OranLmNr2NrOnnxMro::SetModeClassifierPath(const std::string& path)
{
    CheckOnnxFile(path);
    m_modeSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

void
OranLmNr2NrOnnxMro::SetTrajXPath(const std::string& path)
{
    CheckOnnxFile(path);
    m_trajXSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

void
OranLmNr2NrOnnxMro::SetTrajYPath(const std::string& path)
{
    CheckOnnxFile(path);
    m_trajYSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

void
OranLmNr2NrOnnxMro::SetRsrpPath(const std::string& path)
{
    CheckOnnxFile(path);
    m_rsrpSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

void
OranLmNr2NrOnnxMro::SetDqnPath(const std::string& path)
{
    CheckOnnxFile(path);
    m_dqnSession = Ort::Session(m_env, path.c_str(), Ort::SessionOptions{});
}

void
OranLmNr2NrOnnxMro::SetChannelConditionModel(Ptr<ChannelConditionModel> ccm)
{
    m_channelConditionModel = ccm;
}

void
OranLmNr2NrOnnxMro::SetEnbMobility(uint64_t enbE2NodeId, Ptr<MobilityModel> mobility)
{
    m_enbMobility[enbE2NodeId] = mobility;
}

void
OranLmNr2NrOnnxMro::SetUeMobility(uint64_t ueE2NodeId, Ptr<MobilityModel> mobility)
{
    m_ueMobility[ueE2NodeId] = mobility;
}

OranLmNr2NrOnnxMro::Kinematics
OranLmNr2NrOnnxMro::GetKinematics(Ptr<OranDataRepository> data, uint64_t ueE2NodeId) const
{
    Kinematics k;

    // maxEntries defaults to 1 (latest-only) -- explicitly ask for enough
    // history to finite-difference through jerk (needs 4 samples).
    std::map<Time, Vector> history =
        data->GetNodePositions(ueE2NodeId, Seconds(0), Simulator::Now(), 8);
    if (history.size() < 4)
    {
        return k; // cold start: not enough samples yet, valid stays false
    }

    std::vector<std::pair<Time, Vector>> samples(history.begin(), history.end());
    const std::size_t n = samples.size();
    const auto& [t0, p0] = samples[n - 4];
    const auto& [t1, p1] = samples[n - 3];
    const auto& [t2, p2] = samples[n - 2];
    const auto& [t3, p3] = samples[n - 1];

    auto dt = [](Time a, Time b) { return std::max((b - a).GetSeconds(), 1e-3); };
    auto dist2d = [](const Vector& a, const Vector& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    };
    auto bearing = [](const Vector& a, const Vector& b) {
        return std::atan2(b.y - a.y, b.x - a.x);
    };
    auto wrapAngle = [](double a) {
        while (a > M_PI)
            a -= 2 * M_PI;
        while (a < -M_PI)
            a += 2 * M_PI;
        return a;
    };

    const double dt01 = dt(t0, t1);
    const double dt12 = dt(t1, t2);
    const double dt23 = dt(t2, t3);

    const double v1 = dist2d(p0, p1) / dt01;
    const double v2 = dist2d(p1, p2) / dt12;
    const double v3 = dist2d(p2, p3) / dt23;

    const double a2 = (v2 - v1) / dt12;
    const double a3 = (v3 - v2) / dt23;

    const double jerk = (a3 - a2) / dt23;

    const double bearing2 = bearing(p1, p2);
    const double bearing3 = bearing(p2, p3);
    const double bearingRate = wrapAngle(bearing3 - bearing2) / dt23;

    k.valid = true;
    k.velocity = v3;
    k.accel = a3;
    k.jerk = jerk;
    k.bearingRate = bearingRate;
    k.currentPos = p3;
    return k;
}

float
OranLmNr2NrOnnxMro::RunScalarSession(Ort::Session& session, const std::vector<float>& input) const
{
    std::array<int64_t, 2> shape{1, static_cast<int64_t>(input.size())};
    Ort::Value tensor = Ort::Value::CreateTensor<float>(
        m_memoryInfo, const_cast<float*>(input.data()), input.size(), shape.data(), shape.size());

    const auto inputName = session.GetInputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> inputNames{inputName.get()};
    const auto outputName = session.GetOutputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> outputNames{outputName.get()};

    auto output = session.Run(Ort::RunOptions{}, inputNames.data(), &tensor, 1, outputNames.data(), 1);
    return output[0].GetTensorData<float>()[0];
}

int64_t
OranLmNr2NrOnnxMro::RunModeClassifier(const Kinematics& k)
{
    std::vector<float> input{static_cast<float>(k.velocity),
                             static_cast<float>(k.accel),
                             static_cast<float>(k.jerk),
                             static_cast<float>(k.bearingRate)};
    std::array<int64_t, 2> shape{1, static_cast<int64_t>(input.size())};
    Ort::Value tensor = Ort::Value::CreateTensor<float>(
        m_memoryInfo, input.data(), input.size(), shape.data(), shape.size());

    const auto inputName = m_modeSession.GetInputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> inputNames{inputName.get()};
    // Request only "output_label" (index 0) -- the classifier's other output
    // (output_probability) is a seq(map(int64,float)), not needed here.
    const auto outputName = m_modeSession.GetOutputNameAllocated(0UL, m_allocator);
    std::array<const char*, 1> outputNames{outputName.get()};

    auto output =
        m_modeSession.Run(Ort::RunOptions{}, inputNames.data(), &tensor, 1, outputNames.data(), 1);
    return output[0].GetTensorData<int64_t>()[0];
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrOnnxMro::Run()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "OranLmNr2NrOnnxMro: no Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    std::vector<uint64_t> enbIds = data->GetNrGnbE2NodeIds();
    std::sort(enbIds.begin(), enbIds.end());

    const std::size_t numRus = enbIds.size();
    if (numRus == 0)
    {
        return commands;
    }

    // gNB positions and cellId<->enbId lookups (built once per Run()).
    std::vector<Vector> enbPos(numRus);
    std::map<uint16_t, uint64_t> cellIdToEnbId;
    for (std::size_t i = 0; i < numRus; ++i)
    {
        std::map<Time, Vector> hist =
            data->GetNodePositions(enbIds[i], Seconds(0), Simulator::Now());
        NS_ABORT_MSG_IF(hist.empty(), "OranLmNr2NrOnnxMro: no position for gNB " << enbIds[i]);
        enbPos[i] = hist.rbegin()->second;

        bool found;
        uint16_t cellId;
        std::tie(found, cellId) = data->GetNrGnbCellInfo(enbIds[i]);
        if (found)
        {
            cellIdToEnbId[cellId] = enbIds[i];
        }
    }

    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        Kinematics k = GetKinematics(data, ueId);
        if (!k.valid)
        {
            continue; // cold start: not enough position history yet
        }

        bool found;
        uint16_t servingCellId;
        uint16_t rnti;
        std::tie(found, servingCellId, rnti) = data->GetNrUeCellInfo(ueId);
        if (!found)
        {
            continue;
        }

        int64_t modeIdx = RunModeClassifier(k);
        modeIdx = std::clamp<int64_t>(modeIdx, 0, static_cast<int64_t>(kNumModes) - 1);

        std::vector<float> trajInput{static_cast<float>(k.velocity),
                                     static_cast<float>(k.accel),
                                     static_cast<float>(modeIdx),
                                     static_cast<float>(k.currentPos.x),
                                     static_cast<float>(k.currentPos.y),
                                     static_cast<float>(k.bearingRate)};
        float predX = RunScalarSession(m_trajXSession, trajInput);
        float predY = RunScalarSession(m_trajYSession, trajInput);

        std::vector<float> state(kNumModes + 2 + numRus, 0.0f);
        state[modeIdx] = 1.0f;
        state[kNumModes] = predX / 200.0f;
        state[kNumModes + 1] = predY / 200.0f;
        for (std::size_t i = 0; i < numRus; ++i)
        {
            double dist = std::sqrt(std::pow(k.currentPos.x - enbPos[i].x, 2) +
                                    std::pow(k.currentPos.y - enbPos[i].y, 2));

            // Real LOS/NLOS state from the same ChannelConditionModel driving the
            // actual downlink channel -- not a hidden variable to the regressor
            // anymore. Defaults to "LOS" (optimistic/neutral) only if the model
            // or mobility references were never wired in from the example.
            float isLos = 1.0f;
            auto enbMobIt = m_enbMobility.find(enbIds[i]);
            auto ueMobIt = m_ueMobility.find(ueId);
            if (m_channelConditionModel && enbMobIt != m_enbMobility.end() &&
                ueMobIt != m_ueMobility.end())
            {
                Ptr<ChannelCondition> cond =
                    m_channelConditionModel->GetChannelCondition(enbMobIt->second, ueMobIt->second);
                isLos = cond->IsLos() ? 1.0f : 0.0f;
            }

            std::vector<float> rsrpInput{static_cast<float>(k.velocity),
                                         static_cast<float>(k.accel),
                                         static_cast<float>(dist),
                                         static_cast<float>(modeIdx),
                                         isLos};
            float predRsrp = RunScalarSession(m_rsrpSession, rsrpInput);
            state[kNumModes + 2 + i] = predRsrp / 100.0f;
        }

        std::array<int64_t, 2> dqnShape{1, static_cast<int64_t>(state.size())};
        Ort::Value dqnTensor = Ort::Value::CreateTensor<float>(
            m_memoryInfo, state.data(), state.size(), dqnShape.data(), dqnShape.size());
        const auto dqnInputName = m_dqnSession.GetInputNameAllocated(0UL, m_allocator);
        std::array<const char*, 1> dqnInputNames{dqnInputName.get()};
        const auto dqnOutputName = m_dqnSession.GetOutputNameAllocated(0UL, m_allocator);
        std::array<const char*, 1> dqnOutputNames{dqnOutputName.get()};
        auto dqnOutput = m_dqnSession.Run(Ort::RunOptions{},
                                          dqnInputNames.data(),
                                          &dqnTensor,
                                          1,
                                          dqnOutputNames.data(),
                                          1);
        const float* q = dqnOutput[0].GetTensorData<float>();
        std::size_t targetIdx = 0;
        float bestQ = q[0];
        for (std::size_t i = 1; i < numRus; ++i)
        {
            if (q[i] > bestQ)
            {
                bestQ = q[i];
                targetIdx = i;
            }
        }

        bool foundTargetCell;
        uint16_t targetCellId;
        std::tie(foundTargetCell, targetCellId) = data->GetNrGnbCellInfo(enbIds[targetIdx]);
        if (!foundTargetCell || targetCellId == servingCellId)
        {
            continue; // no change needed
        }

        auto servingIt = cellIdToEnbId.find(servingCellId);
        if (servingIt == cellIdToEnbId.end())
        {
            continue; // serving cell isn't one of our known gNBs (shouldn't happen)
        }

        auto lastHoIt = m_lastHandoverTime.find(ueId);
        if (lastHoIt != m_lastHandoverTime.end() &&
            (Simulator::Now() - lastHoIt->second).GetSeconds() < m_handoverHoldoffSec)
        {
            // A handover issued within the last m_handoverHoldoffSec for this
            // UE may still be settling in NrGnbRrc's state machine; issuing
            // another one now crashes NrUeManager::PrepareHandover.
            continue;
        }

        Ptr<OranCommandNr2NrHandover> cmd = CreateObject<OranCommandNr2NrHandover>();
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(servingIt->second));
        cmd->SetAttribute("TargetRnti", UintegerValue(rnti));
        cmd->SetAttribute("TargetCellId", UintegerValue(targetCellId));
        data->LogCommandLm(m_name, cmd);
        m_lastHandoverTime[ueId] = Simulator::Now();
        commands.push_back(cmd);
    }

    return commands;
}

} // namespace ns3
