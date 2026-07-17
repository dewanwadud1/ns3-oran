/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * DQN-based Mobility Robustness Optimisation (handover) Logic Module,
 * ported from DQN_rApp_HO.m (workspace/ml/mro_dqn/train_mro.py) and deployed
 * here purely for ONNX Runtime inference.
 *
 * Pipeline per UE, mirroring the MATLAB state construction:
 *   1. Kinematic features (velocity, accel, jerk, bearingRate) derived by
 *      finite-differencing recent reported positions.
 *   2. Mode classifier (kNN, ONNX): kinematics -> one of 7 mobility modes.
 *   3. Trajectory regressors (RandomForest, ONNX): kinematics+mode+position
 *      -> predicted next (X, Y).
 *   4. RSRP regressor (RandomForest, ONNX), called once per candidate eNB
 *      with that eNB's distance AND real LOS/NLOS state substituted in ->
 *      predicted RSRP per eNB. LOS/NLOS is looked up from the same
 *      ChannelConditionModel instance actually driving the downlink channel
 *      (wired in from the example via SetChannelConditionModel/
 *      SetEnbMobility/SetUeMobility) -- without it, LOS/NLOS is a hidden
 *      variable the regressor can't see, capping accuracy well below what's
 *      achievable (see workspace/ml/mro_dqn/train_mro_real.py).
 *   5. DQN head (ONNX): [mode one-hot(7), predicted next X, Y, predicted
 *      RSRP per eNB] -> target eNB index; emits a handover command if that
 *      differs from the UE's current serving cell.
 */

#ifndef ORAN_LM_LTE_2_LTE_ONNX_MRO_H
#define ORAN_LM_LTE_2_LTE_ONNX_MRO_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include "ns3/channel-condition-model.h"
#include "ns3/mobility-model.h"
#include "ns3/vector.h"

#include <onnxruntime_cxx_api.h>
#include <map>
#include <vector>

namespace ns3
{

class OranLmLte2LteOnnxMro : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmLte2LteOnnxMro();
    ~OranLmLte2LteOnnxMro() override;

    std::vector<Ptr<OranCommand>> Run() override;

    void SetModeClassifierPath(const std::string& path);
    void SetTrajXPath(const std::string& path);
    void SetTrajYPath(const std::string& path);
    void SetRsrpPath(const std::string& path);
    void SetDqnPath(const std::string& path);

    /**
     * Wires in the shared ChannelConditionModel instance actually driving
     * the downlink channel (same one the LTE PHY uses), so real LOS/NLOS
     * state can be looked up instead of left as a hidden variable. Called
     * once from the example after device installation, since that's when
     * the propagation loss model (and its channel condition model) first
     * exists -- see oran-lte-2-lte-dublin-four-xapp-example.cc.
     */
    void SetChannelConditionModel(Ptr<ChannelConditionModel> ccm);
    /** Registers an eNB's mobility model by its E2 node ID. */
    void SetEnbMobility(uint64_t enbE2NodeId, Ptr<MobilityModel> mobility);
    /** Registers a UE's mobility model by its E2 node ID. */
    void SetUeMobility(uint64_t ueE2NodeId, Ptr<MobilityModel> mobility);

  private:
    struct Kinematics
    {
        bool valid{false};
        double velocity{0.0};
        double accel{0.0};
        double jerk{0.0};
        double bearingRate{0.0};
        Vector currentPos;
    };

    /**
     * Derives (velocity, accel, jerk, bearingRate) for a UE by
     * finite-differencing its most recent reported positions. Returns
     * Kinematics::valid = false if fewer than 4 position samples are
     * available yet (cold start).
     */
    Kinematics GetKinematics(Ptr<OranDataRepository> data, uint64_t ueE2NodeId) const;

    /** Runs a single-input, single-float-output ONNX session (traj_x/y, rsrp). */
    float RunScalarSession(Ort::Session& session, const std::vector<float>& input) const;

    /** Runs the mode classifier; returns the predicted mode index [0,6]. */
    int64_t RunModeClassifier(const Kinematics& k);

    Ort::Env m_env;
    Ort::Session m_modeSession{nullptr};
    Ort::Session m_trajXSession{nullptr};
    Ort::Session m_trajYSession{nullptr};
    Ort::Session m_rsrpSession{nullptr};
    Ort::Session m_dqnSession{nullptr};
    Ort::MemoryInfo m_memoryInfo{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};
    Ort::AllocatorWithDefaultOptions m_allocator;

    /**
     * Per-UE recent position history (time-ordered), refreshed each Run()
     * from the data repository, kept only long enough to finite-difference.
     */
    mutable std::map<uint64_t, std::vector<std::pair<Time, Vector>>> m_posHistory;

    Ptr<ChannelConditionModel> m_channelConditionModel; //!< null if not wired in (LOS/NLOS omitted)
    std::map<uint64_t, Ptr<MobilityModel>> m_enbMobility; //!< eNB E2 node ID -> mobility model
    std::map<uint64_t, Ptr<MobilityModel>> m_ueMobility;   //!< UE E2 node ID -> mobility model

}; // class OranLmLte2LteOnnxMro

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_ONNX_MRO_H
