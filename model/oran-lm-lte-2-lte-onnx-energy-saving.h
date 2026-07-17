/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * DQN-based Energy Saving Logic Module (Wang et al., arXiv:2409.15098),
 * ported from MATLAB/PyTorch training (workspace/ml/es_dqn/) and deployed
 * here purely for ONNX Runtime inference -- see oran-lm-lte-2-lte-onnx-handover.cc
 * for the integration pattern this follows.
 *
 * Per-eNB state (19-D = 6*numRUs+1): [mean_RSRP, std_RSRP, max_RSRP, min_RSRP,
 * reachable_frac, prev_active] for each eNB (RSRP from all UEs' measurement
 * reports, serving or not), plus a t/T placeholder (see .cc for why this is
 * a constant at inference time). Output (2*numRUs Q-values): [Q_off(1..N),
 * Q_on(1..N)]; per-eNB argmax(Q_on, Q_off) decides sleep/wake.
 */

#ifndef ORAN_LM_LTE_2_LTE_ONNX_ENERGY_SAVING_H
#define ORAN_LM_LTE_2_LTE_ONNX_ENERGY_SAVING_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include <onnxruntime_cxx_api.h>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \brief DQN Energy Saving Logic Module, using an ONNX model for inference.
 */
class OranLmLte2LteOnnxEnergySaving : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmLte2LteOnnxEnergySaving();
    ~OranLmLte2LteOnnxEnergySaving() override;

    std::vector<Ptr<OranCommand>> Run() override;

    /**
     * Sets the path of the trained ONNX ML model.
     *
     * @param onnxModelPath the file path of the ONNX ML model.
     */
    void SetOnnxModelPath(const std::string& onnxModelPath);

  private:
    /**
     * Per-eNB aggregated RSRP statistics gathered from all UEs' measurement
     * reports (serving or neighbor), matching the per-RU feature set built
     * during training (see workspace/ml/es_dqn/train_es_dqn.py:build_state).
     */
    struct EnbRsrpStats
    {
        double mean{0.0};
        double stdDev{0.0};
        double max{0.0};
        double min{0.0};
        double reachableFrac{0.0};
    };

    /**
     * Gathers per-eNB RSRP statistics across all UEs currently registered.
     *
     * @param data The data repository.
     * @param enbIds The eNB E2 node IDs, in the fixed order used for the
     *               ONNX model's input/output vector layout.
     *
     * @return A vector of per-eNB RSRP statistics, same order as enbIds.
     */
    std::vector<EnbRsrpStats> GetEnbRsrpStats(Ptr<OranDataRepository> data,
                                              const std::vector<uint64_t>& enbIds) const;

    Ort::Env m_env;
    Ort::Session m_session{nullptr};
    Ort::MemoryInfo m_memoryInfo{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};
    Ort::AllocatorWithDefaultOptions m_allocator;

    double m_nominalTxPowerDbm; //!< "Awake" TxPower (dBm); "asleep" = 0 dBm.

    /**
     * Per-eNB believed awake/asleep state, tracked locally since the LM has
     * no read-back access to the actual LteEnbPhy TxPower register -- only
     * OranCommandLte2LteTxPower (a relative dB delta) can change it. Keyed
     * by E2 node ID. Absent entries are assumed awake (nominal TxPower).
     */
    std::map<uint64_t, bool> m_enbAwake;

}; // class OranLmLte2LteOnnxEnergySaving

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_ONNX_ENERGY_SAVING_H
