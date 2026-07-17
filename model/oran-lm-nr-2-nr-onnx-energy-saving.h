/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * NR counterpart of OranLmLte2LteOnnxEnergySaving -- see that class for the
 * full design rationale. State/action layout is RAT-agnostic (RSRP stats
 * sourced from OranDataRepository's Nr-named accessors instead of Lte-named
 * ones), so this is a direct port with no structural changes.
 *
 * Per-gNB state (19-D = 6*numRUs+1): [mean_RSRP, std_RSRP, max_RSRP, min_RSRP,
 * reachable_frac, prev_active] for each gNB, plus a t/T placeholder. Output
 * (2*numRUs Q-values): [Q_off(1..N), Q_on(1..N)]; per-gNB argmax(Q_on, Q_off)
 * decides sleep/wake.
 */

#ifndef ORAN_LM_NR_2_NR_ONNX_ENERGY_SAVING_H
#define ORAN_LM_NR_2_NR_ONNX_ENERGY_SAVING_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include <onnxruntime_cxx_api.h>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \brief DQN Energy Saving Logic Module for NR, using an ONNX model for inference.
 */
class OranLmNr2NrOnnxEnergySaving : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrOnnxEnergySaving();
    ~OranLmNr2NrOnnxEnergySaving() override;

    std::vector<Ptr<OranCommand>> Run() override;

    /**
     * Sets the path of the trained ONNX ML model.
     *
     * @param onnxModelPath the file path of the ONNX ML model.
     */
    void SetOnnxModelPath(const std::string& onnxModelPath);

  private:
    /**
     * Per-gNB aggregated RSRP statistics gathered from all UEs' measurement
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
     * Gathers per-gNB RSRP statistics across all UEs currently registered.
     *
     * @param data The data repository.
     * @param enbIds The gNB E2 node IDs, in the fixed order used for the
     *               ONNX model's input/output vector layout.
     *
     * @return A vector of per-gNB RSRP statistics, same order as enbIds.
     */
    std::vector<EnbRsrpStats> GetEnbRsrpStats(Ptr<OranDataRepository> data,
                                              const std::vector<uint64_t>& enbIds) const;

    Ort::Env m_env;
    Ort::Session m_session{nullptr};
    Ort::MemoryInfo m_memoryInfo{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};
    Ort::AllocatorWithDefaultOptions m_allocator;

    double m_nominalTxPowerDbm; //!< "Awake" TxPower (dBm); "asleep" = 0 dBm.

    /**
     * Per-gNB believed awake/asleep state, tracked locally since the LM has
     * no read-back access to the actual NrGnbPhy TxPower register -- only
     * OranCommandNr2NrTxPower (a relative dB delta) can change it. Keyed
     * by E2 node ID. Absent entries are assumed awake (nominal TxPower).
     */
    std::map<uint64_t, bool> m_enbAwake;

}; // class OranLmNr2NrOnnxEnergySaving

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_ONNX_ENERGY_SAVING_H
