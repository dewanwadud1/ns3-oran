/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * NR counterpart of OranLmLte2LteOnnxMlb -- see that class for the full
 * design rationale.
 *
 * Per-UE state (2*numRUs+2-D): [rsrp_vec/100 (numRUs), prevLoadFrac (numRUs),
 * demand/10, t/T]. Output (numRUs Q-values): argmax = the cell the DQN
 * judges best for that UE, trading off signal quality against load.
 *
 * Unlike MRO (which issues handovers directly), this LM does NOT act on
 * individual UEs' preferred-cell votes directly: per-UE votes are aggregated
 * per cell and translated into a CIO nudge, matching the actuator type the
 * rule-based MLB (OranLmNr2NrMobilityLoadBalancing) already uses, so CDC
 * keeps recognizing this as MLB activity.
 */

#ifndef ORAN_LM_NR_2_NR_ONNX_MLB_H
#define ORAN_LM_NR_2_NR_ONNX_MLB_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include <onnxruntime_cxx_api.h>
#include <map>
#include <vector>

namespace ns3
{

class OranLmNr2NrOnnxMlb : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrOnnxMlb();
    ~OranLmNr2NrOnnxMlb() override;

    std::vector<Ptr<OranCommand>> Run() override;

    void SetDqnPath(const std::string& path);

  private:
    /**
     * Deterministic per-UE traffic demand proxy in [1, 10] Mbps. Real
     * per-user demand isn't tracked by the data repository, so (matching
     * how workspace/ml/mlb_dqn/train_mlb.py handled this for training) a
     * stable per-UE value is derived from the E2 node ID instead of being
     * re-randomized every Run().
     */
    static double SyntheticDemand(uint64_t ueE2NodeId);

    Ort::Env m_env;
    Ort::Session m_dqnSession{nullptr};
    Ort::MemoryInfo m_memoryInfo{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};
    Ort::AllocatorWithDefaultOptions m_allocator;

    double m_cioStepDb;              //!< CIO adjustment step in dB.
    double m_maxAbsCioDb;             //!< Absolute CIO clamp in dB.
    double m_voteImbalanceThreshold; //!< Fractional vote/load mismatch that triggers a CIO nudge.

}; // class OranLmNr2NrOnnxMlb

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_ONNX_MLB_H
