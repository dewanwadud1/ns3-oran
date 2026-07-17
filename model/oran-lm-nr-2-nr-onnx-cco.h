/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * NR counterpart of OranLmLte2LteOnnxCco -- see that class for the full
 * design rationale. TxPower-only (no RET), same actuator type as ES, so it
 * composes the same way in the conflict-mitigation study.
 *
 * Per-gNB state (7-D): [meanRsrpServed/100, stdRsrpServed/20, meanIFC/100,
 * loadRatio, headroom/20, t/T, n/numRUs]. Output (10 Q-values): argmax picks
 * an absolute target PTX level from a fixed dBm grid (see
 * workspace/ml/cco_dqn/calibrate_ptx_sweep.py for the NR-recalibrated
 * range); the LM tracks each gNB's believed current PTX (no PHY read-back
 * exists) and issues an OranCommandNr2NrTxPower delta to reach the target.
 */

#ifndef ORAN_LM_NR_2_NR_ONNX_CCO_H
#define ORAN_LM_NR_2_NR_ONNX_CCO_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include <onnxruntime_cxx_api.h>
#include <array>
#include <map>
#include <vector>

namespace ns3
{

class OranLmNr2NrOnnxCco : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrOnnxCco();
    ~OranLmNr2NrOnnxCco() override;

    std::vector<Ptr<OranCommand>> Run() override;

    void SetDqnPath(const std::string& path);

  private:
    /** Real per-gNB serving/interference RSRP statistics, matching the
     * calibration done offline in workspace/ml/cco_dqn/calibrate_ptx_sweep.py:
     * meanRsrpServed/stdRsrpServed over UEs actually served by this gNB, and
     * meanIfc = mean RSRP those same UEs report from the OTHER cells. */
    struct CcoStats
    {
        double meanRsrpServed{-95.0};
        double stdRsrpServed{0.0};
        double meanIfc{-115.0};
        uint32_t load{0};
    };

    std::vector<CcoStats> GetCcoStats(Ptr<OranDataRepository> data,
                                      const std::vector<uint64_t>& enbIds) const;

    Ort::Env m_env;
    Ort::Session m_dqnSession{nullptr};
    Ort::MemoryInfo m_memoryInfo{Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU)};
    Ort::AllocatorWithDefaultOptions m_allocator;

    double m_nominalTxPowerDbm; //!< Believed initial PTX (dBm) for gNBs not seen yet.

    /**
     * Per-gNB believed current PTX (dBm), tracked locally since the LM has
     * no read-back access to the actual NrGnbPhy TxPower register -- only
     * OranCommandNr2NrTxPower (a relative dB delta) can change it. Keyed
     * by E2 node ID. Absent entries default to m_nominalTxPowerDbm.
     */
    std::map<uint64_t, double> m_currentPtxDbm;

}; // class OranLmNr2NrOnnxCco

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_ONNX_CCO_H
