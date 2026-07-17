/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * Diagnostic-only Logic Module: issues a large, known-schedule TxPower
 * square wave (alternating between two fixed extremes) on every gNB,
 * bypassing all real xApp decision logic. Used to empirically measure the
 * lag between a parameter change and its effect appearing in reported KPIs
 * (RSRP/SINR), with a signal amplitude (default 35 dB swing) far larger
 * than normal channel-fading noise, so the response is unambiguous.
 *
 * Not part of the four production xApps -- for calibration runs only (see
 * --calibrate-txp-lag in oran-nr-2-nr-dublin-four-xapp-example.cc).
 */

#ifndef ORAN_LM_NR_2_NR_TXP_CALIBRATION_H
#define ORAN_LM_NR_2_NR_TXP_CALIBRATION_H

#include "oran-lm.h"

#include <map>

namespace ns3
{

class OranLmNr2NrTxpCalibration : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmNr2NrTxpCalibration();
    ~OranLmNr2NrTxpCalibration() override;

    std::vector<Ptr<OranCommand>> Run() override;

  private:
    double m_lowDbm;             //!< Low extreme of the square wave (dBm).
    double m_highDbm;            //!< High extreme of the square wave (dBm).
    uint32_t m_periodCycles;     //!< Cycles spent at each extreme before toggling.

    std::map<uint64_t, bool> m_isHigh;      //!< Per-gNB: currently at the high extreme?
    std::map<uint64_t, uint32_t> m_cycleCount; //!< Per-gNB: cycles since last toggle.
    std::map<uint64_t, double> m_believedDbm;  //!< Per-gNB: believed current TxPower (dBm).

}; // class OranLmNr2NrTxpCalibration

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_TXP_CALIBRATION_H
