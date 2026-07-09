/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup oran
 *
 * Logic Module for optimizing eNB energy consumption by adjusting transmit power.
 * Uses energy depletion rate (ΔJ per window) rather than bits/joule to avoid
 * dependency on rx-byte accounting.
 */

#ifndef ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H
#define ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H

#include "oran-lm.h"
#include "oran-data-repository.h"

namespace ns3 {

/**
 * \brief A Logic Module that monitors per-eNB energy depletion and issues
 *        TxPower adjustments to keep consumption near a target power budget.
 */
class OranLmLte2LteEnergySaving : public OranLm
{
public:
  static TypeId GetTypeId (void);
  OranLmLte2LteEnergySaving ();
  ~OranLmLte2LteEnergySaving () override;

  std::vector<Ptr<OranCommand>> Run () override;

private:
  double m_targetPowerW;  //!< Target average power consumption per eNB (W)
  double m_stepSize;      //!< Tx power adjustment step (dB) per LM invocation
  double m_lmIntervalSec; //!< Expected LM invocation interval (s) — used to derive W from ΔJ
};

} // namespace ns3

#endif /* ORAN_LM_LTE_2_LTE_ENERGY_SAVING_H */
