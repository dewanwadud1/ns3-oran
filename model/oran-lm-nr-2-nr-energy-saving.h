/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * \ingroup oran
 *
 * Logic Module for optimizing gNB energy consumption by adjusting transmit
 * power. Mirrors OranLmLte2LteEnergySaving.
 */

#ifndef ORAN_LM_NR_2_NR_ENERGY_SAVING_H
#define ORAN_LM_NR_2_NR_ENERGY_SAVING_H

#include "oran-lm.h"
#include "oran-data-repository.h"

namespace ns3 {

class OranLmNr2NrEnergySaving : public OranLm
{
public:
  static TypeId GetTypeId (void);
  OranLmNr2NrEnergySaving ();
  ~OranLmNr2NrEnergySaving () override;

  std::vector<Ptr<OranCommand>> Run () override;

private:
  double m_targetPowerW;
  double m_stepSize;
  double m_lmIntervalSec;
};

} // namespace ns3

#endif /* ORAN_LM_NR_2_NR_ENERGY_SAVING_H */
