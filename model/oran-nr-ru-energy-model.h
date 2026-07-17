/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Oran NR RU Energy Model for ns-3 / ns3-oran
 *
 * NR gNB counterpart of OranRuDeviceEnergyModel (oran-ru-energy-model.h).
 * Reuses the same RAT-agnostic OranRuPowerModel (dBm->Watt/current physics)
 * -- only the PHY pointer type differs (NrGnbPhy instead of LteEnbPhy).
 *
 * Author: Abdul Wadud, UCD, Ireland
 */

#ifndef ORAN_NR_RU_ENERGY_MODEL_H
#define ORAN_NR_RU_ENERGY_MODEL_H

#include "oran-ru-energy-model.h"

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/traced-value.h"
#include "ns3/nstime.h"
#include "ns3/type-id.h"
#include "ns3/device-energy-model.h"

namespace ns3 {

class NrGnbPhy;

namespace energy {
class EnergySource;
}

/**
 * \brief A DeviceEnergyModel that uses OranRuPowerModel to report current
 *        consumption to an EnergySource, reading TxPower from an NrGnbPhy.
 */
class OranNrRuDeviceEnergyModel : public energy::DeviceEnergyModel
{
public:
  static TypeId GetTypeId ();
  OranNrRuDeviceEnergyModel ();
  ~OranNrRuDeviceEnergyModel () override;

  // --- DeviceEnergyModel API --- //
  void SetEnergySource (Ptr<energy::EnergySource> source) override;
  double DoGetCurrentA (void) const override;
  double GetTotalEnergyConsumption () const override;
  void ChangeState (int newState) override;
  void HandleEnergyDepletion () override;
  void HandleEnergyRecharged () override;
  void HandleEnergyChanged () override;

  // --- Convenience setters --- //
  void SetNrGnbPhy (Ptr<NrGnbPhy> phy);
  Ptr<NrGnbPhy> GetNrGnbPhy () const;

  void SetRuPowerModel (Ptr<OranRuPowerModel> m);
  Ptr<OranRuPowerModel> GetRuPowerModel () const;

private:
  double ReadTxPowerDbm () const;

  Ptr<NrGnbPhy>               m_gnbPhy;
  Ptr<OranRuPowerModel>       m_model;
  Ptr<energy::EnergySource>   m_source;
  double                      m_txPowerDbm;

  mutable bool   m_initialized;
  mutable Time   m_lastUpdate;
  mutable double m_lastCurrentA;
  mutable double m_accumulatedEnergyJ;

  mutable TracedValue<double> m_traceCurrentA;
  mutable TracedValue<double> m_tracePowerW;
  mutable TracedValue<double> m_traceTxPowerDbm;
};

} // namespace ns3

#endif /* ORAN_NR_RU_ENERGY_MODEL_H */
