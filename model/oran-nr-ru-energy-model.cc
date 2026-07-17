/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Oran NR RU Energy Model for ns-3 / ns3-oran
 *
 * Author: Abdul Wadud, UCD, Ireland
 */

#include "oran-nr-ru-energy-model.h"

#include "ns3/log.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/pointer.h"
#include "ns3/energy-source.h"
#include "ns3/basic-energy-source.h"
#include "ns3/nr-gnb-phy.h"
#include <cmath>
#include <limits>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("OranNrRuEnergyModel");

NS_OBJECT_ENSURE_REGISTERED (OranNrRuDeviceEnergyModel);

TypeId
OranNrRuDeviceEnergyModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::OranNrRuDeviceEnergyModel")
    .SetParent<energy::DeviceEnergyModel> ()
    .SetGroupName ("Energy")
    .AddConstructor<OranNrRuDeviceEnergyModel> ()
    .AddAttribute ("TxPowerDbm",
                   "Fallback TxPower (dBm) when no NrGnbPhy is attached.",
                   DoubleValue (30.0),
                   MakeDoubleAccessor (&OranNrRuDeviceEnergyModel::m_txPowerDbm),
                   MakeDoubleChecker<double> (-200.0, 200.0))
    .AddAttribute ("PowerModel",
                   "Pointer to the OranRuPowerModel.",
                   PointerValue (CreateObject<OranRuPowerModel> ()),
                   MakePointerAccessor (&OranNrRuDeviceEnergyModel::m_model),
                   MakePointerChecker<OranRuPowerModel> ())
    .AddAttribute ("NrGnbPhy",
                   "Optional pointer to the gNB PHY.",
                   PointerValue (nullptr),
                   MakePointerAccessor (&OranNrRuDeviceEnergyModel::m_gnbPhy),
                   MakePointerChecker<NrGnbPhy> ())
    .AddTraceSource ("CurrentA",
                     "Reported device current (A).",
                     MakeTraceSourceAccessor (&OranNrRuDeviceEnergyModel::m_traceCurrentA),
                     "ns3::TracedValueCallback::Double")
    .AddTraceSource ("PowerW",
                     "Reported device power (W).",
                     MakeTraceSourceAccessor (&OranNrRuDeviceEnergyModel::m_tracePowerW),
                     "ns3::TracedValueCallback::Double")
    .AddTraceSource ("TxPowerDbmTrace",
                     "TxPower (dBm) used for computation.",
                     MakeTraceSourceAccessor (&OranNrRuDeviceEnergyModel::m_traceTxPowerDbm),
                     "ns3::TracedValueCallback::Double");
  return tid;
}

OranNrRuDeviceEnergyModel::OranNrRuDeviceEnergyModel ()
  : m_gnbPhy (nullptr),
    m_model (CreateObject<OranRuPowerModel> ()),
    m_source (nullptr),
    m_txPowerDbm (30.0),
    m_initialized (false),
    m_lastUpdate (Seconds (0.0)),
    m_lastCurrentA (0.0),
    m_accumulatedEnergyJ (0.0)
{
  NS_LOG_FUNCTION (this);
}

OranNrRuDeviceEnergyModel::~OranNrRuDeviceEnergyModel ()
{
  NS_LOG_FUNCTION (this);
  m_gnbPhy = nullptr;
  m_model = nullptr;
  m_source = nullptr;
}

void
OranNrRuDeviceEnergyModel::SetEnergySource (Ptr<energy::EnergySource> source)
{
  NS_LOG_FUNCTION (this << source);
  m_source = source;
}

void
OranNrRuDeviceEnergyModel::SetNrGnbPhy (Ptr<NrGnbPhy> phy)
{
  m_gnbPhy = phy;
}

Ptr<NrGnbPhy>
OranNrRuDeviceEnergyModel::GetNrGnbPhy () const
{
  return m_gnbPhy;
}

void
OranNrRuDeviceEnergyModel::SetRuPowerModel (Ptr<OranRuPowerModel> m)
{
  m_model = m;
}

Ptr<OranRuPowerModel>
OranNrRuDeviceEnergyModel::GetRuPowerModel () const
{
  return m_model;
}

double
OranNrRuDeviceEnergyModel::ReadTxPowerDbm () const
{
  if (m_gnbPhy != nullptr)
    return m_gnbPhy->GetTxPower ();
  return m_txPowerDbm;
}

double
OranNrRuDeviceEnergyModel::DoGetCurrentA (void) const
{
  NS_ASSERT_MSG (m_model != nullptr, "OranNrRuDeviceEnergyModel: no OranRuPowerModel set");

  const double txDbm = ReadTxPowerDbm ();
  const double currentA = m_model->GetCurrentA (txDbm);
  const double powerW   = m_model->GetPowerW   (txDbm);

  const Time now = Simulator::Now ();
  if (m_initialized)
    {
      const double dt = (now - m_lastUpdate).GetSeconds ();
      double v = 48.0;
      Ptr<energy::BasicEnergySource> b = DynamicCast<energy::BasicEnergySource> (m_source);
      if (b) v = b->GetSupplyVoltage ();
      m_accumulatedEnergyJ += m_lastCurrentA * v * dt;
    }
  else
    {
      m_initialized = true;
    }

  m_lastUpdate   = now;
  m_lastCurrentA = currentA;
  m_traceTxPowerDbm = txDbm;
  m_traceCurrentA   = currentA;
  m_tracePowerW     = powerW;

  return currentA;
}

double
OranNrRuDeviceEnergyModel::GetTotalEnergyConsumption () const
{
  const Time now = Simulator::Now ();
  if (m_initialized)
    {
      double v = 48.0;
      Ptr<energy::BasicEnergySource> b = DynamicCast<energy::BasicEnergySource> (m_source);
      if (b) v = b->GetSupplyVoltage ();
      const double dt = (now - m_lastUpdate).GetSeconds ();
      return m_accumulatedEnergyJ + (m_lastCurrentA * v * dt);
    }
  return m_accumulatedEnergyJ;
}

void OranNrRuDeviceEnergyModel::ChangeState (int) {}

void OranNrRuDeviceEnergyModel::HandleEnergyDepletion ()
{
  NS_LOG_INFO ("OranNrRuDeviceEnergyModel: Energy depleted");
}

void OranNrRuDeviceEnergyModel::HandleEnergyRecharged ()
{
  NS_LOG_INFO ("OranNrRuDeviceEnergyModel: Energy recharged");
}

void OranNrRuDeviceEnergyModel::HandleEnergyChanged () {}

} // namespace ns3
