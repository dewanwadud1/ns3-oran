/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-command-nr-2-nr-tx-power.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranCommandNr2NrTxPower");
NS_OBJECT_ENSURE_REGISTERED(OranCommandNr2NrTxPower);

TypeId
OranCommandNr2NrTxPower::GetTypeId()
{
  static TypeId tid = TypeId("ns3::OranCommandNr2NrTxPower")
    .SetParent<OranCommand>()
    .AddConstructor<OranCommandNr2NrTxPower>()
    .AddAttribute("PowerDeltaDb",
                  "Adjustment in dB to apply to TxPower",
                  DoubleValue(0.0),
                  MakeDoubleAccessor(&OranCommandNr2NrTxPower::m_powerDeltaDb),
                  MakeDoubleChecker<double>());
  return tid;
}

OranCommandNr2NrTxPower::OranCommandNr2NrTxPower()
{
  NS_LOG_FUNCTION(this);
}

OranCommandNr2NrTxPower::~OranCommandNr2NrTxPower()
{
  NS_LOG_FUNCTION(this);
}

double
OranCommandNr2NrTxPower::GetPowerDeltaDb() const
{
  return m_powerDeltaDb;
}

} // namespace ns3
