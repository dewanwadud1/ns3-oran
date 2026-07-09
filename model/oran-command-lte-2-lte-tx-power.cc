#include "oran-command-lte-2-lte-tx-power.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranCommandLte2LteTxPower");
NS_OBJECT_ENSURE_REGISTERED(OranCommandLte2LteTxPower);

TypeId
OranCommandLte2LteTxPower::GetTypeId()
{
  static TypeId tid = TypeId("ns3::OranCommandLte2LteTxPower")
    .SetParent<OranCommand>()
    .AddConstructor<OranCommandLte2LteTxPower>()
    .AddAttribute("PowerDeltaDb",
                  "Adjustment in dB to apply to TxPower",
                  DoubleValue(0.0),
                  MakeDoubleAccessor(&OranCommandLte2LteTxPower::m_powerDeltaDb),
                  MakeDoubleChecker<double>());
  return tid;
}

OranCommandLte2LteTxPower::OranCommandLte2LteTxPower()
{
  NS_LOG_FUNCTION(this);
}

OranCommandLte2LteTxPower::~OranCommandLte2LteTxPower()
{
  NS_LOG_FUNCTION(this);
}

double
OranCommandLte2LteTxPower::GetPowerDeltaDb() const
{
  return m_powerDeltaDb;
}

} // namespace ns3
