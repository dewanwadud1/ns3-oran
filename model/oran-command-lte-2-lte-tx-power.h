#ifndef ORAN_COMMAND_LTE_2_LTE_TX_POWER_H
#define ORAN_COMMAND_LTE_2_LTE_TX_POWER_H

#include "oran-command.h"
#include <ns3/uinteger.h>
#include <ns3/double.h>

namespace ns3 {

/**
 * \brief Command to adjust an eNB’s Tx power by a delta in dB.
 */
class OranCommandLte2LteTxPower : public OranCommand
{
public:
  static TypeId GetTypeId();
  OranCommandLte2LteTxPower();
  ~OranCommandLte2LteTxPower() override;

  double GetPowerDeltaDb() const;

private:
  double m_powerDeltaDb; //!< dB to add (can be negative)
};

} // namespace ns3

#endif // ORAN_COMMAND_LTE_2_LTE_TX_POWER_H
