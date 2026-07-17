/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_COMMAND_NR_2_NR_TX_POWER_H
#define ORAN_COMMAND_NR_2_NR_TX_POWER_H

#include "oran-command.h"
#include <ns3/uinteger.h>
#include <ns3/double.h>

namespace ns3 {

/**
 * \brief Command to adjust a gNB's Tx power by a delta in dB.
 */
class OranCommandNr2NrTxPower : public OranCommand
{
public:
  static TypeId GetTypeId();
  OranCommandNr2NrTxPower();
  ~OranCommandNr2NrTxPower() override;

  double GetPowerDeltaDb() const;

private:
  double m_powerDeltaDb; //!< dB to add (can be negative)
};

} // namespace ns3

#endif // ORAN_COMMAND_NR_2_NR_TX_POWER_H
