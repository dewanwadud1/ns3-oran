/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_LTE_CELL_CONTROL_STATE_H
#define ORAN_LTE_CELL_CONTROL_STATE_H

#include <cstdint>
#include <map>
#include <string>

namespace ns3
{

struct OranLteCellControlParams
{
    double cioDb = 0.0;
    double tttSec = 0.0;
    double hysDb = 0.0;
    double retDeg = 0.0;
};

bool ApplyLteCellControlParameter(uint64_t e2NodeId,
                                  const std::string& parameterName,
                                  double value,
                                  bool isDelta);

OranLteCellControlParams GetLteCellControlParameters(uint64_t e2NodeId);
std::map<uint64_t, OranLteCellControlParams> GetAllLteCellControlParameters();
std::string LteCellControlParametersToString(const OranLteCellControlParams& params);

} // namespace ns3

#endif // ORAN_LTE_CELL_CONTROL_STATE_H
