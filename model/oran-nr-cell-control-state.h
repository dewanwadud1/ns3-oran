/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_NR_CELL_CONTROL_STATE_H
#define ORAN_NR_CELL_CONTROL_STATE_H

#include <cstdint>
#include <map>
#include <string>

namespace ns3
{

struct OranNrCellControlParams
{
    double cioDb = 0.0;
    double tttSec = 0.0;
    double hysDb = 0.0;
    double retDeg = 0.0;
};

bool ApplyNrCellControlParameter(uint64_t e2NodeId,
                                 const std::string& parameterName,
                                 double value,
                                 bool isDelta);

OranNrCellControlParams GetNrCellControlParameters(uint64_t e2NodeId);
std::map<uint64_t, OranNrCellControlParams> GetAllNrCellControlParameters();
std::string NrCellControlParametersToString(const OranNrCellControlParams& params);

} // namespace ns3

#endif // ORAN_NR_CELL_CONTROL_STATE_H
