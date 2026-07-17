/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-lte-cell-control-state.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace ns3
{

namespace
{

std::map<uint64_t, OranLteCellControlParams> g_lteCellControlParams;

std::string
NormalizeParameterName(const std::string& parameterName)
{
    std::string out = parameterName;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return out;
}

} // namespace

bool
ApplyLteCellControlParameter(uint64_t e2NodeId,
                             const std::string& parameterName,
                             double value,
                             bool isDelta)
{
    OranLteCellControlParams& params = g_lteCellControlParams[e2NodeId];
    const std::string name = NormalizeParameterName(parameterName);

    double* target = nullptr;
    if (name == "CIO")
    {
        target = &params.cioDb;
    }
    else if (name == "TTT")
    {
        target = &params.tttSec;
    }
    else if (name == "HYS")
    {
        target = &params.hysDb;
    }
    else if (name == "RET")
    {
        target = &params.retDeg;
    }
    else
    {
        return false;
    }

    *target = isDelta ? *target + value : value;
    return true;
}

OranLteCellControlParams
GetLteCellControlParameters(uint64_t e2NodeId)
{
    auto it = g_lteCellControlParams.find(e2NodeId);
    if (it == g_lteCellControlParams.end())
    {
        return OranLteCellControlParams();
    }
    return it->second;
}

std::map<uint64_t, OranLteCellControlParams>
GetAllLteCellControlParameters()
{
    return g_lteCellControlParams;
}

std::string
LteCellControlParametersToString(const OranLteCellControlParams& params)
{
    std::ostringstream ss;
    ss << "CIO=" << params.cioDb << ";TTT=" << params.tttSec << ";HYS=" << params.hysDb
       << ";RET=" << params.retDeg;
    return ss.str();
}

} // namespace ns3
