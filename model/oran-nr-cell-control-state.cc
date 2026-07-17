/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#include "oran-nr-cell-control-state.h"

#include <algorithm>
#include <cctype>
#include <sstream>

namespace ns3
{

namespace
{

std::map<uint64_t, OranNrCellControlParams> g_nrCellControlParams;

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
ApplyNrCellControlParameter(uint64_t e2NodeId,
                            const std::string& parameterName,
                            double value,
                            bool isDelta)
{
    OranNrCellControlParams& params = g_nrCellControlParams[e2NodeId];
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

OranNrCellControlParams
GetNrCellControlParameters(uint64_t e2NodeId)
{
    auto it = g_nrCellControlParams.find(e2NodeId);
    if (it == g_nrCellControlParams.end())
    {
        return OranNrCellControlParams();
    }
    return it->second;
}

std::map<uint64_t, OranNrCellControlParams>
GetAllNrCellControlParameters()
{
    return g_nrCellControlParams;
}

std::string
NrCellControlParametersToString(const OranNrCellControlParams& params)
{
    std::ostringstream ss;
    ss << "CIO=" << params.cioDb << ";TTT=" << params.tttSec << ";HYS=" << params.hysDb
       << ";RET=" << params.retDeg;
    return ss.str();
}

} // namespace ns3
