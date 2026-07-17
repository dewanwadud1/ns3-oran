/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_COMMAND_NR_2_NR_CELL_PARAMETER_H
#define ORAN_COMMAND_NR_2_NR_CELL_PARAMETER_H

#include "oran-command.h"

#include <string>

namespace ns3
{

class OranCommandNr2NrCellParameter : public OranCommand
{
  public:
    static TypeId GetTypeId();
    OranCommandNr2NrCellParameter();
    ~OranCommandNr2NrCellParameter() override;

    std::string ToString() const override;
    std::string GetParameterName() const;
    double GetValue() const;
    bool IsDelta() const;

  private:
    std::string m_parameterName;
    double m_value;
    bool m_isDelta;
};

} // namespace ns3

#endif // ORAN_COMMAND_NR_2_NR_CELL_PARAMETER_H
