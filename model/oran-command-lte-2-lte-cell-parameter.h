/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_COMMAND_LTE_2_LTE_CELL_PARAMETER_H
#define ORAN_COMMAND_LTE_2_LTE_CELL_PARAMETER_H

#include "oran-command.h"

#include <string>

namespace ns3
{

class OranCommandLte2LteCellParameter : public OranCommand
{
  public:
    static TypeId GetTypeId();
    OranCommandLte2LteCellParameter();
    ~OranCommandLte2LteCellParameter() override;

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

#endif // ORAN_COMMAND_LTE_2_LTE_CELL_PARAMETER_H
