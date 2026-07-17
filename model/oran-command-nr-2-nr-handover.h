/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_COMMAND_NR_2_NR_HANDOVER_H
#define ORAN_COMMAND_NR_2_NR_HANDOVER_H

#include "oran-command.h"

namespace ns3
{

/**
 * @ingroup oran
 * A Command instructing an NR gNB to handover a UE to another NR gNB.
 * In this command the Target E2 Node ID is the serving NR gNB, and the command
 * contains the cell ID of the gNB to handover to, and the RNTI of the UE to be
 * handoverd.
 */
class OranCommandNr2NrHandover : public OranCommand
{
  public:
    /**
     * Gets the TypeId of the OranCommandNr2NrHandover class.
     *
     * @return The TypeId.
     */
    static TypeId GetTypeId();
    /**
     * Creates an instance of the OranCommandNr2NrHandover class.
     */
    OranCommandNr2NrHandover();
    /**
     * The destructor of the OranCommandNr2NrHandover class.
     */
    ~OranCommandNr2NrHandover() override;

    std::string ToString() const override;

  private:
    /**
     * The ID of the cell to handover to.
     */
    uint16_t m_targetCellId;
    /**
     * The RNTI of the UE to handover.
     */
    uint16_t m_targetRnti;

  public:
    /**
     * Gets the ID of the cell to handover to.
     *
     * @returns The cell ID.
     */
    uint16_t GetTargetCellId() const;
    /**
     * Gets the RNTI of the UE to handover.
     *
     * @returns The RNTI.
     */
    uint16_t GetTargetRnti() const;
}; // class OranCommandNr2NrHandover

} // namespace ns3

#endif /* ORAN_COMMAND_NR_2_NR_HANDOVER_H */
