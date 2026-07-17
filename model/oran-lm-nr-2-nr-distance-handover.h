/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_NR_2_NR_DISTANCE_HANDOVER_H
#define ORAN_LM_NR_2_NR_DISTANCE_HANDOVER_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include "ns3/vector.h"

namespace ns3
{

/**
 * @ingroup oran
 *
 * Logic Module for the Near-RT RIC that issues Commands to handover from
 * an NR cell to another based on the distance from the UE to the gNBs.
 * Mirrors OranLmLte2LteDistanceHandover.
 */
class OranLmNr2NrDistanceHandover : public OranLm
{
  protected:
    struct UeInfo
    {
        uint64_t nodeId;
        uint16_t cellId;
        uint16_t rnti;
        Vector position;
    };

    struct GnbInfo
    {
        uint64_t nodeId;
        uint16_t cellId;
        Vector position;
    };

  public:
    static TypeId GetTypeId();
    OranLmNr2NrDistanceHandover();
    ~OranLmNr2NrDistanceHandover() override;
    std::vector<Ptr<OranCommand>> Run() override;

  private:
    std::vector<OranLmNr2NrDistanceHandover::UeInfo> GetUeInfos(Ptr<OranDataRepository> data) const;
    std::vector<OranLmNr2NrDistanceHandover::GnbInfo> GetGnbInfos(
        Ptr<OranDataRepository> data) const;
    std::vector<Ptr<OranCommand>> GetHandoverCommands(
        Ptr<OranDataRepository> data,
        std::vector<OranLmNr2NrDistanceHandover::UeInfo> ueInfos,
        std::vector<OranLmNr2NrDistanceHandover::GnbInfo> gnbInfos) const;
}; // class OranLmNr2NrDistanceHandover

} // namespace ns3

#endif /* ORAN_LM_NR_2_NR_DISTANCE_HANDOVER_H */
