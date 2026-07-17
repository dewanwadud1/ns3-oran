/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_LM_NR_2_NR_RSRP_HANDOVER_H
#define ORAN_LM_NR_2_NR_RSRP_HANDOVER_H

#include "ns3/oran-data-repository.h"
#include "ns3/oran-lm.h"
#include <ns3/vector.h>

#include <vector>     // std::vector
#include <cstdint>    // uint16_t, uint64_t
#include <map>
#include <utility>

namespace ns3
{
/**
 * \ingroup oran
 *
 * Logic Module for the Near-RT RIC that issues NR->NR handover commands
 * based on RSRP measurements (with hysteresis and per-UE hold-off). Mirrors
 * OranLmLte2LteRsrpHandover.
 */
class OranLmNr2NrRsrpHandover : public OranLm
{
  protected:
    struct UeInfo
    {
        uint64_t nodeId; //!< E2 UE node ID
        uint16_t cellId; //!< serving NR cell ID
        uint16_t rnti;   //!< serving RNTI
        Vector   position;
    };

    struct GnbInfo
    {
        uint64_t nodeId; //!< E2 gNB node ID
        uint16_t cellId; //!< NR cell ID
        Vector   position;
    };

  public:
    static TypeId GetTypeId (void);
    OranLmNr2NrRsrpHandover (void);
    ~OranLmNr2NrRsrpHandover (void) override;

    std::vector<Ptr<OranCommand>> Run (void) override;

  private:
    std::vector<UeInfo>  GetUeInfos  (Ptr<OranDataRepository> data) const;
    std::vector<GnbInfo> GetGnbInfos (Ptr<OranDataRepository> data) const;

    std::vector<Ptr<OranCommand>> GetHandoverCommands(
        Ptr<OranDataRepository> data,
        std::vector<UeInfo>  ueInfos,
        std::vector<GnbInfo> gnbInfos) const;

    double m_handoverHoldoffSec;
    double m_rsrpHysteresisDb;
    double m_timeToTriggerSec;
    bool m_enableCellControlBias;
    mutable std::map<uint64_t, double> m_lastHoTime;
    mutable std::map<uint64_t, std::pair<uint16_t, double>> m_pendingTargetSince;
};

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_RSRP_HANDOVER_H
