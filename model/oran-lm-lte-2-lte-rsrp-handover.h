/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ORAN_LM_LTE_2_LTE_RSRP_HANDOVER_H
#define ORAN_LM_LTE_2_LTE_RSRP_HANDOVER_H

#include "ns3/oran-data-repository.h"
#include "ns3/oran-lm.h"
#include <ns3/vector.h>

#include <vector>     // std::vector
#include <cstdint>    // uint16_t, uint64_t

namespace ns3
{
/**
 * \ingroup oran
 *
 * Logic Module for the Near-RT RIC that issues LTE→LTE handover commands
 * based on RSRP measurements (with hysteresis and per-UE hold-off).
 */
class OranLmLte2LteRsrpHandover : public OranLm
{
  protected:
    struct UeInfo
    {
        uint64_t nodeId; //!< E2 UE node ID
        uint16_t cellId; //!< serving LTE cell ID
        uint16_t rnti;   //!< serving RNTI
        Vector   position;
    };

    struct EnbInfo
    {
        uint64_t nodeId; //!< E2 eNB node ID
        uint16_t cellId; //!< LTE cell ID
        Vector   position;
    };

  public:
    static TypeId GetTypeId (void);
    OranLmLte2LteRsrpHandover (void);
    ~OranLmLte2LteRsrpHandover (void) override;

    std::vector<Ptr<OranCommand>> Run (void) override;

  private:
    std::vector<UeInfo>  GetUeInfos  (Ptr<OranDataRepository> data) const;
    std::vector<EnbInfo> GetEnbInfos (Ptr<OranDataRepository> data) const;

    std::vector<Ptr<OranCommand>> GetHandoverCommands(
        Ptr<OranDataRepository> data,
        std::vector<UeInfo>  ueInfos,
        std::vector<EnbInfo> enbInfos) const;
};

} // namespace ns3

#endif // ORAN_LM_LTE_2_LTE_RSRP_HANDOVER_H

