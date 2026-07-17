/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
#ifndef ORAN_REPORT_TRIGGER_NR_UE_HANDOVER_H
#define ORAN_REPORT_TRIGGER_NR_UE_HANDOVER_H

#include "oran-report-trigger.h"

#include "ns3/object.h"
#include "ns3/ptr.h"

#include <string>

namespace ns3
{

class OranReporter;

/**
 * @ingroup oran
 *
 * A class that triggers reports based on the successful handover of an NR
 * UE. Mirrors OranReportTriggerLteUeHandover, hooked to NrUeRrc's
 * "HandoverEndOk"/"ConnectionEstablished" trace sources (confirmed to have
 * the identical (imsi, cellId, rnti) signature as their LTE counterparts).
 */
class OranReportTriggerNrUeHandover : public OranReportTrigger
{
  public:
    static TypeId GetTypeId();
    OranReportTriggerNrUeHandover();
    ~OranReportTriggerNrUeHandover() override;
    void Activate(Ptr<OranReporter> reporter) override;
    void Deactivate() override;

  protected:
    void DoDispose() override;
    virtual void HandoverCompleteSink(uint64_t imsi, uint16_t cellId, uint16_t rnti);
    virtual void ConnectionEstablishedSink(uint64_t imsi, uint16_t cellId, uint16_t rnti);

  private:
    void DisconnectSink();
}; // class OranReportTriggerNrUeHandover

} // namespace ns3

#endif /* ORAN_REPORT_TRIGGER_NR_UE_HANDOVER_H */
