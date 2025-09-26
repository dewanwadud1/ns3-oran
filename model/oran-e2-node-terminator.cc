/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "oran-e2-node-terminator.h"

#include "oran-near-rt-ric-e2terminator.h"
#include "oran-reporter.h"

#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/object-vector.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/nstime.h>   // Time, TimeValue, Seconds
#include <ns3/string.h>
#include <ns3/uinteger.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("OranE2NodeTerminator");
NS_OBJECT_ENSURE_REGISTERED(OranE2NodeTerminator);

TypeId
OranE2NodeTerminator::GetTypeId(void)
{
  static TypeId tid =
      TypeId("ns3::OranE2NodeTerminator")
          .SetParent<Object>()
          .AddAttribute("E2NodeId",
                        "The E2 Node ID of the terminator.",
                        UintegerValue(0),
                        MakeUintegerAccessor(&OranE2NodeTerminator::m_e2NodeId),
                        MakeUintegerChecker<uint64_t>())
          .AddAttribute("Name",
                        "The name of the terminator.",
                        StringValue("OranE2NodeTerminator"),
                        MakeStringAccessor(&OranE2NodeTerminator::m_name),
                        MakeStringChecker())
          .AddAttribute("NearRtRic",
                        "The Near-RT RIC.",
                        PointerValue(nullptr),
                        MakePointerAccessor(&OranE2NodeTerminator::m_nearRtRic),
                        MakePointerChecker<OranNearRtRic>())
          .AddAttribute("Reporters",
                        "The collection of associated reporters.",
                        ObjectVectorValue(),
                        MakeObjectVectorAccessor(&OranE2NodeTerminator::m_reporters),
                        MakeObjectVectorChecker<OranReporter>())
          .AddAttribute("RegistrationIntervalRv",
                        "Random variable (s) for periodic registration.",
                        StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                        MakePointerAccessor(&OranE2NodeTerminator::m_registrationIntervalRv),
                        MakePointerChecker<RandomVariableStream>())
          .AddAttribute("SendIntervalRv",
                        "Random variable (s) that schedules report sends.",
                        StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                        MakePointerAccessor(&OranE2NodeTerminator::m_sendIntervalRv),
                        MakePointerChecker<RandomVariableStream>())
          .AddAttribute("TransmissionDelayRv",
                        "Random variable (s) for per-report transmission delay.",
                        StringValue("ns3::ConstantRandomVariable[Constant=0]"),
                        MakePointerAccessor(&OranE2NodeTerminator::m_transmissionDelayRv),
                        MakePointerChecker<RandomVariableStream>());
  return tid;
}

OranE2NodeTerminator::OranE2NodeTerminator(void)
  : Object(),
    m_active(false),
    m_node(nullptr),
    m_reports(),
    m_registrationEvent(EventId()),
    m_sendEvent(EventId())
{
  NS_LOG_FUNCTION(this);
}

OranE2NodeTerminator::~OranE2NodeTerminator(void)
{
  NS_LOG_FUNCTION(this);
}

void
OranE2NodeTerminator::Activate(void)
{
  NS_LOG_FUNCTION(this);

  if (m_nearRtRic == nullptr)
  {
    m_active = false;
    Deactivate();
    return;
  }

  if (!m_active)
  {
    m_active = true;
    m_reports.clear();

    Register();

    for (auto const& r : m_reporters)
    {
      r->Activate();
    }
  }
}

void
OranE2NodeTerminator::AddReporter(Ptr<OranReporter> reporter)
{
  NS_LOG_FUNCTION(this << reporter);
  m_reporters.push_back(reporter);
}

void
OranE2NodeTerminator::Attach(Ptr<Node> node, uint32_t netDeviceIndex)
{
  NS_LOG_FUNCTION(this << node << netDeviceIndex);
  m_node = node;
  m_netDeviceIndex = netDeviceIndex;
}

void
OranE2NodeTerminator::Deactivate(void)
{
  NS_LOG_FUNCTION(this);

  if (!m_active)
    return;

  for (auto const& r : m_reporters)
  {
    r->Deactivate();
  }

  CancelNextSend();
  Deregister();

  m_active = false;
}

bool
OranE2NodeTerminator::IsActive(void) const
{
  NS_LOG_FUNCTION(this);
  return m_active;
}

void
OranE2NodeTerminator::StoreReport(Ptr<OranReport> report)
{
  NS_LOG_FUNCTION(this << report);

  if (m_active && report != nullptr)
  {
    m_reports.push_back(report);
  }
}

void
OranE2NodeTerminator::ReceiveDeregistrationResponse(uint64_t e2NodeId)
{
  NS_LOG_FUNCTION(this << e2NodeId);
  m_e2NodeId = 0;
}

void
OranE2NodeTerminator::ReceiveRegistrationResponse(uint64_t e2NodeId)
{
  NS_LOG_FUNCTION(this << e2NodeId);

  if (!m_active)
    return;

  if (m_e2NodeId != e2NodeId)
  {
    m_e2NodeId = e2NodeId;

    if (e2NodeId > 0)
    {
      for (auto const& r : m_reporters)
      {
        r->NotifyRegistrationComplete();
      }
    }
  }

  ScheduleNextSend();
}

void
OranE2NodeTerminator::CancelNextRegistration(void)
{
  NS_LOG_FUNCTION(this);
  if (m_registrationEvent.IsRunning())
  {
    m_registrationEvent.Cancel();
  }
}

void
OranE2NodeTerminator::CancelNextSend(void)
{
  NS_LOG_FUNCTION(this);
  if (m_sendEvent.IsRunning())
  {
    m_sendEvent.Cancel();
  }
}

void
OranE2NodeTerminator::Deregister(void)
{
  NS_LOG_FUNCTION(this);

  NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "Deregister with NULL Near-RT RIC");

  CancelNextRegistration();

  Simulator::Schedule(Seconds(m_transmissionDelayRv->GetValue()),
                      &OranNearRtRicE2Terminator::ReceiveDeregistrationRequest,
                      m_nearRtRic->GetE2Terminator(),
                      m_e2NodeId);
}

void
OranE2NodeTerminator::DoDispose(void)
{
  NS_LOG_FUNCTION(this);

  CancelNextRegistration();
  CancelNextSend();

  m_node = nullptr;
  m_nearRtRic = nullptr;
  m_reports.clear();
  m_reporters.clear();
  m_registrationIntervalRv = nullptr;
  m_sendIntervalRv = nullptr;
  m_transmissionDelayRv = nullptr;

  Object::DoDispose();
}

void
OranE2NodeTerminator::DoSendReports(void)
{
  NS_LOG_FUNCTION(this);

  if (!m_active)
    return;

  NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "Send reports to NULL Near-RT RIC");

  for (auto const& r : m_reports)
  {
    Simulator::Schedule(Seconds(m_transmissionDelayRv->GetValue()),
                        &OranNearRtRicE2Terminator::ReceiveReport,
                        m_nearRtRic->GetE2Terminator(),
                        r);
  }

  m_reports.clear();
  ScheduleNextSend();
}

void
OranE2NodeTerminator::Register(void)
{
  NS_LOG_FUNCTION(this);

  if (!m_active)
    return;

  NS_ABORT_MSG_IF(m_nearRtRic == nullptr, "Register with NULL Near-RT RIC");

  CancelNextRegistration();

  Simulator::Schedule(Seconds(m_transmissionDelayRv->GetValue()),
                      &OranNearRtRicE2Terminator::ReceiveRegistrationRequest,
                      m_nearRtRic->GetE2Terminator(),
                      GetNodeType(),
                      m_e2NodeId,
                      GetObject<OranE2NodeTerminator>());

  TimeValue inactivityThVal;
  m_nearRtRic->GetAttribute("E2NodeInactivityThreshold", inactivityThVal);
  Time registrationDelay = Seconds(m_registrationIntervalRv->GetValue());
  if (registrationDelay > inactivityThVal.Get())
  {
    NS_LOG_WARN("E2 Node registration delay > Near-RT RIC inactivity threshold.");
  }

  m_registrationEvent =
      Simulator::Schedule(registrationDelay, &OranE2NodeTerminator::Register, this);
}

void
OranE2NodeTerminator::ScheduleNextSend(void)
{
  NS_LOG_FUNCTION(this);

  if (m_active)
  {
    m_sendEvent = Simulator::Schedule(Seconds(m_sendIntervalRv->GetValue()),
                                      &OranE2NodeTerminator::DoSendReports,
                                      this);
  }
}

uint64_t
OranE2NodeTerminator::GetE2NodeId(void) const
{
  NS_LOG_FUNCTION(this);
  return m_e2NodeId;
}

Ptr<OranNearRtRic>
OranE2NodeTerminator::GetNearRtRic(void) const
{
  NS_LOG_FUNCTION(this);
  return m_nearRtRic;
}

Ptr<Node>
OranE2NodeTerminator::GetNode(void) const
{
  NS_LOG_FUNCTION(this);
  return m_node;
}

uint32_t
OranE2NodeTerminator::GetNetDeviceIndex(void) const
{
  NS_LOG_FUNCTION(this);
  return m_netDeviceIndex;
}

} // namespace ns3

