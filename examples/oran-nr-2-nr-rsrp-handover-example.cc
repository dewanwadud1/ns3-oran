/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-rsrp-handover-lm-example.cc.
 *
 * The scenario consists of an NR UE moving back and forth between 2 NR
 * gNBs on a single sub-6GHz band (per the project's confirmed choice: the
 * Dublin macro-site geometry used elsewhere in this repo is not realistic
 * for true mmWave, which needs sub-200m inter-site distances). The UE
 * reports its location, cell info, and RSRP/RSRQ to the RIC; the
 * OranLmNr2NrRsrpHandover LM periodically checks RSRP and, if needed,
 * issues a handover command -- exactly the same O-RAN RIC/xApp framework
 * used throughout this project, just pointed at NR devices instead of LTE.
 *
 * This is the Phase G "minimal NR-only" validation example: one rule-based
 * LM, no conflict-mitigation logic yet.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrRsrpHandoverExample");

// Trace handover events (NrUeRrc's HandoverEndOk: imsi, cellId, rnti -- same
// order as LteUeRrc, confirmed against the real nr module source).
void
HandoverTrace(Ptr<OutputStreamWrapper> stream, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    *stream->GetStream() << Simulator::Now().GetSeconds() << " " << imsi << " " << cellid << " "
                         << rnti << std::endl;
}

// Output DB queries
void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";
    if (!args.empty())
    {
        std::cout << " (" << args << ")";
    }
    std::cout << std::endl;
}

// Reverse UE velocity periodically to force back-and-forth handovers
void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(mobility->GetVelocity().x * -1, 0, 0));
    }
    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfUes = 1;
    uint16_t numberOfGnbs = 2;
    Time simTime = Seconds(30);
    double distance = 200;   // distance between gNBs (m) -- sub-6GHz macro-ish spacing
    Time interval = Seconds(15);
    double speed = 5;        // UE speed (m/s)
    bool dbLog = false;
    Time lmQueryInterval = Seconds(5);
    std::string dbFileName = "oran-nr-repository.db";

    // Sub-6GHz NR band (confirmed choice: not mmWave -- see file header comment).
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;
    double totalTxPower = 35;

    CommandLine cmd(__FILE__);
    cmd.AddValue("db-log", "Enable printing SQL queries results", dbLog);
    cmd.AddValue("lm-query-interval",
                 "The interval at which to query the LM for commands",
                 lmQueryInterval);
    cmd.AddValue("sim-time", "The amount of time to simulate", simTime);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // ── NR setup ──────────────────────────────────────────────────────────
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   numCcPerBand,
                                                   BandwidthPartInfo::UMi_StreetCanyon);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    nrHelper->InitializeOperationBand(&band);
    allBwps = CcBwpCreator::GetAllBwps({band});

    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrEpcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    NodeContainer ueNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    // Install Mobility Model (mirrors the LTE RSRP-handover example exactly)
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfGnbs; i++)
    {
        positionAlloc->Add(Vector(distance * i, 0, 20));
    }
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)), 0, 1.5));
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> m =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        m->SetVelocity(Vector(speed, 0, 0));
    }
    Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetAttribute("Numerology",
                                                               UintegerValue(numerology));
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetAttribute("TxPower",
                                                               DoubleValue(totalTxPower));
    }

    nrHelper->UpdateDeviceConfigs(gnbNetDev);
    nrHelper->UpdateDeviceConfigs(ueNetDev);

    // Required for X2-based handover (mirrors LteHelper::AddX2Interface) --
    // without this, NrGnbRrc::SendHandoverRequest() crashes because the
    // gNB's NrEpcX2 instance has no peer link info configured.
    nrHelper->AddX2Interface(gnbNodes);

    Ptr<Node> pgw = nrEpcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach all UEs to the first gNB
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));
    }

    // Simple downlink traffic so the link stays active
    uint16_t dlPort = 1234;
    ApplicationContainer serverApps;
    UdpServerHelper dlPacketSink(dlPort);
    serverApps.Add(dlPacketSink.Install(ueNodes));

    UdpClientHelper dlClient;
    dlClient.SetAttribute("RemotePort", UintegerValue(dlPort));
    dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    dlClient.SetAttribute("PacketSize", UintegerValue(1000));
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));

    NrEpsBearer bearer(NrEpsBearer::NGBR_LOW_LAT_EMBB);
    ApplicationContainer clientApps;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        dlClient.SetAttribute("RemoteAddress", AddressValue(ueIpIface.GetAddress(i)));
        clientApps.Add(dlClient.Install(remoteHost));
        nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(i), bearer, Create<NrEpcTft>());
    }

    serverApps.Start(Seconds(1));
    clientApps.Start(Seconds(2));
    serverApps.Stop(simTime + Seconds(1));
    clientApps.Stop(simTime);

    // ── ORAN Models -- BEGIN ─────────────────────────────────────────────
    Ptr<OranNearRtRic> nearRtRic = nullptr;
    OranE2NodeTerminatorContainer e2NodeTerminatorsGnbs;
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();

    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));

    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrRsrpHandover");
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    nearRtRic = oranHelper->CreateNearRtRic();

    // UE Nodes setup
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter =
            CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
        nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

        Ptr<NrUeNetDevice> nrUeDevice = ueNetDev.Get(idx)->GetObject<NrUeNetDevice>();
        if (nrUeDevice)
        {
            Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(0);
            uePhy->TraceConnectWithoutContext(
                "ReportUeMeasurements",
                MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
        }

        nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                     StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrUeTerminator->SetAttribute("SendIntervalRv",
                                     StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        nrUeTerminator->AddReporter(locationReporter);
        nrUeTerminator->AddReporter(nrUeCellInfoReporter);
        nrUeTerminator->AddReporter(rsrpRsrqReporter);

        nrUeTerminator->Attach(ueNodes.Get(idx));

        Simulator::Schedule(Seconds(1), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
    }

    // gNB Nodes setup
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorNrGnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));

    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    e2NodeTerminatorsGnbs.Add(oranHelper->DeployTerminators(nearRtRic, gnbNodes));

    if (dbLog)
    {
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsGnbs);
    Simulator::Schedule(Seconds(2),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2NodeTerminatorsUes);
    // ── ORAN Models -- END ───────────────────────────────────────────────

    // Trace successful handovers (NrUeRrc's HandoverEndOk trace, confirmed
    // identical (imsi, cellId, rnti) signature to LteUeRrc's).
    Ptr<OutputStreamWrapper> handoverTraceStream =
        Create<OutputStreamWrapper>("handover-nr.tr", std::ios::out);
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                                  MakeBoundCallback(&HandoverTrace, handoverTraceStream));

    Simulator::Stop(simTime);
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
