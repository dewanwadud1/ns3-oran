/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-mlb-example.cc.
 *
 * Exercises: OranLmNr2NrMobilityLoadBalancing, OranCommandNr2NrCellParameter,
 *            OranNrCellControlState (CIO tracking).
 *
 * Topology: 3 gNBs on a line (sub-6GHz), 6 UEs starting clustered on gNB-0.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/oran-nr-cell-control-state.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrMlbExample");

static uint32_t g_handoverOkCount = 0;

void
HandoverEndOk(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t /*cellId*/, uint16_t /*rnti*/)
{
    g_handoverOkCount++;
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfGnbs = 3;
    uint16_t numberOfUes = 6;
    Time simTime = Seconds(30);
    double distance = 200.0;
    double speed = 5.0;
    Time lmQueryInterval = Seconds(5);
    double loadImbalanceThreshold = 0.20;
    double cioStepDb = 1.0;
    bool dbLog = false;
    std::string dbFileName = "oran-nr-mlb-repository.db";

    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("load-imbalance-threshold",
                 "Fractional imbalance threshold for MLB",
                 loadImbalanceThreshold);
    cmd.AddValue("cio-step-db", "CIO adjustment step in dB", cioStepDb);
    cmd.AddValue("db-log", "Print SQL query results", dbLog);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmNr2NrMobilityLoadBalancing", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));

    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency,
                                                   bandwidth,
                                                   1,
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
    ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfGnbs; i++)
        posAlloc->Add(Vector(distance * i, 0, 20));
    for (uint16_t i = 0; i < numberOfUes; i++)
        posAlloc->Add(Vector(5.0 + i * 2.0, 0, 1.5));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(gnbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        double vx = (i % 2 == 0) ? speed : 0.0;
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(vx, 0, 0));
    }

    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    for (uint32_t i = 0; i < gnbNetDev.GetN(); i++)
    {
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetAttribute("Numerology",
                                                               UintegerValue(numerology));
    }
    nrHelper->UpdateDeviceConfigs(gnbNetDev);
    nrHelper->UpdateDeviceConfigs(ueNetDev);
    nrHelper->AddX2Interface(gnbNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach all UEs to gNB-0 initially (creates imbalance)
    for (uint16_t i = 0; i < numberOfUes; i++)
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));

    uint16_t basePort = 1000;
    ApplicationContainer ueApps, remoteApps;
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        uint16_t port = basePort + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ueApps.Add(sinkHelper.Install(ueNodes.Get(i)));

        Ptr<OnOffApplication> srv = CreateObject<OnOffApplication>();
        srv->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        srv->SetAttribute("DataRate", DataRateValue(DataRate("512kbps")));
        srv->SetAttribute("PacketSize", UintegerValue(512));
        srv->SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        srv->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        remoteHost->AddApplication(srv);
        remoteApps.Add(srv);

        nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(i),
                                             NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
                                             Create<NrEpcTft>());
    }
    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime);
    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime - Seconds(1));

    // ── Near-RT RIC with MLB LM ───────────────────────────────────────────────
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));

    std::remove(dbFileName.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrMobilityLoadBalancing",
                                      "LoadImbalanceThreshold",
                                      DoubleValue(loadImbalanceThreshold),
                                      "CioStep",
                                      DoubleValue(cioStepDb),
                                      "MaxAbsCio",
                                      DoubleValue(6.0));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    OranE2NodeTerminatorContainer e2UeTerminators;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> cellRep = CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranE2NodeTerminatorNrUe> ueTerm = CreateObject<OranE2NodeTerminatorNrUe>();

        locReporter->SetAttribute("Terminator", PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->SetAttribute("SendIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        ueTerm->AddReporter(locReporter);
        ueTerm->AddReporter(cellRep);
        ueTerm->Attach(ueNodes.Get(idx));

        e2UeTerminators.Add(ueTerm);
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, ueTerm);
    }

    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorNrGnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    OranE2NodeTerminatorContainer e2GnbTerminators =
        oranHelper->DeployTerminators(nearRtRic, gnbNodes);

    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2GnbTerminators);

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverEndOk));

    Simulator::Stop(simTime);
    Simulator::Run();

    std::cout << "\nRESULT: handover_count " << g_handoverOkCount;
    std::cout << "\nRESULT: final_cio_state";
    for (uint32_t i = 0; i < e2GnbTerminators.GetN(); i++)
    {
        uint64_t e2Id = e2GnbTerminators.Get(i)->GetE2NodeId();
        OranNrCellControlParams params = GetNrCellControlParameters(e2Id);
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i));
        std::cout << "  gNB" << i << "(cell=" << dev->GetCellId() << ",e2=" << e2Id
                  << ",CIO=" << params.cioDb << ")";
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
