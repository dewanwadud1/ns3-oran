/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-multiple-net-devices-example.cc.
 * NR does not support colocating multiple NR UE net devices on one UE node
 * in the same way as LTE, so this example uses multiple UE nodes with one NR
 * net device each; OranLmNr2NrDistanceHandover still sees multiple NR UE E2
 * terminators/devices.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

#include <stdio.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrMultipleNetDevicesExample");

void
TraceGnbRx(std::string context, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t delay)
{
    std::cout << Simulator::Now().GetSeconds() << " s: " << context << " recieved " << bytes
              << " bytes from RNTI " << (uint32_t)rnti << std::endl;
}

void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " s:" << context << " gNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

void
RxTrace(std::string context, Ptr<const Packet> packet, const Address& address)
{
    InetSocketAddress isa = InetSocketAddress::ConvertFrom(address);

    std::cout << Simulator::Now().GetSeconds() << " s: " << context << " recieved "
              << packet->GetSize() << " bytes from " << isa.GetIpv4() << " on port "
              << (uint32_t)isa.GetPort() << std::endl;
}

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

void
StartNodeMotion(Ptr<Node> node, double speed, Time interval)
{
    Ptr<ConstantVelocityMobilityModel> mobility = node->GetObject<ConstantVelocityMobilityModel>();
    mobility->SetVelocity(Vector(speed, 0, 0));

    NodeContainer nodes;
    nodes.Add(node);
    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfGnbs = 2;
    uint32_t numberOfNetDevs = 2;
    uint16_t numberOfUes = numberOfNetDevs;
    Time simTime = Seconds(40);
    double distance = 200;
    Time interval = Seconds(15);
    double speed = 5;
    bool verbose = false;
    bool applicationOutput = false;
    bool gnbOutput = true;

    std::string dbFileName = "oran-nr-multiple-net-devices-repository.db";

    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("application-output",
                 "Enable printing application traffic information",
                 applicationOutput);
    cmd.AddValue("gnb-output", "Enable gNB traffic information", gnbOutput);
    cmd.AddValue("sim-time", "The amount of time to simulate", simTime);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

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

    // Create remote hosts
    NodeContainer remoteHostsContainer;
    remoteHostsContainer.Create(numberOfNetDevs);
    InternetStackHelper internet;
    internet.Install(remoteHostsContainer);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");

    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for (uint16_t i = 0; i < remoteHostsContainer.GetN(); i++)
    {
        PointToPointHelper p2ph;
        p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
        p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
        p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));

        NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHostsContainer.Get(i));
        Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
        Ptr<Node> remoteHost = remoteHostsContainer.Get(i);
        Ipv4Address remoteHostAddr = remoteHost->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
        remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                                   Ipv4Mask("255.0.0.0"),
                                                   1);
        Ptr<Ipv4StaticRouting> pgwStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(pgw->GetObject<Ipv4>());
        pgwStaticRouting->AddHostRouteTo(remoteHostAddr, internetDevices.Get(0)->GetIfIndex());
    }

    NodeContainer ueNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfGnbs; i++)
    {
        positionAlloc->Add(Vector(distance * i, 0.0, 20.0));
    }
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        const double ueYOffset = 2.0 * i;
        positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)),
                                  ueYOffset,
                                  1.5));
    }

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(0, 0, 0));
        Simulator::Schedule(Seconds(5 * idx),
                            &StartNodeMotion,
                            ueNodes.Get(idx),
                            speed,
                            interval);
    }

    // Install NR Devices: gNBs and one UE device per UE node. 5G-LENA's NR RRC
    // state machine does not tolerate multiple NR UE net devices attached from
    // the same node during initial context setup.
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);

    NetDeviceContainer ueNetDev;
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        ueNetDev.Add(nrHelper->InstallUeDevice(ueNodes.Get(i), allBwps));
    }

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

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    for (uint16_t i = 0; i < ueNetDev.GetN(); i++)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));
    }

    // Install and start applications on UEs and remote host
    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;

    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetAttribute("Min", DoubleValue(0.05));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.06));

    ApplicationContainer clientApps;
    ApplicationContainer serverApps;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        const uint32_t remoteHostIdx = u % remoteHostsContainer.GetN();
        Ptr<Node> remoteHost = remoteHostsContainer.Get(remoteHostIdx);
        Ipv4Address remoteHostAddr = remoteHost->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        Ipv4Address ueAddress = ueIpIfaces.GetAddress(u);
        const int32_t ueIfIndex = ue->GetObject<Ipv4>()->GetInterfaceForDevice(ueNetDev.Get(u));
        NS_ABORT_MSG_IF(ueIfIndex < 0, "Could not find IPv4 interface for NR UE device " << u);

        ueStaticRouting->AddHostRouteTo(remoteHostAddr,
                                        nrEpcHelper->GetUeDefaultGatewayAddress(),
                                        ueIfIndex);

        dlPort++;

        OnOffHelper dlClientHelper("ns3::UdpSocketFactory",
                                   InetSocketAddress(ueAddress, dlPort));
        dlClientHelper.SetAttribute("Local",
                                    AddressValue(InetSocketAddress(remoteHostAddr, dlPort)));
        clientApps.Add(dlClientHelper.Install(remoteHost));

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        serverApps.Add(dlPacketSinkHelper.Install(ue));

        ulPort++;

        OnOffHelper ulClientHelper("ns3::UdpSocketFactory",
                                   InetSocketAddress(remoteHostAddr, ulPort));
        ulClientHelper.SetAttribute("Local",
                                    AddressValue(InetSocketAddress(ueAddress, ulPort)));
        clientApps.Add(ulClientHelper.Install(ue));

        PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        serverApps.Add(ulPacketSinkHelper.Install(remoteHost));
    }

    Time startTime = Seconds(startTimeSeconds->GetValue());
    serverApps.Start(startTime);
    clientApps.Start(startTime);
    clientApps.Stop(simTime);

    nrHelper->AddX2Interface(gnbNodes);

    // ── ORAN Models -- BEGIN ─────────────────────────────────────────────────
    Ptr<OranNearRtRic> nearRtRic = nullptr;
    OranE2NodeTerminatorContainer e2NodeTerminatorsGnbs;
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();

    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(Seconds(5)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", EnumValue(OranNearRtRic::DROP));
    oranHelper->SetAttribute("RicTransmissionDelayRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }

    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrDistanceHandover",
                                      "ProcessingDelayRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    nearRtRic = oranHelper->CreateNearRtRic();

    // UE Nodes setup: one E2 terminator per NR UE device/node.
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorNrUe",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "TransmissionDelayRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));
    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));
    oranHelper->AddReporter(
        "ns3::OranReporterNrUeCellInfo",
        "Trigger",
        StringValue("ns3::OranReportTriggerNrUeHandover[InitialReport=true]"));
    e2NodeTerminatorsUes.Add(oranHelper->DeployTerminators(nearRtRic, ueNodes, 0));

    // gNB Nodes setup
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorNrGnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "TransmissionDelayRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    e2NodeTerminatorsGnbs.Add(oranHelper->DeployTerminators(nearRtRic, gnbNodes));

    if (verbose)
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
    // ── ORAN Models -- END ───────────────────────────────────────────────────

    if (applicationOutput)
    {
        Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",
                        MakeCallback(&RxTrace));
    }

    if (gnbOutput)
    {
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                        MakeCallback(&NotifyHandoverEndOkGnb));
    }

    Simulator::Stop(simTime);
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
