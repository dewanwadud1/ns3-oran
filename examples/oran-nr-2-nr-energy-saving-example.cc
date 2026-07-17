/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-energy-saving-example.cc.
 *
 * Exercises: OranLmNr2NrEnergySaving, OranReporterNrEnergyEfficiency,
 *            OranCommandNr2NrTxPower, OranNrRuDeviceEnergyModel,
 *            SaveNrEnergyRemaining / GetNrEnergyRemaining DB APIs.
 *
 * Topology: 2 gNBs on a sub-6GHz band, 1 UE moving back and forth.
 *           Each gNB has a BasicEnergySource + OranNrRuDeviceEnergyModel.
 *           The Near-RT RIC runs OranLmNr2NrEnergySaving every 5 s.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrEnergySavingExample");

void
ReverseVelocity(NodeContainer nodes, Time interval)
{
    for (uint32_t idx = 0; idx < nodes.GetN(); idx++)
    {
        Ptr<ConstantVelocityMobilityModel> mob =
            nodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(mob->GetVelocity().x * -1, 0, 0));
    }
    Simulator::Schedule(interval, &ReverseVelocity, nodes, interval);
}

void
QueryRcSink(std::string query, std::string args, int rc)
{
    std::cout << Simulator::Now().GetSeconds() << " Query "
              << ((rc == SQLITE_OK || rc == SQLITE_DONE) ? "OK" : "ERROR") << "(" << rc << "): \""
              << query << "\"";
    if (!args.empty())
        std::cout << " (" << args << ")";
    std::cout << std::endl;
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfGnbs = 2;
    uint16_t numberOfUes = 1;
    Time simTime = Seconds(30);
    double distance = 200.0;
    Time interval = Seconds(15);
    double speed = 5.0;
    Time lmQueryInterval = Seconds(5);
    double targetPowerW = 50.0;
    double stepSizeDb = 1.0;
    bool dbLog = false;
    std::string dbFileName = "oran-nr-energy-saving-repository.db";

    // Sub-6GHz NR band (confirmed project choice, not mmWave).
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("target-power-w", "Energy-saving target power per gNB (W)", targetPowerW);
    cmd.AddValue("step-size-db", "TxPower step (dB) per LM decision", stepSizeDb);
    cmd.AddValue("db-log", "Print SQL query results", dbLog);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmNr2NrEnergySaving", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));

    // ── NR setup ─────────────────────────────────────────────────────────────
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
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ── Nodes and mobility ───────────────────────────────────────────────────
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfGnbs; i++)
        positionAlloc->Add(Vector(distance * i, 0, 20));
    for (uint16_t i = 0; i < numberOfUes; i++)
        positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)), 0, 1.5));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gnbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
        ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(speed, 0, 0));
    Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

    // ── NR devices and IP ────────────────────────────────────────────────────
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

    for (uint16_t i = 0; i < numberOfUes; i++)
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));

    // ── UDP traffic (remote host -> UE) ──────────────────────────────────────
    uint16_t basePort = 1000;
    ApplicationContainer ueApps, remoteApps;
    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        uint16_t port = basePort * (i + 1);
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ueApps.Add(sinkHelper.Install(ueNodes.Get(i)));

        Ptr<OnOffApplication> srv = CreateObject<OnOffApplication>();
        srv->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        srv->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        srv->SetAttribute("PacketSize", UintegerValue(1500));
        srv->SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
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

    // ── Energy model: BasicEnergySource + OranNrRuDeviceEnergyModel ──────────
    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(500000.0));
    sourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    energy::EnergySourceContainer sources = sourceHelper.Install(gnbNodes);

    std::vector<Ptr<OranNrRuDeviceEnergyModel>> gnbEnergyModels;
    std::vector<Ptr<energy::BasicEnergySource>> gnbEnergySources;

    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i));
        Ptr<NrGnbPhy> phy = gnbDev->GetPhy(0);
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(sources.Get(i));
        NS_ABORT_MSG_IF(src == nullptr, "Energy source is not a BasicEnergySource");

        Ptr<OranNrRuDeviceEnergyModel> dem = CreateObject<OranNrRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetNrGnbPhy(phy);

        Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
        ru->SetAttribute("NumTrx", UintegerValue(4));
        ru->SetAttribute("EtaPA", DoubleValue(0.30));
        ru->SetAttribute("FixedOverheadW", DoubleValue(5.0));
        ru->SetAttribute("DeltaAf", DoubleValue(0.5));
        ru->SetAttribute("DeltaDC", DoubleValue(0.07));
        ru->SetAttribute("DeltaMS", DoubleValue(0.09));
        ru->SetAttribute("DeltaCool", DoubleValue(0.10));
        ru->SetAttribute("Vdc", DoubleValue(48.0));
        ru->SetAttribute("SleepPowerW", DoubleValue(2.0));
        ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));

        src->AppendDeviceEnergyModel(dem);
        gnbEnergyModels.push_back(dem);
        gnbEnergySources.push_back(src);
    }

    // ── Near-RT RIC with Energy-Saving LM ────────────────────────────────────
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
    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrEnergySaving",
                                      "TargetPowerW",
                                      DoubleValue(targetPowerW),
                                      "StepSize",
                                      DoubleValue(stepSizeDb),
                                      "LmIntervalSec",
                                      DoubleValue(lmQueryInterval.GetSeconds()));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    if (dbLog)
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));

    // ── UE E2 terminators ────────────────────────────────────────────────────
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> cellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranE2NodeTerminatorNrUe> ueTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        cellInfoReporter->SetAttribute("Terminator", PointerValue(ueTerminator));

        ueTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerminator->SetAttribute("RegistrationIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerminator->SetAttribute("SendIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        ueTerminator->AddReporter(locationReporter);
        ueTerminator->AddReporter(cellInfoReporter);
        ueTerminator->Attach(ueNodes.Get(idx));

        e2NodeTerminatorsUes.Add(ueTerminator);
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, ueTerminator);
    }

    // ── gNB E2 terminators (manual, to wire energy source) ───────────────────
    OranE2NodeTerminatorContainer e2NodeTerminatorsGnbs;
    for (uint32_t idx = 0; idx < gnbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrEnergyEfficiency> energyReporter =
            CreateObject<OranReporterNrEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorNrGnb> gnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

        locReporter->SetAttribute("Terminator", PointerValue(gnbTerminator));
        locReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        energyReporter->SetEnergySource(gnbEnergySources[idx]);
        energyReporter->SetAttribute("Terminator", PointerValue(gnbTerminator));
        energyReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        gnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        gnbTerminator->SetAttribute("RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        gnbTerminator->SetAttribute("SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=2]"));

        gnbTerminator->AddReporter(locReporter);
        gnbTerminator->AddReporter(energyReporter);
        gnbTerminator->Attach(gnbNodes.Get(idx));

        e2NodeTerminatorsGnbs.Add(gnbTerminator);
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, gnbTerminator);
    }

    // ── Activate O-RAN ───────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    // ── Run ──────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime);
    Simulator::Run();

    // ── Final KPIs ───────────────────────────────────────────────────────────
    std::cout << "\nRESULT: energy_consumed_J";
    for (uint32_t i = 0; i < gnbEnergyModels.size(); i++)
        std::cout << "  gNB" << i << "=" << gnbEnergyModels[i]->GetTotalEnergyConsumption();
    std::cout << "\nRESULT: final_txpower_dBm";
    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i));
        std::cout << "  gNB" << i << "=" << dev->GetPhy(0)->GetTxPower();
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
