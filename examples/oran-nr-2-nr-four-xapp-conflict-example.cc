/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-four-xapp-conflict-example.cc.
 *
 * Demonstrates simultaneous direct and indirect conflicts between the four
 * standard O-RAN xApps (CCO, ES, MRO, MLB) on NR gNBs, mediated by
 * OranCmmConflictTriageNr.
 *
 * Topology (3 gNBs, 6 UEs) mirrors the LTE original, scaled to sub-6GHz
 * inter-site distances.
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-cmm-conflict-triage-nr.h"
#include "ns3/oran-module.h"
#include "ns3/oran-nr-cell-control-state.h"
#include "ns3/oran-nr-ru-energy-model.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrFourXappConflict");

static uint32_t g_handoverOk = 0;
static uint64_t g_rxBytesTotal = 0;

void HandoverOk(std::string, uint64_t, uint16_t, uint16_t) { g_handoverOk++; }

void RxBytes(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytesTotal += pkt->GetSize();
}

static std::vector<double> g_initialEnergyJ;

int
main(int argc, char* argv[])
{
    uint16_t nGnbs = 3;
    uint16_t nUes = 6;
    double simTime = 60.0;
    double lmInterval = 5.0;
    double gnbDist = 800.0;
    double ueSpeed = 2.0;

    double esTargetW = 50.0;
    double esStepDb = 1.0;
    double ccoRsrpDbm = -85.0;
    double ccoFraction = 0.1;
    double ccoStepDb = 1.0;
    double mlbThreshold = 0.20;
    double mlbCioStep = 1.0;
    double mroHoldoffSec = 1.5;
    double mroHysDb = 1.0;

    std::string dbFileName = "oran-nr-four-xapp-repository.db";
    std::string cmmMethod = "none";

    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("mitigation-method",
                 "Triage CMM method (none|cancel|dampen|priority|qacm)",
                 cmmMethod);
    cmd.AddValue("lm-interval", "LM query interval (s)", lmInterval);
    cmd.AddValue("gnb-distance", "Inter-gNB distance (m)", gnbDist);
    cmd.AddValue("es-target-w", "ES target power (W)", esTargetW);
    cmd.AddValue("es-step-db", "ES TxPower step (dB)", esStepDb);
    cmd.AddValue("cco-rsrp-threshold", "CCO RSRP threshold (dBm)", ccoRsrpDbm);
    cmd.AddValue("cco-fraction", "CCO low-RSRP fraction", ccoFraction);
    cmd.AddValue("cco-step-db", "CCO TxPower step (dB)", ccoStepDb);
    cmd.AddValue("mlb-threshold", "MLB imbalance threshold", mlbThreshold);
    cmd.AddValue("mlb-cio-step", "MLB CIO step (dB)", mlbCioStep);
    cmd.AddValue("mro-holdoff", "MRO handover holdoff (s)", mroHoldoffSec);
    cmd.AddValue("mro-hys-db", "MRO RSRP hysteresis (dB)", mroHysDb);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmNr2NrEnergySaving", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmNr2NrCoverageCapacityOptimization",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmNr2NrMobilityLoadBalancing", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmNr2NrRsrpHandover", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

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
    NodeContainer remoteHostC;
    remoteHostC.Create(1);
    Ptr<Node> remoteHost = remoteHostC.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostC);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(2500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.0)));
    NetDeviceContainer internetDevs = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevs);

    Ipv4StaticRoutingHelper ipv4Routing;
    Ptr<Ipv4StaticRouting> remoteRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer gnbNodes, ueNodes;
    gnbNodes.Create(nGnbs);
    ueNodes.Create(nUes);

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < nGnbs; i++)
        pos->Add(Vector(gnbDist * i, 0, 20));
    pos->Add(Vector(gnbDist - 5.0, 0, 1.5));
    pos->Add(Vector(5.0, 0, 1.5));
    pos->Add(Vector(8.0, 0, 1.5));
    pos->Add(Vector(11.0, 0, 1.5));
    pos->Add(Vector(7.0, 0, 1.5));
    pos->Add(Vector(13.0, 0, 1.5));

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.SetPositionAllocator(pos);
    mob.Install(gnbNodes);

    mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mob.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
    ueNodes.Get(4)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(ueSpeed, 0, 0));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbDevs, randomStream);
    randomStream += nrHelper->AssignStreams(ueDevs, randomStream);

    for (uint32_t i = 0; i < gnbDevs.GetN(); i++)
    {
        nrHelper->GetGnbPhy(gnbDevs.Get(i), 0)->SetAttribute("Numerology",
                                                             UintegerValue(numerology));
    }
    nrHelper->UpdateDeviceConfigs(gnbDevs);
    nrHelper->UpdateDeviceConfigs(ueDevs);
    nrHelper->AddX2Interface(gnbNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIp = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueRoute =
            ipv4Routing.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueRoute->SetDefaultRoute(nrEpcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    for (uint16_t i = 0; i < nUes; i++)
        nrHelper->AttachToGnb(ueDevs.Get(i), gnbDevs.Get(0));

    uint16_t basePort = 1000;
    ApplicationContainer sinkApps;
    for (uint16_t i = 0; i < nUes; i++)
    {
        uint16_t port = basePort + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkC = sinkHelper.Install(ueNodes.Get(i));
        sinkApps.Add(sinkC);
        sinkC.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxBytes));

        Ptr<OnOffApplication> srv = CreateObject<OnOffApplication>();
        srv->SetAttribute("Remote", AddressValue(InetSocketAddress(ueIp.GetAddress(i), port)));
        srv->SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
        srv->SetAttribute("PacketSize", UintegerValue(1000));
        srv->SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        srv->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        remoteHost->AddApplication(srv);
        srv->SetStartTime(Seconds(2));
        srv->SetStopTime(Seconds(simTime - 1.0));

        nrHelper->ActivateDedicatedEpsBearer(ueDevs.Get(i),
                                             NrEpsBearer(NrEpsBearer::NGBR_LOW_LAT_EMBB),
                                             Create<NrEpcTft>());
    }
    sinkApps.Start(Seconds(1));
    sinkApps.Stop(Seconds(simTime));

    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(500000.0));
    srcHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(48.0));
    energy::EnergySourceContainer eSources = srcHelper.Install(gnbNodes);

    std::vector<Ptr<energy::BasicEnergySource>> gnbSrcVec;
    std::vector<Ptr<OranNrRuDeviceEnergyModel>> gnbEnergyVec;

    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        Ptr<NrGnbPhy> phy = gnbDev->GetPhy(0);
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(eSources.Get(i));
        NS_ABORT_MSG_IF(!src, "Energy source cast failed");

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

        g_initialEnergyJ.push_back(src->GetInitialEnergy());
        gnbSrcVec.push_back(src);
        gnbEnergyVec.push_back(dem);
    }

    // ── Near-RT RIC with all four xApp LMs ───────────────────────────────────
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose", BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval", TimeValue(Seconds(lmInterval)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));

    std::remove(dbFileName.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));

    oranHelper->SetDefaultLogicModule("ns3::OranLmNr2NrRsrpHandover",
                                      "HandoverHoldoffSec",
                                      DoubleValue(mroHoldoffSec),
                                      "RsrpHysteresisDb",
                                      DoubleValue(mroHysDb),
                                      "EnableCellControlBias",
                                      BooleanValue(true));

    oranHelper->AddLogicModule("ns3::OranLmNr2NrEnergySaving",
                              "TargetPowerW",
                              DoubleValue(esTargetW),
                              "StepSize",
                              DoubleValue(esStepDb),
                              "LmIntervalSec",
                              DoubleValue(lmInterval));

    oranHelper->AddLogicModule("ns3::OranLmNr2NrCoverageCapacityOptimization",
                              "LowRsrpThresholdDbm",
                              DoubleValue(ccoRsrpDbm),
                              "LowRsrpFractionThreshold",
                              DoubleValue(ccoFraction),
                              "StepSize",
                              DoubleValue(ccoStepDb),
                              "MinSamplesPerCell",
                              UintegerValue(1));

    oranHelper->AddLogicModule("ns3::OranLmNr2NrMobilityLoadBalancing",
                              "LoadImbalanceThreshold",
                              DoubleValue(mlbThreshold),
                              "CioStep",
                              DoubleValue(mlbCioStep),
                              "MaxAbsCio",
                              DoubleValue(6.0));

    if (cmmMethod == "none")
    {
        oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
    }
    else
    {
        oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriageNr",
                                                "Method",
                                                StringValue(cmmMethod));
    }

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    OranE2NodeTerminatorContainer e2UeTerms;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locRep = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> cellRep = CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranReporterNrUeRsrpRsrq> rsrpRep = CreateObject<OranReporterNrUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorNrUe> ueTerm = CreateObject<OranE2NodeTerminatorNrUe>();

        locRep->SetAttribute("Terminator", PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));

        Ptr<NrUeNetDevice> nrDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(idx));
        if (nrDev)
        {
            nrDev->GetPhy(0)->TraceConnectWithoutContext(
                "ReportUeMeasurements",
                MakeCallback(&OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
        }

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->SetAttribute("SendIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->AddReporter(locRep);
        ueTerm->AddReporter(cellRep);
        ueTerm->AddReporter(rsrpRep);
        ueTerm->Attach(ueNodes.Get(idx));

        e2UeTerms.Add(ueTerm);
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, ueTerm);
    }

    OranE2NodeTerminatorContainer e2GnbTerms;
    for (uint32_t idx = 0; idx < gnbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locRep = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrEnergyEfficiency> energyRep =
            CreateObject<OranReporterNrEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorNrGnb> gnbTerm = CreateObject<OranE2NodeTerminatorNrGnb>();

        locRep->SetAttribute("Terminator", PointerValue(gnbTerm));
        energyRep->SetAttribute("Terminator", PointerValue(gnbTerm));

        gnbTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        gnbTerm->SetAttribute("RegistrationIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        gnbTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        gnbTerm->AddReporter(locRep);
        gnbTerm->AddReporter(energyRep);
        gnbTerm->Attach(gnbNodes.Get(idx));

        e2GnbTerms.Add(gnbTerm);
    }

    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2GnbTerms);

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk", MakeCallback(&HandoverOk));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    std::cout << "\n=== NR FOUR-XAPP CONFLICT SCENARIO -- FINAL KPIs ===" << std::endl;
    std::cout << "  Sim time     : " << simTime << " s" << std::endl;
    std::cout << "  LM interval  : " << lmInterval << " s" << std::endl;

    std::cout << "\n  -- Per-gNB TXP and CIO (end of simulation) --" << std::endl;
    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        double txp = dev->GetPhy(0)->GetTxPower();
        uint64_t e2id = e2GnbTerms.Get(i)->GetE2NodeId();
        OranNrCellControlParams cp = GetNrCellControlParameters(e2id);
        double energyConsumedJ = g_initialEnergyJ[i] - gnbSrcVec[i]->GetRemainingEnergy();
        std::cout << "  gNB-" << i << "  cell=" << dev->GetCellId() << "  TXP=" << txp << " dBm"
                  << "  CIO=" << cp.cioDb << " dB"
                  << "  TTT=" << cp.tttSec << " s"
                  << "  Energy_consumed=" << energyConsumedJ << " J" << std::endl;
    }

    double throughputMbps = (g_rxBytesTotal * 8.0) / (simTime * 1e6);
    std::cout << "\n  -- Network-wide KPIs --" << std::endl;
    std::cout << "  DL throughput  : " << throughputMbps << " Mbps" << std::endl;
    std::cout << "  Handovers OK   : " << g_handoverOk << std::endl;

    std::cout << "\nRESULT:"
              << " method=" << cmmMethod << " handover_ok=" << g_handoverOk
              << " throughput_mbps=" << throughputMbps;
    for (uint32_t i = 0; i < gnbNodes.GetN(); i++)
    {
        Ptr<NrGnbNetDevice> dev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        uint64_t e2id = e2GnbTerms.Get(i)->GetE2NodeId();
        OranNrCellControlParams cp = GetNrCellControlParameters(e2id);
        std::cout << " gnb" << i << "_txp=" << dev->GetPhy(0)->GetTxPower() << " gnb" << i
                  << "_cio=" << cp.cioDb;
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
