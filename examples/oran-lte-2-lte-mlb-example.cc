/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Mobility Load Balancing (MLB) xApp example for ns-3.42 O-RAN.
 *
 * Exercises: OranLmLte2LteMobilityLoadBalancing, OranCommandLte2LteCellParameter,
 *            OranLteCellControlState (CIO tracking), UE cell-info and location reporters.
 *
 * Topology: 3 eNBs placed on a line (40 m apart), 6 UEs.
 *           UEs start clustered on eNB-0 (load imbalance) then spread out.
 *           The MLB LM monitors UE counts per cell and adjusts CIO to steer
 *           load toward under-utilised cells.
 *
 * Expected output:
 *   - MLB log lines showing normalizedError and CIO adjustments
 *   - RESULT: per-cell UE counts and CIO values at end
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-lte-cell-control-state.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteMlbExample");

static uint32_t g_handoverOkCount = 0;

void
HandoverEndOk(std::string /*ctx*/, uint64_t /*imsi*/, uint16_t /*cellId*/, uint16_t /*rnti*/)
{
    g_handoverOkCount++;
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfEnbs = 3;
    uint16_t numberOfUes  = 6;
    Time simTime = Seconds(30);
    double distance = 40.0;  // distance between adjacent eNBs (m)
    double speed = 1.5;
    Time lmQueryInterval = Seconds(5);
    double loadImbalanceThreshold = 0.20;
    double cioStepDb = 1.0;
    bool dbLog = false;
    std::string dbFileName = "oran-mlb-repository.db";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("load-imbalance-threshold",
                 "Fractional imbalance threshold for MLB",
                 loadImbalanceThreshold);
    cmd.AddValue("cio-step-db", "CIO adjustment step in dB", cioStepDb);
    cmd.AddValue("db-log", "Print SQL query results", dbLog);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmLte2LteMobilityLoadBalancing",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    // ── LTE / EPC ────────────────────────────────────────────────────────────
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(50));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ── Nodes and positions ──────────────────────────────────────────────────
    NodeContainer enbNodes, ueNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    // eNBs on x-axis; all UEs start near eNB-0 (deliberate load imbalance)
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
        posAlloc->Add(Vector(distance * i, 0, 20));
    for (uint16_t i = 0; i < numberOfUes; i++)
        posAlloc->Add(Vector(5.0 + i * 2.0, 0, 1.5));  // all near eNB-0

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(enbNodes);

    // UEs drift slowly toward eNB-2 to spread load over time
    Ptr<RandomWaypointMobilityModel> dummy;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        // Odd UEs stay near eNB-0; even UEs drift right toward eNB-2
        double vx = (i % 2 == 0) ? speed : 0.0;
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(vx, 0, 0));
    }

    // ── LTE devices and IP ───────────────────────────────────────────────────
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs  = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Attach all UEs to eNB-0 initially (creates imbalance)
    for (uint16_t i = 0; i < numberOfUes; i++)
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));

    lteHelper->AddX2Interface(enbNodes);

    // ── Minimal UDP traffic ───────────────────────────────────────────────────
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
        srv->SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        srv->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        remoteHost->AddApplication(srv);
        remoteApps.Add(srv);
    }
    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime);
    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime - Seconds(1));

    // ── Near-RT RIC with MLB LM ───────────────────────────────────────────────
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose",                BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval",        TimeValue(lmQueryInterval));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",     TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy", StringValue("DROP"));

    std::remove(dbFileName.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile",
                                  StringValue(dbFileName));
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteMobilityLoadBalancing",
                                      "LoadImbalanceThreshold",
                                      DoubleValue(loadImbalanceThreshold),
                                      "CioStep",
                                      DoubleValue(cioStepDb),
                                      "MaxAbsCio",
                                      DoubleValue(6.0));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    // ── UE E2 terminators ────────────────────────────────────────────────────
    OranE2NodeTerminatorContainer e2UeTerminators;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locReporter   = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> cellRep  = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranE2NodeTerminatorLteUe> ueTerm   = CreateObject<OranE2NodeTerminatorLteUe>();

        locReporter->SetAttribute("Terminator", PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator",     PointerValue(ueTerm));

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        ueTerm->AddReporter(locReporter);
        ueTerm->AddReporter(cellRep);
        ueTerm->Attach(ueNodes.Get(idx));

        e2UeTerminators.Add(ueTerm);
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorLteUe::Activate, ueTerm);
    }

    // ── eNB E2 terminators ────────────────────────────────────────────────────
    oranHelper->SetE2NodeTerminator("ns3::OranE2NodeTerminatorLteEnb",
                                    "RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"),
                                    "SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    oranHelper->AddReporter("ns3::OranReporterLocation",
                            "Trigger",
                            StringValue("ns3::OranReportTriggerPeriodic"));

    OranE2NodeTerminatorContainer e2EnbTerminators =
        oranHelper->DeployTerminators(nearRtRic, enbNodes);

    // ── Activate ─────────────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators,
                        oranHelper,
                        e2EnbTerminators);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverEndOk));

    // ── Run ──────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime);
    Simulator::Run();

    // ── Final KPIs ───────────────────────────────────────────────────────────
    std::cout << "\nRESULT: handover_count " << g_handoverOkCount;
    std::cout << "\nRESULT: final_cio_state";
    for (uint32_t i = 0; i < e2EnbTerminators.GetN(); i++)
    {
        uint64_t e2Id = e2EnbTerminators.Get(i)->GetE2NodeId();
        OranLteCellControlParams params = GetLteCellControlParameters(e2Id);
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        std::cout << "  eNB" << i << "(cell=" << dev->GetCellId()
                  << ",e2=" << e2Id
                  << ",CIO=" << params.cioDb
                  << ")";
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
