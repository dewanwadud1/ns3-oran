/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * Coverage and Capacity Optimization (CCO) xApp example for ns-3.42 O-RAN.
 *
 * Exercises: OranLmLte2LteCoverageCapacityOptimization, OranCommandLte2LteTxPower,
 *            RSRP/RSRQ reporters (OranReporterLteUeRsrpRsrq), UE cell-info reporters.
 *
 * Topology: 2 eNBs placed 300 m apart, 2 UEs.
 *           UE-0 starts at the edge of eNB-0's coverage (cell boundary, ~150 m — low RSRP ≈ -70 dBm).
 *           UE-1 starts close to eNB-0 (10 m — good RSRP).
 *           The CCO LM detects low-RSRP samples from cell-0 and increases TxPower.
 *
 * Expected output:
 *   - CCO log lines with lowRsrpFraction and txDeltaDb
 *   - RESULT: final TxPower values (eNB-0 should have increased)
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteCcoExample");

int
main(int argc, char* argv[])
{
    uint16_t numberOfEnbs = 2;
    uint16_t numberOfUes  = 2;
    Time simTime = Seconds(30);
    double distance = 300.0;  // distance between eNBs (m) — realistic micro-cell spacing
    Time lmQueryInterval = Seconds(5);
    // At ~150m from eNB with Cost231, RSRP ≈ -70 dBm; threshold -65 dBm triggers CCO
    double lowRsrpThresholdDbm = -65.0;
    double lowRsrpFraction = 0.05;  // 5% of samples below threshold triggers CCO
    double ccoStepDb = 1.0;
    bool dbLog = false;
    std::string dbFileName = "oran-cco-repository.db";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("low-rsrp-threshold-dbm", "RSRP threshold for CCO action (dBm)",
                 lowRsrpThresholdDbm);
    cmd.AddValue("low-rsrp-fraction", "Fraction of low-RSRP samples to trigger CCO",
                 lowRsrpFraction);
    cmd.AddValue("cco-step-db", "TxPower increase step (dB)", ccoStepDb);
    cmd.AddValue("db-log", "Print SQL query results", dbLog);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmLte2LteCoverageCapacityOptimization",
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

    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
        posAlloc->Add(Vector(distance * i, 0, 20));
    // UE-0 at the cell boundary (~150 m from eNB-0, low RSRP); UE-1 close to eNB-0
    posAlloc->Add(Vector(148.0, 0, 1.5));  // cell boundary — low RSRP from eNB-0
    posAlloc->Add(Vector(10.0,  0, 1.5));  // close to eNB-0 — strong RSRP

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(enbNodes);
    mobility.Install(ueNodes);

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

    // Attach both UEs to eNB-0 (so we measure eNB-0 RSRP)
    for (uint16_t i = 0; i < numberOfUes; i++)
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));

    lteHelper->AddX2Interface(enbNodes);

    // ── UDP traffic ───────────────────────────────────────────────────────────
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
        srv->SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));
        srv->SetAttribute("PacketSize", UintegerValue(1000));
        srv->SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        srv->SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        remoteHost->AddApplication(srv);
        remoteApps.Add(srv);
    }
    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime);
    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime - Seconds(1));

    // ── Near-RT RIC with CCO LM ───────────────────────────────────────────────
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
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteCoverageCapacityOptimization",
                                      "LowRsrpThresholdDbm",
                                      DoubleValue(lowRsrpThresholdDbm),
                                      "LowRsrpFractionThreshold",
                                      DoubleValue(lowRsrpFraction),
                                      "StepSize",
                                      DoubleValue(ccoStepDb),
                                      "MinSamplesPerCell",
                                      UintegerValue(1));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    // ── UE E2 terminators (with RSRP reporter) ────────────────────────────────
    OranE2NodeTerminatorContainer e2UeTerminators;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation>    locReporter  = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> cellRep    = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRep   = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe> ueTerm     = CreateObject<OranE2NodeTerminatorLteUe>();

        locReporter->SetAttribute("Terminator", PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator",     PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator",     PointerValue(ueTerm));

        // Hook RSRP measurements from the LTE PHY layer
        for (uint32_t devIdx = 0; devIdx < ueNodes.Get(idx)->GetNDevices(); devIdx++)
        {
            Ptr<LteUeNetDevice> lteUeDev =
                ueNodes.Get(idx)->GetDevice(devIdx)->GetObject<LteUeNetDevice>();
            if (lteUeDev)
            {
                lteUeDev->GetPhy()->TraceConnectWithoutContext(
                    "ReportUeMeasurements",
                    MakeCallback(&OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
            }
        }

        ueTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerm->SetAttribute("RegistrationIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerm->SetAttribute("SendIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        ueTerm->AddReporter(locReporter);
        ueTerm->AddReporter(cellRep);
        ueTerm->AddReporter(rsrpRep);
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

    // ── Run ──────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime);
    Simulator::Run();

    // ── Final KPIs ───────────────────────────────────────────────────────────
    std::cout << "\nRESULT: final_txpower_dBm";
    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        std::cout << "  eNB" << i << "=" << dev->GetPhy()->GetTxPower();
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
