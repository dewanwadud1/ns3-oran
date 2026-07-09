/**
 * Four-xApp Conflict Scenario for ns-3.42 O-RAN.
 *
 * Demonstrates simultaneous direct and indirect conflicts between the four
 * standard O-RAN xApps: CCO, ES, MRO, and MLB.
 *
 * ICP ownership per xApp (from QACM thesis chapter):
 *   ES  : TXP
 *   CCO : TXP, RET(logical)
 *   MRO : TXP, CIO, TTT, HYS
 *   MLB : CIO, TTT, TXP(logical), RET(logical)
 *
 * Conflict pairs:
 *   DIRECT   — ES↔CCO : both issue TXP commands for the same eNBs
 *   DIRECT   — ES↔MRO : ES reduces TXP, MRO needs coverage for handovers
 *   INDIRECT — MLB↔MRO: MLB adjusts CIO/TTT, MRO uses CIO/TTT in its RSRP bias
 *
 * Topology (3 eNBs, 6 UEs):
 *   eNB-0 at x=0 m (overloaded — 6 UEs start here, triggers MLB)
 *   eNB-1 at x=200 m
 *   eNB-2 at x=400 m
 *   UE-0  at x=195 m — cell boundary, low RSRP from eNB-0 → triggers CCO
 *   UE-1..3 at x=5..11 m — near eNB-0 (load pressure)
 *   UE-4  at x=7 m, vx=+2 m/s — drifts toward eNB-1 → triggers MRO handover
 *   UE-5  at x=13 m — near eNB-0 (additional load)
 *
 * All eNBs carry energy sources (EARTH-model RU power) — ES detects power > target.
 *
 * Expected conflict sequence:
 *   t= 5 s  — ES issues TXP- on all eNBs; CCO sees low RSRP on eNB-0, issues TXP+
 *             → Direct TXP conflict on eNB-0 (and eNB-1,2 from ES only)
 *   t=10 s  — MLB detects 6:0:0 load imbalance, issues CIO-(-1dB) on eNB-0
 *             → Indirect CIO conflict: MRO now uses biased CIO in its RSRP decisions
 *   t=15+ s — UE-4 has moved ~30m toward eNB-1, MRO may trigger handover
 *             → ES/MRO conflict over TXP reduction vs coverage need
 *
 * CLI arguments:
 *   --sim-time=60              simulation duration (s)
 *   --lm-interval=5            LM query interval (s)
 *   --es-target-w=50           ES target power per eNB (W)
 *   --cco-rsrp-threshold=-70   CCO RSRP trigger threshold (dBm)
 *   --mlb-threshold=0.20       MLB load imbalance threshold
 *   --enb-distance=200         inter-eNB distance (m)
 *
 * Output:
 *   Console log from each LM at every query cycle.
 *   Final summary: per-eNB TXP, CIO, handover count, energy consumed.
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-cmm-conflict-triage.h"
#include "ns3/oran-lte-cell-control-state.h"
#include "ns3/oran-module.h"
#include "ns3/oran-ru-energy-model.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteFourXappConflict");

// ── Global KPI counters ──────────────────────────────────────────────────────
static uint32_t g_handoverOk   = 0;
static uint64_t g_rxBytesTotal = 0;

void HandoverOk(std::string, uint64_t, uint16_t, uint16_t) { g_handoverOk++; }

void RxBytes(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytesTotal += pkt->GetSize();
}

// ── eNB energy tracking ──────────────────────────────────────────────────────
static std::vector<double> g_initialEnergyJ;

int
main(int argc, char* argv[])
{
    uint16_t nEnbs      = 3;
    uint16_t nUes       = 6;
    double   simTime    = 60.0;
    double   lmInterval = 5.0;
    double   enbDist    = 200.0;
    double   ueSpeed    = 2.0;   // m/s for the moving UE

    // ES parameters
    double   esTargetW     = 50.0;
    double   esStepDb      = 1.0;
    // CCO parameters
    double   ccoRsrpDbm    = -70.0;
    double   ccoFraction   = 0.1;
    double   ccoStepDb     = 1.0;
    // MLB parameters
    double   mlbThreshold  = 0.20;
    double   mlbCioStep    = 1.0;
    // MRO parameters
    double   mroHoldoffSec = 1.5;
    double   mroHysDb      = 1.0;

    std::string dbFileName    = "oran-four-xapp-repository.db";
    std::string cmmMethod     = "none"; // none | cancel | dampen | priority | qacm

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time",          "Simulation duration (s)",     simTime);
    cmd.AddValue("mitigation-method", "Triage CMM method (none|cancel|dampen|priority|qacm)", cmmMethod);
    cmd.AddValue("lm-interval",       "LM query interval (s)",       lmInterval);
    cmd.AddValue("enb-distance",      "Inter-eNB distance (m)",      enbDist);
    cmd.AddValue("es-target-w",       "ES target power (W)",         esTargetW);
    cmd.AddValue("es-step-db",        "ES TxPower step (dB)",        esStepDb);
    cmd.AddValue("cco-rsrp-threshold","CCO RSRP threshold (dBm)",    ccoRsrpDbm);
    cmd.AddValue("cco-fraction",      "CCO low-RSRP fraction",       ccoFraction);
    cmd.AddValue("cco-step-db",       "CCO TxPower step (dB)",       ccoStepDb);
    cmd.AddValue("mlb-threshold",     "MLB imbalance threshold",     mlbThreshold);
    cmd.AddValue("mlb-cio-step",      "MLB CIO step (dB)",           mlbCioStep);
    cmd.AddValue("mro-holdoff",       "MRO handover holdoff (s)",    mroHoldoffSec);
    cmd.AddValue("mro-hys-db",        "MRO RSRP hysteresis (dB)",    mroHysDb);
    cmd.Parse(argc, argv);

    // ── Logging ──────────────────────────────────────────────────────────────
    LogComponentEnable("OranNearRtRic",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmLte2LteEnergySaving",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmLte2LteCoverageCapacityOptimization",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmLte2LteMobilityLoadBalancing",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmLte2LteRsrpHandover",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    // ── LTE / EPC ─────────────────────────────────────────────────────────────
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetAttribute("PathlossModel",
                            StringValue("ns3::Cost231PropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(50));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    lteHelper->SetHandoverAlgorithmType("ns3::NoOpHandoverAlgorithm");

    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostC;
    remoteHostC.Create(1);
    Ptr<Node> remoteHost = remoteHostC.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostC);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu",      UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay",   TimeValue(MilliSeconds(0)));
    NetDeviceContainer internetDevs = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    ipv4h.Assign(internetDevs);

    Ipv4StaticRoutingHelper ipv4Routing;
    Ptr<Ipv4StaticRouting> remoteRoute =
        ipv4Routing.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteRoute->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ── Nodes and mobility ────────────────────────────────────────────────────
    NodeContainer enbNodes, ueNodes;
    enbNodes.Create(nEnbs);
    ueNodes.Create(nUes);

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    // eNBs on x-axis
    for (uint16_t i = 0; i < nEnbs; i++)
        pos->Add(Vector(enbDist * i, 0, 20));
    // UE-0: cell boundary between eNB-0 and eNB-1 (low RSRP → CCO trigger)
    pos->Add(Vector(enbDist - 5.0, 0, 1.5));
    // UE-1..3: clustered near eNB-0 (load → MLB trigger)
    pos->Add(Vector(5.0,  0, 1.5));
    pos->Add(Vector(8.0,  0, 1.5));
    pos->Add(Vector(11.0, 0, 1.5));
    // UE-4: near eNB-0 but moving toward eNB-1 (triggers MRO)
    pos->Add(Vector(7.0,  0, 1.5));
    // UE-5: near eNB-0
    pos->Add(Vector(13.0, 0, 1.5));

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.SetPositionAllocator(pos);
    mob.Install(enbNodes);

    // UE-0 and UE-1..3,5 are stationary; UE-4 moves
    mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mob.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
            ->SetVelocity(Vector(0, 0, 0));
    // UE-4 drifts toward eNB-1
    ueNodes.Get(4)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(ueSpeed, 0, 0));

    // ── LTE devices and IP ────────────────────────────────────────────────────
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueDevs  = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueRoute =
            ipv4Routing.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueRoute->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // All UEs start on eNB-0 (deliberate imbalance for MLB)
    for (uint16_t i = 0; i < nUes; i++)
        lteHelper->Attach(ueDevs.Get(i), enbDevs.Get(0));

    lteHelper->AddX2Interface(enbNodes);

    // ── UDP traffic (1 Mbps per UE, DL) ──────────────────────────────────────
    uint16_t basePort = 1000;
    ApplicationContainer sinkApps;
    for (uint16_t i = 0; i < nUes; i++)
    {
        uint16_t port = basePort + i;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkC = sinkHelper.Install(ueNodes.Get(i));
        sinkApps.Add(sinkC);
        // connect RxBytes trace for throughput accounting
        sinkC.Get(0)->TraceConnectWithoutContext(
            "Rx", MakeCallback(&RxBytes));

        Ptr<OnOffApplication> srv = CreateObject<OnOffApplication>();
        srv->SetAttribute("Remote",
                          AddressValue(InetSocketAddress(ueIp.GetAddress(i), port)));
        srv->SetAttribute("DataRate",   DataRateValue(DataRate("1Mbps")));
        srv->SetAttribute("PacketSize", UintegerValue(1000));
        srv->SetAttribute("OnTime",
                          StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        srv->SetAttribute("OffTime",
                          StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        remoteHost->AddApplication(srv);
        srv->SetStartTime(Seconds(2));
        srv->SetStopTime(Seconds(simTime - 1.0));
    }
    sinkApps.Start(Seconds(1));
    sinkApps.Stop(Seconds(simTime));

    // ── Energy model on eNBs (EARTH-model RU) ────────────────────────────────
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(500000.0));
    srcHelper.Set("BasicEnergySupplyVoltageV",        DoubleValue(48.0));
    energy::EnergySourceContainer eSources = srcHelper.Install(enbNodes);

    std::vector<Ptr<energy::BasicEnergySource>> enbSrcVec;
    std::vector<Ptr<OranRuDeviceEnergyModel>>   enbEnergyVec;

    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
        Ptr<LteEnbPhy>       phy    = enbDev->GetPhy();
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(eSources.Get(i));
        NS_ABORT_MSG_IF(!src, "Energy source cast failed");

        Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetLteEnbPhy(phy);
        Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
        ru->SetAttribute("NumTrx",            UintegerValue(4));
        ru->SetAttribute("EtaPA",             DoubleValue(0.30));
        ru->SetAttribute("FixedOverheadW",    DoubleValue(5.0));
        ru->SetAttribute("DeltaAf",           DoubleValue(0.5));
        ru->SetAttribute("DeltaDC",           DoubleValue(0.07));
        ru->SetAttribute("DeltaMS",           DoubleValue(0.09));
        ru->SetAttribute("DeltaCool",         DoubleValue(0.10));
        ru->SetAttribute("Vdc",               DoubleValue(48.0));
        ru->SetAttribute("SleepPowerW",       DoubleValue(2.0));
        ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));
        src->AppendDeviceEnergyModel(dem);

        g_initialEnergyJ.push_back(src->GetInitialEnergy());
        enbSrcVec.push_back(src);
        enbEnergyVec.push_back(dem);
    }

    // ── Near-RT RIC with all four xApp LMs ───────────────────────────────────
    Ptr<OranHelper> oranHelper = CreateObject<OranHelper>();
    oranHelper->SetAttribute("Verbose",                   BooleanValue(true));
    oranHelper->SetAttribute("LmQueryInterval",           TimeValue(Seconds(lmInterval)));
    oranHelper->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    oranHelper->SetAttribute("E2NodeInactivityIntervalRv",
                             StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    oranHelper->SetAttribute("LmQueryMaxWaitTime",        TimeValue(Seconds(0)));
    oranHelper->SetAttribute("LmQueryLateCommandPolicy",  StringValue("DROP"));

    std::remove(dbFileName.c_str());
    oranHelper->SetDataRepository("ns3::OranDataRepositorySqlite",
                                  "DatabaseFile", StringValue(dbFileName));

    // Default LM: MRO (RSRP-based handover) — controls CIO/TTT/HYS bias + handover
    oranHelper->SetDefaultLogicModule(
        "ns3::OranLmLte2LteRsrpHandover",
        "HandoverHoldoffSec",   DoubleValue(mroHoldoffSec),
        "RsrpHysteresisDb",     DoubleValue(mroHysDb),
        "EnableCellControlBias", BooleanValue(true));

    // Additional LM: ES — controls TXP (reduction for energy saving)
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteEnergySaving",
        "TargetPowerW",   DoubleValue(esTargetW),
        "StepSize",       DoubleValue(esStepDb),
        "LmIntervalSec",  DoubleValue(lmInterval));

    // Additional LM: CCO — controls TXP (increase for coverage)
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteCoverageCapacityOptimization",
        "LowRsrpThresholdDbm",     DoubleValue(ccoRsrpDbm),
        "LowRsrpFractionThreshold", DoubleValue(ccoFraction),
        "StepSize",                DoubleValue(ccoStepDb),
        "MinSamplesPerCell",       UintegerValue(1));

    // Additional LM: MLB — controls CIO (+ logical TTT/RET for mobility)
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteMobilityLoadBalancing",
        "LoadImbalanceThreshold", DoubleValue(mlbThreshold),
        "CioStep",                DoubleValue(mlbCioStep),
        "MaxAbsCio",              DoubleValue(6.0));

    // CMM: Noop (none) shows raw conflicts; triage CMM mediates them
    if (cmmMethod == "none")
    {
        oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
    }
    else
    {
        oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                 "Method", StringValue(cmmMethod));
    }

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    // ── UE E2 terminators (location + cell-info + RSRP) ──────────────────────
    OranE2NodeTerminatorContainer e2UeTerms;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation>      locRep  = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> cellRep = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRep = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe> ueTerm  = CreateObject<OranE2NodeTerminatorLteUe>();

        locRep->SetAttribute("Terminator",  PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));

        // Wire RSRP trace from LTE PHY
        for (uint32_t d = 0; d < ueNodes.Get(idx)->GetNDevices(); d++)
        {
            Ptr<LteUeNetDevice> lteDev =
                ueNodes.Get(idx)->GetDevice(d)->GetObject<LteUeNetDevice>();
            if (lteDev)
            {
                lteDev->GetPhy()->TraceConnectWithoutContext(
                    "ReportUeMeasurements",
                    MakeCallback(&OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
            }
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
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorLteUe::Activate, ueTerm);
    }

    // ── eNB E2 terminators (location + energy reporter) ───────────────────────
    OranE2NodeTerminatorContainer e2EnbTerms;
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation>             locRep    = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteEnergyEfficiency>  energyRep =
            CreateObject<OranReporterLteEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorLteEnb> enbTerm = CreateObject<OranE2NodeTerminatorLteEnb>();

        locRep->SetAttribute("Terminator",    PointerValue(enbTerm));
        energyRep->SetAttribute("Terminator", PointerValue(enbTerm));

        enbTerm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        enbTerm->SetAttribute("RegistrationIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        enbTerm->SetAttribute("SendIntervalRv",
                              StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        enbTerm->AddReporter(locRep);
        enbTerm->AddReporter(energyRep);
        enbTerm->Attach(enbNodes.Get(idx));

        e2EnbTerms.Add(enbTerm);
    }

    // ── Activate RIC and terminators ──────────────────────────────────────────
    Simulator::Schedule(Seconds(1),
                        &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators, oranHelper, e2EnbTerms);

    // ── Handover KPI traces ───────────────────────────────────────────────────
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverOk));

    // ── Run ───────────────────────────────────────────────────────────────────
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // ── Final KPI summary ──────────────────────────────────────────────────────
    std::cout << "\n=== FOUR-XAPP CONFLICT SCENARIO — FINAL KPIs ===" << std::endl;
    std::cout << "  Sim time     : " << simTime << " s" << std::endl;
    std::cout << "  LM interval  : " << lmInterval << " s" << std::endl;

    std::cout << "\n  -- Per-eNB TXP and CIO (end of simulation) --" << std::endl;
    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
        double txp = dev->GetPhy()->GetTxPower();
        uint64_t e2id = e2EnbTerms.Get(i)->GetE2NodeId();
        OranLteCellControlParams cp = GetLteCellControlParameters(e2id);
        double energyConsumedJ =
            g_initialEnergyJ[i] - enbSrcVec[i]->GetRemainingEnergy();
        std::cout << "  eNB-" << i
                  << "  cell=" << dev->GetCellId()
                  << "  TXP=" << txp << " dBm"
                  << "  CIO=" << cp.cioDb << " dB"
                  << "  TTT=" << cp.tttSec << " s"
                  << "  Energy_consumed=" << energyConsumedJ << " J"
                  << std::endl;
    }

    double throughputMbps =
        (g_rxBytesTotal * 8.0) / (simTime * 1e6);
    std::cout << "\n  -- Network-wide KPIs --" << std::endl;
    std::cout << "  DL throughput  : " << throughputMbps << " Mbps" << std::endl;
    std::cout << "  Handovers OK   : " << g_handoverOk << std::endl;

    // Machine-readable RESULT line for test scripts
    std::cout << "\nRESULT:"
              << " method=" << cmmMethod
              << " handover_ok=" << g_handoverOk
              << " throughput_mbps=" << throughputMbps;
    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
        uint64_t e2id = e2EnbTerms.Get(i)->GetE2NodeId();
        OranLteCellControlParams cp = GetLteCellControlParameters(e2id);
        std::cout << " enb" << i << "_txp=" << dev->GetPhy()->GetTxPower()
                  << " enb" << i << "_cio=" << cp.cioDb;
    }
    std::cout << std::endl;

    Simulator::Destroy();
    return 0;
}
