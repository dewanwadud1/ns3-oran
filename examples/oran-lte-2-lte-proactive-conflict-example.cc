/**
 * Proactive Conflict Management Example for ns-3.42 O-RAN (Chapter 6, TNSM).
 *
 * Demonstrates proactive conflict prevention through KPI prediction (EMA-based
 * RSRP forecasting) vs. reactive conflict response (CCO acts after violation).
 *
 * Experiment modes (--mode=<X>):
 *   baseline : ES + MRO + MLB; Noop CMM.
 *              ES aggressively lowers TXP each cycle toward 15 W target.
 *              No coverage protection -> RSRP degrades -> many violation cycles.
 *
 *   reactive : ES + MRO + MLB + CCO (threshold=-70 dBm); priority CMM.
 *              CCO acts AFTER RSRP crosses -70 dBm; some violations occur before CCO reacts.
 *
 *   proactive: ES + MRO + MLB + KPI predictor (threshold=-65 dBm); priority CMM.
 *              Predictor acts when forecast RSRP < -65 dBm (before -70 violation).
 *              Fewer or zero violations compared to reactive.
 *
 * Key KPI: rsrp_violation_cycles = number of LM-query cycles where any UE's serving-cell
 *          RSRP drops below -70 dBm. Expected: baseline >> reactive > proactive.
 *
 * Topology: 3 eNBs (200 m apart), 6 UEs.
 *   UE-0 at 62.8 m (3-D) from eNB-0: RSRP = -65 dBm at TXP=30 dBm.
 *   Path loss: LogDistance, L0=4.29 dB, exponent=3.5.
 *   RSRP formula (50 RBs, 12 SC/RB): RSRP = TXP - PL - 10*log10(600) = TXP - PL - 27.78.
 *   ES target = 15 W -> forces TXP from 30 toward ~20 dBm (1 dB/cycle).
 *   At TXP=24: RSRP = -71 dBm -> violation.  Predictor triggers when minRsrp < -65 dBm.
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

#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteProactiveConflict");

// Global KPI counters
static uint32_t g_handoverOk          = 0;
static uint64_t g_rxBytesTotal        = 0;
static uint32_t g_rsrpViolationCycles = 0;
static double   g_rsrpViolThreshDbm   = -70.0;
// Per-UE latest serving-cell RSRP (keyed by RNTI).
// Checks fire 2 s AFTER each LM boundary so the CMM (which fires
// ~1 s after the LM query) has applied its corrections before we sample.
static std::map<uint16_t, double> g_latestUeRsrp;
// Per-UE latest serving cell ID (keyed by RNTI) — for per-cell RSRP aggregation in [STATE].
static std::map<uint16_t, uint16_t> g_latestUeCell;
// eNB devices and terminators exposed for LogCellState()
static NetDeviceContainer              g_enbDevs;
static OranE2NodeTerminatorContainer   g_e2EnbTerms;

void
HandoverOk(std::string, uint64_t, uint16_t, uint16_t)
{
    g_handoverOk++;
}

void
RxBytes(Ptr<const Packet> pkt, const Address&)
{
    g_rxBytesTotal += pkt->GetSize();
}

// Trace callback: record the most recent serving-cell RSRP per UE.
void
ObserveRsrp(uint16_t rnti,
            uint16_t cellId,
            double rsrp,
            double /*rsrq*/,
            bool serving,
            uint8_t /*ccId*/)
{
    if (!serving)
        return;
    g_latestUeRsrp[rnti]  = rsrp;
    g_latestUeCell[rnti]   = cellId;
}

// Periodic per-eNB state log: emits [STATE] line with current TXP, CIO, and per-cell min RSRP.
// Runs at the same schedule as CheckViolation so both appear at the same timestamp.
void
LogCellState(Time interval)
{
    double now = Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < g_enbDevs.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(g_enbDevs.Get(i));
        uint64_t e2id = g_e2EnbTerms.Get(i)->GetE2NodeId();
        OranLteCellControlParams cp = GetLteCellControlParameters(e2id);

        // Compute min RSRP and UE count for this cell
        uint16_t cellId  = dev->GetCellId();
        double   minRsrp = 0.0;
        uint32_t nUes    = 0;
        for (const auto& kv : g_latestUeRsrp)
        {
            auto cit = g_latestUeCell.find(kv.first);
            if (cit == g_latestUeCell.end() || cit->second != cellId)
                continue;
            if (nUes == 0 || kv.second < minRsrp)
                minRsrp = kv.second;
            nUes++;
        }

        std::cout << "[STATE] t=" << now << "s"
                  << " e2=" << e2id
                  << " cell=" << cellId
                  << " TXP=" << std::fixed << std::setprecision(2)
                  << dev->GetPhy()->GetTxPower() << "dBm"
                  << " CIO=" << cp.cioDb << "dB"
                  << " TTT=" << cp.tttSec * 1000.0 << "ms"
                  << " nUEs=" << nUes
                  << " minRSRP=" << (nUes > 0 ? minRsrp : 0.0) << "dBm"
                  << "\n";
    }
    Simulator::Schedule(interval, &LogCellState, interval);
}

// Periodic check: sample the worst serving-cell RSRP at this instant.
// Scheduled 2 s after each LM boundary so post-CMM corrections are visible.
// This is the PMon stage of the CMS pipeline: detect KPI degradation below QoS.
// When a violation is found, the CDC (inside OranCmmConflictTriage::Filter) will
// identify the conflicting xApps and ICP at the NEXT LM cycle, and the CMC will
// apply the configured mitigation method.
void
CheckViolation(Time interval)
{
    double worstRsrp = 0.0;
    for (const auto& kv : g_latestUeRsrp)
    {
        if (worstRsrp == 0.0 || kv.second < worstRsrp)
            worstRsrp = kv.second;
    }

    if (worstRsrp > -200.0) // guard against uninitialized map
    {
        bool viol = (worstRsrp < g_rsrpViolThreshDbm);
        if (viol)
            g_rsrpViolationCycles++;

        std::cout << "[PMON] t=" << Simulator::Now().GetSeconds() << "s"
                  << " worst_RSRP=" << std::fixed << std::setprecision(2)
                  << worstRsrp << "dBm"
                  << " QoS_threshold=" << g_rsrpViolThreshDbm << "dBm"
                  << " status=" << (viol ? "VIOLATION ← triggers CDC/CMC next cycle"
                                         : "OK")
                  << "\n";
    }

    Simulator::Schedule(interval, &CheckViolation, interval);
}

int
main(int argc, char* argv[])
{
    uint16_t    nEnbs      = 3;
    uint16_t    nUes       = 6;
    double      simTime    = 60.0;
    double      lmInterval = 5.0;
    double      enbDist    = 200.0;
    double      ueSpeed    = 0.0;  // stationary — no MRO handover contaminating violation count
    std::string mode       = "reactive"; // baseline | reactive | proactive

    double esTargetW       = 15.0;  // ES target: forces TXP from 30 down to ~20 dBm over 10 cycles
    double esStepDb        = 1.0;

    double ccoRsrpDbm      = -70.0; // reactive CCO threshold (also violation threshold)
    double proactiveThresh = -65.0; // predictor acts 5 dB before -70 violation threshold

    double mlbThreshold    = 0.20;
    double mroHoldoffSec   = 1.5;
    double mroHysDb        = 1.0;

    std::string dbFileName = "oran-proactive-repository.db";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time",           "Simulation duration (s)",             simTime);
    cmd.AddValue("lm-interval",        "LM query interval (s)",               lmInterval);
    cmd.AddValue("mode",               "baseline | reactive | proactive",      mode);
    cmd.AddValue("enb-distance",       "Inter-eNB distance (m)",              enbDist);
    cmd.AddValue("es-target-w",        "ES target power per eNB (W)",         esTargetW);
    cmd.AddValue("es-step-db",         "ES TxPower step (dB)",                esStepDb);
    cmd.AddValue("cco-rsrp-threshold", "CCO reactive RSRP threshold (dBm)",   ccoRsrpDbm);
    cmd.AddValue("proactive-thr-dbm",  "Predictor proactive threshold (dBm)", proactiveThresh);
    cmd.AddValue("mlb-threshold",      "MLB imbalance threshold",             mlbThreshold);
    cmd.Parse(argc, argv);

    g_rsrpViolThreshDbm = ccoRsrpDbm;

    LogComponentEnable("OranLmLte2LteKpiPrediction",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmLte2LteEnergySaving",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranLmLte2LteCoverageCapacityOptimization",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranCmmConflictTriage",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranNearRtRic",
                       (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    // LTE / EPC
    // LogDistance model calibrated for scenario:
    //   PL(d) = 4.29 + 35*log10(d)  [dB, reference 1 m]
    //   ns-3 LTE RSRP (50 RBs, 12 SC/RB) = TxPower - PL - 10*log10(600) = TxPower - PL - 27.78
    //   3D distance eNB-0→UE-0 = 62.8 m → PL = 67.22 dB
    //   TXP=30: RSRP = 30 - 67.22 - 27.78 = -65 dBm  (proactive threshold)
    //   TXP=24: RSRP = 24 - 95 = -71 dBm               (violation starts)
    //   Baseline violations start cycle 7 (TXP drops to 24 after 6 ES steps).
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetAttribute("PathlossModel",
                            StringValue("ns3::LogDistancePropagationLossModel"));
    lteHelper->SetPathlossModelAttribute("ReferenceLoss", DoubleValue(4.29));
    lteHelper->SetPathlossModelAttribute("Exponent",     DoubleValue(3.5));
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

    // Nodes and mobility
    NodeContainer enbNodes, ueNodes;
    enbNodes.Create(nEnbs);
    ueNodes.Create(nUes);

    Ptr<ListPositionAllocator> pos = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < nEnbs; i++)
        pos->Add(Vector(enbDist * i, 0, 20));
    pos->Add(Vector(60.0, 0, 1.5));  // UE-0: RSRP ~ -61 dBm at 30 dBm TXP (Cost231, shadowing=0)
    pos->Add(Vector(5.0,   0, 1.5));
    pos->Add(Vector(8.0,   0, 1.5));
    pos->Add(Vector(11.0,  0, 1.5));
    pos->Add(Vector(7.0,   0, 1.5)); // UE-4: moves toward eNB-1 (MRO trigger)
    pos->Add(Vector(13.0,  0, 1.5));

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.SetPositionAllocator(pos);
    mob.Install(enbNodes);

    mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mob.Install(ueNodes);
    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
            ->SetVelocity(Vector(0, 0, 0));
    ueNodes.Get(4)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(ueSpeed, 0, 0));

    // LTE devices and IP
    g_enbDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer& enbDevs = g_enbDevs;
    NetDeviceContainer ueDevs  = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIp = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueRoute =
            ipv4Routing.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueRoute->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    for (uint16_t i = 0; i < nUes; i++)
        lteHelper->Attach(ueDevs.Get(i), enbDevs.Get(0));
    lteHelper->AddX2Interface(enbNodes);

    // UDP traffic (1 Mbps DL per UE)
    uint16_t basePort = 1000;
    ApplicationContainer sinkApps;
    for (uint16_t i = 0; i < nUes; i++)
    {
        uint16_t port = basePort + i;
        PacketSinkHelper sinkH("ns3::UdpSocketFactory",
                               InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkC = sinkH.Install(ueNodes.Get(i));
        sinkApps.Add(sinkC);
        sinkC.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&RxBytes));

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

    // Energy model on eNBs (EARTH-model RU)
    BasicEnergySourceHelper srcHelper;
    srcHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(500000.0));
    srcHelper.Set("BasicEnergySupplyVoltageV",        DoubleValue(48.0));
    energy::EnergySourceContainer eSources = srcHelper.Install(enbNodes);

    std::vector<Ptr<energy::BasicEnergySource>> enbSrcVec;
    std::vector<double> initialEnergyJ;

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

        initialEnergyJ.push_back(src->GetInitialEnergy());
        enbSrcVec.push_back(src);
    }

    // Near-RT RIC
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

    // Default LM: MRO
    oranHelper->SetDefaultLogicModule(
        "ns3::OranLmLte2LteRsrpHandover",
        "HandoverHoldoffSec",    DoubleValue(mroHoldoffSec),
        "RsrpHysteresisDb",      DoubleValue(mroHysDb),
        "EnableCellControlBias", BooleanValue(true));

    // ES: always present — aggressively reduces TXP (conflict source in all modes)
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteEnergySaving",
        "TargetPowerW",  DoubleValue(esTargetW),
        "StepSize",      DoubleValue(esStepDb),
        "LmIntervalSec", DoubleValue(lmInterval));

    // MLB: always present
    oranHelper->AddLogicModule(
        "ns3::OranLmLte2LteMobilityLoadBalancing",
        "LoadImbalanceThreshold", DoubleValue(mlbThreshold),
        "CioStep",                DoubleValue(1.0),
        "MaxAbsCio",              DoubleValue(6.0));

    // Mode-specific coverage LM and CMM
    if (mode == "reactive")
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteCoverageCapacityOptimization",
            "LowRsrpThresholdDbm",      DoubleValue(ccoRsrpDbm),
            "LowRsrpFractionThreshold", DoubleValue(0.1),
            "StepSize",                 DoubleValue(1.0),
            "MinSamplesPerCell",        UintegerValue(1));
        oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                 "Method", StringValue("priority"));
    }
    else if (mode == "proactive")
    {
        oranHelper->AddLogicModule(
            "ns3::OranLmLte2LteKpiPrediction",
            "ProactiveThresholdDbm", DoubleValue(proactiveThresh),
            "ReactiveThresholdDbm",  DoubleValue(ccoRsrpDbm),
            "EmaAlpha",              DoubleValue(0.4),
            "StepSizeDb",            DoubleValue(1.0),
            "PredictionHorizon",     UintegerValue(2),
            "MinRsrpSamples",        UintegerValue(1));
        oranHelper->SetConflictMitigationModule("ns3::OranCmmConflictTriage",
                                                 "Method", StringValue("priority"));
    }
    else
    {
        // Baseline: ES wins uncontested, no coverage protection
        oranHelper->SetConflictMitigationModule("ns3::OranCmmNoop");
    }

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    // UE E2 terminators
    OranE2NodeTerminatorContainer e2UeTerms;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation>       locRep  = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo>  cellRep = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRep = CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe>  ueTerm  = CreateObject<OranE2NodeTerminatorLteUe>();

        locRep->SetAttribute("Terminator",  PointerValue(ueTerm));
        cellRep->SetAttribute("Terminator", PointerValue(ueTerm));
        rsrpRep->SetAttribute("Terminator", PointerValue(ueTerm));

        for (uint32_t d = 0; d < ueNodes.Get(idx)->GetNDevices(); d++)
        {
            Ptr<LteUeNetDevice> lteDev =
                ueNodes.Get(idx)->GetDevice(d)->GetObject<LteUeNetDevice>();
            if (lteDev)
            {
                lteDev->GetPhy()->TraceConnectWithoutContext(
                    "ReportUeMeasurements",
                    MakeCallback(&OranReporterLteUeRsrpRsrq::ReportRsrpRsrq, rsrpRep));
                lteDev->GetPhy()->TraceConnectWithoutContext(
                    "ReportUeMeasurements",
                    MakeCallback(&ObserveRsrp));
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

    // eNB E2 terminators
    OranE2NodeTerminatorContainer& e2EnbTerms = g_e2EnbTerms;
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation>            locRep    = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteEnergyEfficiency> energyRep =
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

    // Activate RIC and terminators
    Simulator::Schedule(Seconds(1),
                        &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);
    Simulator::Schedule(Seconds(1.5),
                        &OranHelper::ActivateE2NodeTerminators, oranHelper, e2EnbTerms);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&HandoverOk));

    // RSRP violation check and cell-state log: fire 2 s after the first CMM correction
    // (CMM fires ~1 s after LM query), then repeat every lmInterval.
    Simulator::Schedule(Seconds(lmInterval + 2.0), &CheckViolation, Seconds(lmInterval));
    Simulator::Schedule(Seconds(lmInterval + 2.0), &LogCellState,   Seconds(lmInterval));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Final KPI summary
    double   throughputMbps = (g_rxBytesTotal * 8.0) / (simTime * 1e6);
    uint32_t totalCycles    = static_cast<uint32_t>(simTime / lmInterval);

    std::cout << "\n=== PROACTIVE CONFLICT MANAGEMENT -- FINAL KPIs ===" << std::endl;
    std::cout << "  Mode          : " << mode << std::endl;
    std::cout << "  ES target     : " << esTargetW << " W" << std::endl;

    std::cout << "\n  -- Per-eNB state --" << std::endl;
    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> dev = DynamicCast<LteEnbNetDevice>(enbDevs.Get(i));
        uint64_t e2id = e2EnbTerms.Get(i)->GetE2NodeId();
        OranLteCellControlParams cp = GetLteCellControlParameters(e2id);
        double energyJ = initialEnergyJ[i] - enbSrcVec[i]->GetRemainingEnergy();
        std::cout << "  eNB-" << i
                  << "  TXP=" << dev->GetPhy()->GetTxPower() << " dBm"
                  << "  CIO=" << cp.cioDb << " dB"
                  << "  Energy=" << energyJ << " J"
                  << std::endl;
    }

    std::cout << "\n  -- Network-wide KPIs --" << std::endl;
    std::cout << "  DL throughput         : " << throughputMbps << " Mbps" << std::endl;
    std::cout << "  Handovers OK          : " << g_handoverOk << std::endl;
    std::cout << "  RSRP violation cycles : " << g_rsrpViolationCycles
              << " / " << totalCycles << " total cycles" << std::endl;

    std::cout << "\nRESULT:"
              << " mode=" << mode
              << " rsrp_violation_cycles=" << g_rsrpViolationCycles
              << " total_cycles=" << totalCycles
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
