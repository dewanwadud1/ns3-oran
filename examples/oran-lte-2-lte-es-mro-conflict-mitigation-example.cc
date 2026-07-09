/**
 * ES/MRO conflict mitigation example for ns-3.42 O-RAN.
 *
 * Exercises: OranLmLte2LteRsrpHandover, OranLmLte2LteEnergySaving,
 *            OranCmmLte2LteEsMro, OranCommandLte2LteTxPower,
 *            RSRP/RSRQ and energy-efficiency reporters.
 *
 * Topology: 2 eNBs placed 50 m apart, 1 UE moving back and forth.
 *           Each eNB has a BasicEnergySource + OranRuDeviceEnergyModel.
 *           The Near-RT RIC runs MRO and ES LMs every 5 s.
 *           The CMM mitigates ES TxPower reductions before execution.
 *           Each eNB E2 terminator sends an energy-efficiency report every 2 s.
 *
 * Expected output:
 *   - Periodic "avgPower" log lines from the LM
 *   - Final RESULT: <eNB0_energy_J> <eNB1_energy_J> <eNB0_txpower_dBm> <eNB1_txpower_dBm>
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranLte2LteEsMroConflictMitigationExample");

static uint32_t g_handoverOkCount = 0;

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

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    g_handoverOkCount++;
}

int
main(int argc, char* argv[])
{
    uint16_t numberOfEnbs = 2;
    uint16_t numberOfUes = 1;
    Time simTime = Seconds(30);
    double distance = 50.0;
    Time interval = Seconds(15);
    double speed = 2.0;
    Time lmQueryInterval = Seconds(5);
    double targetPowerW = 50.0;
    double stepSizeDb = 1.0;
    bool dbLog = false;
    std::string mitigationMethod = "qacm";
    std::string dbFileName = "oran-es-mro-conflict-repository.db";

    CommandLine cmd(__FILE__);
    cmd.AddValue("sim-time", "Simulation duration (s)", simTime);
    cmd.AddValue("target-power-w", "Energy-saving target power per eNB (W)", targetPowerW);
    cmd.AddValue("step-size-db", "TxPower step (dB) per LM decision", stepSizeDb);
    cmd.AddValue("mitigation-method",
                 "CMM method: none, cancel, dampen, priority, nswf, eg, qacm",
                 mitigationMethod);
    cmd.AddValue("db-log", "Print SQL query results", dbLog);
    cmd.Parse(argc, argv);

    LogComponentEnable("OranNearRtRic", (LogLevel)(LOG_PREFIX_TIME | LOG_WARN));
    LogComponentEnable("OranLmLte2LteEnergySaving", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));
    LogComponentEnable("OranCmmLte2LteEsMro", (LogLevel)(LOG_PREFIX_TIME | LOG_INFO));

    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(1000 * 1024));
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    // ── LTE / EPC setup ──────────────────────────────────────────────────────
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
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // ── Nodes and mobility ───────────────────────────────────────────────────
    NodeContainer enbNodes;
    NodeContainer ueNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint16_t i = 0; i < numberOfEnbs; i++)
        positionAlloc->Add(Vector(distance * i, 0, 20));
    for (uint16_t i = 0; i < numberOfUes; i++)
        positionAlloc->Add(Vector((distance / 2) - (speed * (interval.GetSeconds() / 2)), 0, 1.5));

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(enbNodes);

    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
        ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(speed, 0, 0));
    Simulator::Schedule(interval, &ReverseVelocity, ueNodes, interval);

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

    for (uint16_t i = 0; i < numberOfUes; i++)
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(0));

    lteHelper->AddX2Interface(enbNodes);

    // ── UDP traffic (remote host → UE) ───────────────────────────────────────
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
    }
    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime);
    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime - Seconds(1));

    // ── Energy model: BasicEnergySource + OranRuDeviceEnergyModel ────────────
    BasicEnergySourceHelper sourceHelper;
    sourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(500000.0)); // 0.5 MJ
    sourceHelper.Set("BasicEnergySupplyVoltageV",        DoubleValue(48.0));
    energy::EnergySourceContainer sources = sourceHelper.Install(enbNodes);

    std::vector<Ptr<OranRuDeviceEnergyModel>> enbEnergyModels;
    std::vector<Ptr<energy::BasicEnergySource>> enbEnergySources;

    for (uint32_t i = 0; i < enbNodes.GetN(); i++)
    {
        Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        Ptr<LteEnbPhy>       phy    = enbDev->GetPhy();
        Ptr<energy::BasicEnergySource> src =
            DynamicCast<energy::BasicEnergySource>(sources.Get(i));
        NS_ABORT_MSG_IF(src == nullptr, "Energy source is not a BasicEnergySource");

        Ptr<OranRuDeviceEnergyModel> dem = CreateObject<OranRuDeviceEnergyModel>();
        dem->SetEnergySource(src);
        dem->SetLteEnbPhy(phy);

        Ptr<OranRuPowerModel> ru = dem->GetRuPowerModel();
        ru->SetAttribute("NumTrx",         UintegerValue(4));
        ru->SetAttribute("EtaPA",          DoubleValue(0.30));
        ru->SetAttribute("FixedOverheadW", DoubleValue(5.0));
        ru->SetAttribute("DeltaAf",        DoubleValue(0.5));
        ru->SetAttribute("DeltaDC",        DoubleValue(0.07));
        ru->SetAttribute("DeltaMS",        DoubleValue(0.09));
        ru->SetAttribute("DeltaCool",      DoubleValue(0.10));
        ru->SetAttribute("Vdc",            DoubleValue(48.0));
        ru->SetAttribute("SleepPowerW",    DoubleValue(2.0));
        ru->SetAttribute("SleepThresholdDbm", DoubleValue(0.0));

        src->AppendDeviceEnergyModel(dem);
        enbEnergyModels.push_back(dem);
        enbEnergySources.push_back(src);
    }

    // ── Near-RT RIC with MRO + Energy-Saving LMs and ES/MRO CMM ──────────────
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
    oranHelper->SetDefaultLogicModule("ns3::OranLmLte2LteRsrpHandover");
    oranHelper->AddLogicModule("ns3::OranLmLte2LteEnergySaving",
                               "TargetPowerW",
                               DoubleValue(targetPowerW),
                               "StepSize",
                               DoubleValue(stepSizeDb),
                               "LmIntervalSec",
                               DoubleValue(lmQueryInterval.GetSeconds()));
    oranHelper->SetConflictMitigationModule("ns3::OranCmmLte2LteEsMro",
                                            "MitigationMethod",
                                            StringValue(mitigationMethod),
                                            "EsPriority",
                                            DoubleValue(0.70),
                                            "MroPriority",
                                            DoubleValue(1.00));

    Ptr<OranNearRtRic> nearRtRic = oranHelper->CreateNearRtRic();

    if (dbLog)
        nearRtRic->Data()->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));

    // ── UE E2 terminators ────────────────────────────────────────────────────
    OranE2NodeTerminatorContainer e2NodeTerminatorsUes;
    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteUeCellInfo> cellInfoReporter = CreateObject<OranReporterLteUeCellInfo>();
        Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter =
            CreateObject<OranReporterLteUeRsrpRsrq>();
        Ptr<OranE2NodeTerminatorLteUe> ueTerminator = CreateObject<OranE2NodeTerminatorLteUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        cellInfoReporter->SetAttribute("Terminator", PointerValue(ueTerminator));
        rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(ueTerminator));

        for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
        {
            Ptr<LteUeNetDevice> lteUeDevice =
                ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
            if (lteUeDevice)
            {
                Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
                uePhy->TraceConnectWithoutContext(
                    "ReportUeMeasurements",
                    MakeCallback(&ns3::OranReporterLteUeRsrpRsrq::ReportRsrpRsrq,
                                 rsrpRsrqReporter));
            }
        }

        ueTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        ueTerminator->SetAttribute("RegistrationIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        ueTerminator->SetAttribute("SendIntervalRv",
                                   StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        ueTerminator->AddReporter(locationReporter);
        ueTerminator->AddReporter(cellInfoReporter);
        ueTerminator->AddReporter(rsrpRsrqReporter);
        ueTerminator->Attach(ueNodes.Get(idx));

        e2NodeTerminatorsUes.Add(ueTerminator);
        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorLteUe::Activate, ueTerminator);
    }

    // ── eNB E2 terminators (manual, to wire energy source) ───────────────────
    OranE2NodeTerminatorContainer e2NodeTerminatorsEnbs;
    for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterLteEnergyEfficiency> energyReporter =
            CreateObject<OranReporterLteEnergyEfficiency>();
        Ptr<OranE2NodeTerminatorLteEnb> enbTerminator = CreateObject<OranE2NodeTerminatorLteEnb>();

        locReporter->SetAttribute("Terminator",  PointerValue(enbTerminator));
        locReporter->SetAttribute("Trigger",
                                  StringValue("ns3::OranReportTriggerPeriodic"));

        energyReporter->SetEnergySource(enbEnergySources[idx]);
        energyReporter->SetAttribute("Terminator", PointerValue(enbTerminator));
        energyReporter->SetAttribute("Trigger",
                                     StringValue("ns3::OranReportTriggerPeriodic"));

        enbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        enbTerminator->SetAttribute("RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        enbTerminator->SetAttribute("SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=2]"));

        enbTerminator->AddReporter(locReporter);
        enbTerminator->AddReporter(energyReporter);
        enbTerminator->Attach(enbNodes.Get(idx));

        e2NodeTerminatorsEnbs.Add(enbTerminator);
        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorLteEnb::Activate, enbTerminator);
    }

    // ── Activate O-RAN ───────────────────────────────────────────────────────
    Simulator::Schedule(Seconds(1), &OranHelper::ActivateAndStartNearRtRic, oranHelper, nearRtRic);

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));

    // ── Run ──────────────────────────────────────────────────────────────────
    Simulator::Stop(simTime);
    Simulator::Run();

    // ── Final KPIs ───────────────────────────────────────────────────────────
    std::cout << "\nRESULT: mitigation_method " << mitigationMethod;
    std::cout << "\nRESULT: handover_end_ok_count " << g_handoverOkCount;
    std::cout << "\nRESULT: energy_consumed_J";
    for (uint32_t i = 0; i < enbEnergyModels.size(); i++)
        std::cout << "  eNB" << i << "=" << enbEnergyModels[i]->GetTotalEnergyConsumption();
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
