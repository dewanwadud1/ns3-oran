/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Abdul Wadud
 * Affiliation: University College Dublin, Ireland.
 */
/**
 * NR (5G) counterpart of oran-lte-2-lte-distance-handover-example.cc.
 * Manual (non-OranHelper) RIC wiring; OranLmNr2NrDistanceHandover.
 */

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

#include <stdio.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrDistanceHandoverExample");

void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().GetSeconds() << " " << context << " gNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
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

int
main(int argc, char* argv[])
{
    uint16_t numberOfUes = 1;
    uint16_t numberOfGnbs = 2;
    Time simTime = Seconds(50);
    double distance = 200;
    Time interval = Seconds(15);
    double speed = 5;
    bool verbose = false;
    std::string dbFileName = "oran-nr-distance-handover-repository.db";

    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
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

    NodeContainer ueNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

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
        Ptr<ConstantVelocityMobilityModel> mobility =
            ueNodes.Get(idx)->GetObject<ConstantVelocityMobilityModel>();
        mobility->SetVelocity(Vector(speed, 0, 0));
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
    }
    nrHelper->UpdateDeviceConfigs(gnbNetDev);
    nrHelper->UpdateDeviceConfigs(ueNetDev);
    nrHelper->AddX2Interface(gnbNodes);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces;
    ueIpIfaces = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        nrHelper->AttachToGnb(ueNetDev.Get(i), gnbNetDev.Get(0));
    }

    // ── ORAN Models -- BEGIN (manual, non-OranHelper wiring) ────────────────
    if (!dbFileName.empty())
    {
        std::remove(dbFileName.c_str());
    }
    Ptr<OranDataRepository> dataRepository = CreateObject<OranDataRepositorySqlite>();
    Ptr<OranLm> defaultLm = CreateObject<OranLmNr2NrDistanceHandover>();
    Ptr<OranCmm> cmm = CreateObject<OranCmmNoop>();
    Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
    Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator =
        CreateObject<OranNearRtRicE2Terminator>();

    dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
    if (verbose)
    {
        dataRepository->TraceConnectWithoutContext("QueryRc", MakeCallback(&QueryRcSink));
    }

    defaultLm->SetAttribute("Verbose", BooleanValue(true));
    defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    defaultLm->SetAttribute("ProcessingDelayRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    cmm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    cmm->SetAttribute("Verbose", BooleanValue(true));

    nearRtRicE2Terminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
    nearRtRicE2Terminator->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRicE2Terminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

    nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(defaultLm));
    nearRtRic->SetAttribute("E2Terminator", PointerValue(nearRtRicE2Terminator));
    nearRtRic->SetAttribute("DataRepository", PointerValue(dataRepository));
    nearRtRic->SetAttribute("LmQueryInterval", TimeValue(Seconds(5)));
    nearRtRic->SetAttribute("ConflictMitigationModule", PointerValue(cmm));
    nearRtRic->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
    nearRtRic->SetAttribute("E2NodeInactivityIntervalRv",
                            StringValue("ns3::ConstantRandomVariable[Constant=2]"));
    nearRtRic->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(0)));
    nearRtRic->SetAttribute("LmQueryLateCommandPolicy", EnumValue(OranNearRtRic::DROP));

    Simulator::Schedule(Seconds(1), &OranNearRtRic::Start, nearRtRic);

    for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter =
            CreateObject<OranReporterNrUeCellInfo>();
        Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

        locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
        nrUeCellInfoReporter->SetAttribute(
            "Trigger",
            StringValue("ns3::OranReportTriggerNrUeHandover[InitialReport=true]"));

        nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                     StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrUeTerminator->SetAttribute("SendIntervalRv",
                                     StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrUeTerminator->SetAttribute("TransmissionDelayRv",
                                     StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

        nrUeTerminator->AddReporter(locationReporter);
        nrUeTerminator->AddReporter(nrUeCellInfoReporter);

        nrUeTerminator->Attach(ueNodes.Get(idx));

        Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
    }

    for (uint32_t idx = 0; idx < gnbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator =
            CreateObject<OranE2NodeTerminatorNrGnb>();

        locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
        locationReporter->SetAttribute("Trigger", StringValue("ns3::OranReportTriggerPeriodic"));

        nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrGnbTerminator->SetAttribute("SendIntervalRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                      StringValue("ns3::ConstantRandomVariable[Constant=0.001]"));

        nrGnbTerminator->AddReporter(locationReporter);

        nrGnbTerminator->Attach(gnbNodes.Get(idx));

        Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, nrGnbTerminator);
    }

    // ── ORAN Models -- END ───────────────────────────────────────────────────

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

    Simulator::Stop(simTime);
    Simulator::Run();

    Simulator::Destroy();
    return 0;
}
