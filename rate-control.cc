/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

/**
 * This example program is designed to illustrate the behavior of
 * rate-adaptive WiFi rate controls such as Minstrel.  Power-adaptive
 * rate controls can be illustrated also, but separate examples exist for
 * highlighting the power adaptation.
 *
 * This simulation consist of 2 nodes, one AP and one STA.
 * The AP generates UDP traffic with a CBR of 54 Mbps to the STA.
 * The AP can use any power and rate control mechanism and the STA uses
 * only Minstrel rate control.
 * The STA can be configured to move away from (or towards to) the AP.
 * By default, the AP is at coordinate (0,0,0) and the STA starts at
 * coordinate (5,0,0) (meters) and moves away on the x axis by 1 meter every
 * second.
 *
 * The output consists of:
 * - A plot of average throughput vs. distance.
 * - (if logging is enabled) the changes of rate to standard output.
 *
 * Example usage:
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel"
 *
 * Another example (moving towards the AP):
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel --stepsSize=1 --STA1_x=-200"
 *
 * Example for HT rates with SGI and channel width of 40MHz:
 * ./ns3 run "wifi-rate-adaptation-distance --staManager=ns3::MinstrelHtWifiManager
 * --apManager=ns3::MinstrelHtWifiManager --outputFileName=minstrelHt --shortGuardInterval=true
 * --channelWidth=40"
 *
 * To enable the log of rate changes:
 * export NS_LOG=RateAdaptationDistance=level_info
 */

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/gnuplot.h"
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/propagation-loss-model.h"
#include <fstream>
#include <sys/stat.h> // for mkdir
#include <string>
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RateAdaptationDistance");

std::ofstream g_csvFile;


void
EndSimulation()
{
  g_csvFile.close();
}

/** Node statistics */
class NodeStatistics
{
  public:
    /**
     * Constructor
     * \param aps AP devices
     * \param stas STA devices
     */
    NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

    /**
     * RX callback
     * \param path path
     * \param packet received packet
     * \param from sender
     */
    void RxCallback(std::string path, Ptr<const Packet> packet, const Address& from);
    /**
     * Set node position
     * \param node the node
     * \param position the position
     */
    void SetPosition(Ptr<Node> node, Vector position);
    /**
     * Advance node position
     * \param node the node
     * \param stepsSize the size of a step
     * \param stepsTime the time interval between steps
     */
    void AdvancePosition(Ptr<Node> node, double stepsSize, int stepsTime);
    /**
     * Get node position
     * \param node the node
     * \return the position
     */
    void RandomMoveWithinRectangle(Ptr<Node> node);
    Vector GetPosition(Ptr<Node> node);
    /**
     * \return the gnuplot 2d dataset
     */
    Gnuplot2dDataset GetDatafile();

  private:
    uint32_t m_bytesTotal;     //!< total bytes
    Gnuplot2dDataset m_output; //!< gnuplot 2d dataset
};

NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
    m_bytesTotal = 0;
}

void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address& from)
{
    m_bytesTotal += packet->GetSize();
}

void
NodeStatistics::SetPosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    mobility->SetPosition(position);
}

Vector
NodeStatistics::GetPosition(Ptr<Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    return mobility->GetPosition();
}

void
NodeStatistics::AdvancePosition(Ptr<Node> node, double stepsSize, int stepsTime)
{
    Vector pos = GetPosition(node);
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
    m_bytesTotal = 0;
    m_output.Add(pos.x, mbs);
    pos.x += stepsSize;
    SetPosition(node, pos);
    Simulator::Schedule(Seconds(1*stepsTime),
                        &NodeStatistics::AdvancePosition,
                        this,
                        node,
                        stepsSize,
                        stepsTime);
}

void
NodeStatistics::RandomMoveWithinRectangle(Ptr<Node> node)
{
    double xMin = 10.0;
    double xMax = 30.0;
    double yMin = -20.0;
    double yMax = 20.0;

    Ptr<UniformRandomVariable> xRandom = CreateObject<UniformRandomVariable>();
    xRandom->SetAttribute("Min", DoubleValue(xMin));
    xRandom->SetAttribute("Max", DoubleValue(xMax));

    Ptr<UniformRandomVariable> yRandom = CreateObject<UniformRandomVariable>();
    yRandom->SetAttribute("Min", DoubleValue(yMin));
    yRandom->SetAttribute("Max", DoubleValue(yMax));

    double xPos = xRandom->GetValue();
    double yPos = yRandom->GetValue();

    SetPosition(node, Vector(xPos, yPos, 0.0));
    std::cout<<"Position: "<<Vector(xPos, yPos, 0.0)<<std::endl;

    Simulator::Schedule(Seconds(5), &NodeStatistics::RandomMoveWithinRectangle, this, node);
}


Gnuplot2dDataset
NodeStatistics::GetDatafile()
{
    return m_output;
}

/**
 * Callback for 'Rate' trace source
 *
 * \param oldRate old MCS rate (bits/sec)
 * \param newRate new MCS rate (bits/sec)
 */
void
RateCallback(uint64_t oldRate, uint64_t newRate)
{
    NS_LOG_INFO("Rate " << newRate / 1000000.0 << " Mbps");
}
struct DataForThpt
{
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  uint32_t totalRxPackets; //Total number of received packets in all flows
  uint64_t totalRxBytes; // Total bytes received in all flows
  double totalDelaySum; // Total delay sum in all flows
  double
  averageDelay ()
  {
    return totalRxPackets ? totalDelaySum / totalRxPackets / 100000000 : 0;
  }
} data; 

double duration = 1.0; // Duration of simulation (s)
double statInterval = 0.5; // Time interval of calling function Throughput

void 
CalculateThroughput(Ptr<PacketSink> sink1, Ptr<PacketSink> sink2, double lastTotalRx[2], double interval)
{
    double currentRx1 = sink1->GetTotalRx();
    double currentRx2 = sink2->GetTotalRx();

    double throughput1 = (currentRx1 - lastTotalRx[0]) * 8.0 / interval / (1024 * 1024);
    double throughput2 = (currentRx2 - lastTotalRx[1]) * 8.0 / interval / (1024 * 1024);

    lastTotalRx[0] = currentRx1;
    lastTotalRx[1] = currentRx2;

    double timeNow = Simulator::Now().GetSeconds();
    g_csvFile << timeNow << " " << throughput1 << std::endl;
    Simulator::Schedule(Seconds(interval), &CalculateThroughput, sink1, sink2, lastTotalRx, interval);
}

int
main(int argc, char* argv[])
{
    uint32_t rtsThreshold = 65535;
    std::string staManager = "ns3::rl-rateWifiManager";
    std::string apManager = "ns3::rl-rateWifiManager";
    std::string interferenceManager = "ns3::MinstrelHtWifiManager";
    std::string standard = "802.11n-5GHz";
    std::string outputFileName = "rl-rate-interference";
    uint32_t BeMaxAmpduSize = 65535;//Disable the A-MPDU
    bool shortGuardInterval = false;
    uint32_t chWidth = 20;
    int ap_num = 1;
    int ap1_x = 0;
    int ap1_y = 0;
    int sta1_x = 5;
    int sta1_y = 0;
    int steps = 80;
    double stepsSize = 1;
    int stepsTime = 1;
    int run = 1;
    int seed = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("staManager", "Rate adaptation manager of the STA", staManager);
    cmd.AddValue("apManager", "Rate adaptation manager of the AP", apManager);
    cmd.AddValue("standard", "Wifi standard (a/b/g/n/ac only)", standard);
    cmd.AddValue("shortGuardInterval",
                 "Enable Short Guard Interval in all stations",
                 shortGuardInterval);
    cmd.AddValue("channelWidth", "Channel width of all the stations", chWidth);
    cmd.AddValue("rtsThreshold", "RTS threshold", rtsThreshold);
    cmd.AddValue("BeMaxAmpduSize", "BE Mac A-MPDU size", BeMaxAmpduSize);
    cmd.AddValue("outputFileName", "Output filename", outputFileName);
    cmd.AddValue("steps", "How many different distances to try", steps);
    cmd.AddValue("stepsTime", "Time on each step", stepsTime);
    cmd.AddValue("stepsSize", "Distance between steps", stepsSize);
    cmd.AddValue("AP1_x", "Position of AP1 in x coordinate", ap1_x);
    cmd.AddValue("AP1_y", "Position of AP1 in y coordinate", ap1_y);
    cmd.AddValue("STA1_x", "Position of STA1 in x coordinate", sta1_x);
    cmd.AddValue("STA1_y", "Position of STA1 in y coordinate", sta1_y);
    cmd.AddValue("APNUM", "AP for interference", ap_num);
    cmd.AddValue("Run", "Random Seed", run);
    cmd.AddValue("Seed", "Random Seed", seed);
    cmd.Parse(argc, argv);

    int simuTime = steps * stepsTime;

    if (standard != "802.11a" && standard != "802.11b" && standard != "802.11g" &&
        standard == "802.11n-2.4GHz" && standard != "802.11n-5GHz" && standard != "802.11ac")
    {
        NS_FATAL_ERROR("Standard " << standard << " is not supported by this program");
    }

    RngSeedManager::SetSeed(seed); 
    std::cout<<"Random seed with"<<seed<<std::endl;
    RngSeedManager::SetRun(run); 
    InitStream(run);
    NodeContainer wifiApNodes;
    wifiApNodes.Create(3);

    // Define the STAs
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(3);

    // Interference APs
    NodeContainer wifiInterferenceApNodes;
    wifiInterferenceApNodes.Create(4);

    std::string errorModelType = "ns3::NistErrorRateModel"; // Error Model
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

    // Set Propagation Loss Model
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::MatrixPropagationLossModel","DefaultLoss",DoubleValue(20.0));

    // Set Transmission Power
    wifiPhy.Set ("TxPowerStart", DoubleValue (20.0));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (20.0));

    // Create a loss model and set default loss
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.SetErrorRateModel (errorModelType);

    // Channel configuration via ChannelSettings attribute can be performed here
    std::string frequencyBand;
    if (standard == "802.11b" || standard == "802.11g" || standard == "802.11n-2.4GHz")
    {
        frequencyBand = "BAND_2_4GHZ";
    }
    else
    {
        frequencyBand = "BAND_5GHZ";
    }
    wifiPhy.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(chWidth) + ", " + frequencyBand + ", 0}"));

    wifiPhy.Set("CcaSensitivity", DoubleValue(-9999));
    wifiPhy.Set("RxNoiseFigure", DoubleValue(0));
    wifiPhy.DisablePreambleDetectionModel();

    NetDeviceContainer wifiApDevices;
    NetDeviceContainer wifiStaDevices;
    NetDeviceContainer wifiInterferenceDevices;
    NetDeviceContainer wifiDevices;

    WifiHelper wifi;

    if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz" || standard == "802.11ac")
    {
        if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz")
        {
            wifi.SetStandard(WIFI_STANDARD_80211n);
        }
        else if (standard == "802.11ac")
        {
            wifi.SetStandard(WIFI_STANDARD_80211ac);
        }

        WifiMacHelper wifiMac;

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                   "DataMode", StringValue("HtMcs1"), // Use MCS 1 for data
                                   "ControlMode", StringValue("HtMcs1"),
                                   "RtsCtsThreshold", UintegerValue(rtsThreshold));

        Ssid ssid = Ssid("AP");
        wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes));

        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                            "DataMode", StringValue("HtMcs7"), // Use MCS 1 for data
                            "ControlMode", StringValue("HtMcs1"),
                            "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(0)));
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                            "DataMode", StringValue("HtMcs7"), // Use MCS 1 for data
                            "ControlMode", StringValue("HtMcs1"),
                            "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(1)));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes.Get(2)));

        
        ssid = Ssid("InterferenceAP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiInterferenceDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiInterferenceApNodes));

        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(BeMaxAmpduSize));
    }

    wifiDevices.Add(wifiStaDevices);
    wifiDevices.Add(wifiApDevices);
    wifiDevices.Add(wifiInterferenceDevices);


    // Set guard interval
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        BooleanValue(shortGuardInterval));

    // Configure the mobility.
    MobilityHelper mobilitySta;
    mobilitySta.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (9.5),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (0.1),
                                 "DeltaY", DoubleValue (0.1),
                                 "GridWidth", UintegerValue (2),
                                 "LayoutType", StringValue ("RowFirst"));
    mobilitySta.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                                "Bounds", StringValue("0|20|0|0.1"),
                                "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
    mobilitySta.Install (wifiStaNodes);

    MobilityHelper mobilityAp;
    mobilityAp.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (19.9),
                                 "DeltaY", DoubleValue (0.0),
                                 "GridWidth", UintegerValue (2),
                                 "LayoutType", StringValue ("RowFirst"));
    mobilityAp.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install (wifiApNodes);

    MobilityHelper mobilityInterferenceAp;
    mobilityInterferenceAp.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (10.0),
                                 "MinY", DoubleValue (5.0),
                                 "DeltaX", DoubleValue (1),
                                 "DeltaY", DoubleValue (1),
                                 "GridWidth", UintegerValue (2),
                                 "LayoutType", StringValue ("RowFirst"));
    mobilityInterferenceAp.SetMobilityModel ("ns3::ConstantPositionMobilityModel");   
    mobilityInterferenceAp.Install (wifiInterferenceApNodes);

    Ptr<MobilityModel> mobility = wifiApNodes.Get(2)->GetObject<MobilityModel>();
    Vector pos = mobility->GetPosition();
    pos.x = 9.5;
    pos.y = 9.6;
    mobility->SetPosition(pos);

    for (uint32_t i = 0; i < wifiInterferenceApNodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mobility = wifiInterferenceApNodes.Get(i)->GetObject<MobilityModel>();
      std::cout<<"Interference AP "<<i<<" position: "<<mobility->GetPosition()<<std::endl;
    }
    for (uint32_t i = 0; i < wifiStaNodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mobility = wifiStaNodes.Get(i)->GetObject<MobilityModel>();
      std::cout<<"STA "<<i<<" position: "<<mobility->GetPosition()<<std::endl;
    }
    for (uint32_t i = 0; i < wifiApNodes.GetN(); ++i)
    {
      Ptr<MobilityModel> mobility = wifiApNodes.Get(i)->GetObject<MobilityModel>();
      std::cout<<"AP "<<i<<" position: "<<mobility->GetPosition()<<std::endl;
    }

    // Statistics counter
    NodeStatistics atpCounter = NodeStatistics(wifiApDevices, wifiStaDevices);

    // Move the AP by stepsSize meters every stepsTime seconds
    Simulator::Schedule(Seconds(0.5 + stepsTime),
                        &NodeStatistics::AdvancePosition,
                        &atpCounter,
                        wifiApNodes.Get(1),
                        stepsSize,
                        stepsTime);

    // Move the AP to a random position within a rectangle every 5 seconds
    Simulator::Schedule(Seconds(1), &NodeStatistics::RandomMoveWithinRectangle, &atpCounter, wifiApNodes.Get(1));


    // Move the InferenceNode by stepsSize meters every stepsTime seconds
    Simulator::Schedule(Seconds(0.5 + stepsTime),
                        &NodeStatistics::AdvancePosition,
                        &atpCounter,
                        wifiInterferenceApNodes.Get(0),
                        stepsSize,
                        stepsTime);

    // Configure the IP stack
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);
    stack.Install(wifiInterferenceApNodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer address_sta = address.Assign(wifiStaDevices);
    Ipv4InterfaceContainer j = address.Assign(wifiApDevices);
    Ipv4InterfaceContainer k = address.Assign(wifiInterferenceDevices);
    Ipv4Address sinkAddress = address_sta.GetAddress(0);
    Ipv4Address sinkAddress_2 = address_sta.GetAddress(1);
    Ipv4Address sinkAddress_3 = address_sta.GetAddress(2);
    Ipv4Address sinkAddress_4 = address_sta.GetAddress(3);

    std::cout<<"ip of Ap 1 is: "<<j.GetAddress(0)<<std::endl;
    std::cout<<"AP0 Mac address is: "<<wifiApDevices.Get(0)->GetAddress()<<std::endl;
    

    uint16_t port = 9;
    uint16_t port2 = 10;
    uint16_t port3 = 12;
    uint16_t port4 = 14;
    std::vector<uint16_t> ports;
    ports.push_back(port);
    ports.push_back(port2);
    ports.push_back(port3);
    ports.push_back(port4);

    // Configure the CBR generator
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    ApplicationContainer apps_sink = sink.Install(wifiStaNodes.Get(0));
    PacketSinkHelper sink_2("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress_2, port2));
    ApplicationContainer apps_sink_2 = sink_2.Install(wifiStaNodes.Get(1));
    PacketSinkHelper sink_3("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress_3, port3));
    ApplicationContainer apps_sink_3 = sink_3.Install(wifiStaNodes.Get(2));
    PacketSinkHelper sink_4("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress_4, port4));
    ApplicationContainer apps_sink_4 = sink_4.Install(wifiStaNodes.Get(3));

    // Configure Onoff for AP 1
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    onoff.SetConstantRate(DataRate("60Mbps"), 1420);
    ApplicationContainer apps_source = onoff.Install(wifiApNodes.Get(0));
    apps_source.Start(Seconds(0.5));
    apps_source.Stop(Seconds(simuTime));

    //Configure Onoff for AP 2,3,4
    for (uint32_t i = 1; i <wifiApNodes.GetN(); ++i)
    {
      std::cout<<"------\ninstalling onoff on ap: "<<i<<std::endl;
      OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(address_sta.GetAddress(1), ports[1]));
      onoff.SetConstantRate(DataRate("60Mbps"), 1420);
      ApplicationContainer apps_source_2 = onoff.Install(wifiApNodes.Get(i));
      apps_source_2.Start(Seconds(0.5));
      apps_source_2.Stop(Seconds(simuTime));
    }
    OnOffHelper onoff_2("ns3::UdpSocketFactory", InetSocketAddress(address_sta.GetAddress(2), ports[2]));
    onoff_2.SetConstantRate(DataRate("60Mbps"), 1420);
    ApplicationContainer apps_source_2 = onoff_2.Install(wifiApNodes.Get(2));
    apps_source_2.Start(Seconds(0.5));
    apps_source_2.Stop(Seconds(simuTime));



    for (uint32_t i = 0; i < wifiInterferenceApNodes.GetN(); ++i)
    {
      std::cout<<"------\ninstalling onoff on interference ap: "<<i<<std::endl;
      OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(address_sta.GetAddress(1), ports[1]));
      onoff.SetConstantRate(DataRate("60Mbps"), 1420);
      ApplicationContainer apps_source_3 = onoff.Install(wifiInterferenceApNodes.Get(i));
      apps_source_3.Start(Seconds(0.5));
      apps_source_3.Stop(Seconds(simuTime));
    }

    //------------------------------------------------------------
    //-- Setup stats and data collection
    //--------------------------------------------

    // // Register packet receptions to calculate throughput
    Config::Connect("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
                    MakeCallback(&NodeStatistics::RxCallback, &atpCounter));

    // // Callbacks to print every change of rate
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + apManager + "/Rate",
        MakeCallback(RateCallback));
    

    data.monitor = data.flowmon.InstallAll();
    data.totalDelaySum = 0;
    data.totalRxBytes = 0;
    data.totalRxPackets = 0;
    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (apps_sink.Get (0));
    Ptr<PacketSink> sink2 = DynamicCast<PacketSink> (apps_sink_3.Get (0));
    double lastTotalRx[2] = {0.0, 0.0}; // initial values
    double interval = 1; // every 1 second
    Simulator::Schedule(Seconds(2), &CalculateThroughput, sink1, sink2, lastTotalRx, interval);

    Simulator::Stop(Seconds(simuTime));
    Simulator::Run();

    double avgThroughputSta0 = sink1->GetTotalRx() * 8.0 / simuTime / (1024 * 1024);
    std::cout << "Average Delay: " << data.averageDelay () << "ms" << std::endl;
    std::cout << "Average Throughput (Sta 0): " << avgThroughputSta0
        << "Mbps" << std::endl;

    std::ofstream outFile;
    outFile.open("avg_throughput_sta0.txt", std::ios_base::app);  // Open in append mode
    if(outFile.is_open())
    {
        outFile << avgThroughputSta0 << std::endl;
        outFile.close();
    }
    else
    {
        std::cerr << "Unable to open file for writing!" << std::endl;
    }
    EndSimulation();

    std::ofstream outfile("throughput-" + outputFileName + ".plt");
    Gnuplot gnuplot = Gnuplot("throughput-" + outputFileName + ".eps", "Throughput");
    gnuplot.SetTerminal("post eps color enhanced");
    gnuplot.SetLegend("Time (seconds)", "Throughput (Mb/s)");
    gnuplot.SetTitle("Throughput (AP to STA) vs time");
    gnuplot.AddDataset(atpCounter.GetDatafile());
    gnuplot.GenerateOutput(outfile);

    Simulator::Destroy();


    return 0;
}