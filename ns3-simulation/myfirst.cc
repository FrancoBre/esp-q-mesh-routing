/*
 * ONGOING
 *
 * copy and paste this code in a fresh ns-3 installation, under
 * scratch/myfirst.cc, build and run /ns3 build && ./ns3 run scratch/myfirst
 *
 * as you may read in the comments I'm fucking dumb and this code is crap but
 * I'm trying to make it work, gotta improve it when that happens
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/yans-wifi-helper.h"

#include <fstream>
#include <iostream>
#include <sstream>

using namespace ns3;

/**
 * Forward declaration of Mesh Test application class because I'm fucking dumb
 * and I don't know how to use CMakeLists and how to link the mf header files
 */

class MeshTest {
public:
  /// Addresses of interfaces:
  Ipv4InterfaceContainer interfaces;

  /// Init test
  MeshTest();
  /**
   * Configure test from command line arguments
   *
   * \param argc command line argument count
   * \param argv command line arguments
   */
  void Configure(int argc, char **argv);
  /**
   * Run test
   * \returns the test status
   */
  int Run();

  void InitializeQLearning();
  void UpdateQTable(uint32_t stateFrom, uint32_t stateTo, double reward);
  uint32_t ChooseAction(uint32_t state);

  uint32_t GetMasterNodeId() const;

  uint32_t GetNodeIdFromAddress(Ipv4Address address) const;

private:
  int m_xSize;             ///< X size
  int m_ySize;             ///< Y size
  double m_step;           ///< step
  double m_randomStart;    ///< random start
  double m_totalTime;      ///< total time
  double m_packetInterval; ///< packet interval
  uint16_t m_packetSize;   ///< packet size
  uint32_t m_nIfaces;      ///< number interfaces
  bool m_chan;             ///< channel
  bool m_pcap;             ///< PCAP
  bool m_ascii;            ///< ASCII
  std::string m_stack;     ///< stack
  std::string m_root;      ///< root
  /// List of network nodes
  NodeContainer nodes;
  /// List of all mesh point devices
  NetDeviceContainer meshDevices;
  /// MeshHelper. Report is not static methods
  MeshHelper mesh;

  // Q-Learning parameters
  double m_alpha;
  double m_gamma;
  double m_epsilon;
  double m_epsilonDecay;

  // Q-Table for each node
  std::map<uint32_t, std::map<uint32_t, double>> qTable;

  // Master node ID
  uint32_t m_masterNodeId;

  std::map<Ipv4Address, uint32_t> addressToNodeIdMap;

private:
  /// Create nodes and setup their mobility
  void CreateNodes();
  /// Install internet m_stack on nodes
  void InstallInternetStack();
  /// Install applications
  void InstallApplication();
  /// Print mesh devices diagnostics
  void Report();
};

/**
 * Forward declaration of Q learning application class because I'm fucking dumb
 * and I don't know how to use CMakeLists and how to link the mf header files
 *
 */

class QLearningApplication : public Application {
public:
  QLearningApplication();
  virtual ~QLearningApplication();
  void Setup(Ptr<Node> node, Ipv4Address sinkAddress, uint16_t port,
             double interval, uint32_t packetSize, MeshTest *meshTest);

private:
  virtual void StartApplication() override;
  virtual void StopApplication() override;
  void ScheduleNextTx();
  void SendPacket();
  void ReceivePacket(Ptr<Socket> socket);

  Ptr<Node> m_node;
  Ipv4Address m_sinkAddress;
  uint16_t m_port;
  double m_interval;
  uint32_t m_packetSize;
  MeshTest *m_meshTest;
  Ptr<Socket> m_socket;
  EventId m_sendEvent;
};

/* ************************************************************************************
 */

// NS_LOG_COMPONENT_DEFINE ("QLearningApplication");

QLearningApplication::QLearningApplication()
    : m_node(nullptr), m_sinkAddress(), m_port(0), m_interval(1.0),
      m_packetSize(1024), m_meshTest(nullptr), m_socket(nullptr) {}

QLearningApplication::~QLearningApplication() { m_socket = nullptr; }

void QLearningApplication::Setup(Ptr<Node> node, Ipv4Address sinkAddress,
                                 uint16_t port, double interval,
                                 uint32_t packetSize, MeshTest *meshTest) {
  m_node = node;
  m_sinkAddress = sinkAddress; // TODO sender address? i think that this node is
                               // the one where the network begins
  m_port = port;
  m_interval = interval;
  m_packetSize = packetSize;
  m_meshTest = meshTest;
}

void QLearningApplication::StartApplication() {
  m_socket = Socket::CreateSocket(
      m_node, TypeId::LookupByName("ns3::UdpSocketFactory"));
  m_socket->Bind();
  m_socket->Connect(InetSocketAddress(
      m_sinkAddress, m_port)); // TODO yup, sink address == sender address
  m_socket->SetRecvCallback(
      MakeCallback(&QLearningApplication::ReceivePacket, this));
  ScheduleNextTx();
}

void QLearningApplication::StopApplication() {
  if (m_socket) {
    m_socket->Close();
  }
  Simulator::Cancel(m_sendEvent);
}

void QLearningApplication::ScheduleNextTx() {
  m_sendEvent = Simulator::Schedule(Seconds(m_interval),
                                    &QLearningApplication::SendPacket, this);
}

// TODO it should send only one packet per episode
void QLearningApplication::SendPacket() {
  Ptr<Packet> packet = Create<Packet>(m_packetSize);
  std::cerr << __func__ << "() -- Sending mf packet" << std::endl;

  m_socket->Send(packet);
  // ScheduleNextTx();
}

void QLearningApplication::ReceivePacket(Ptr<Socket> socket) {
  std::cerr << __func__ << "()" << std::endl;
  std::cerr << std::endl;
  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from))) {
    uint32_t currentNodeId = m_node->GetId();
    InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
    Ipv4Address ipv4Address = address.GetIpv4();
    uint32_t fromNodeId = m_meshTest->GetNodeIdFromAddress(ipv4Address);

    std::cerr << __func__ << "() -- Node " << currentNodeId
              << " received mf packet from " << fromNodeId << std::endl;

    uint32_t nextHop = m_meshTest->ChooseAction(currentNodeId);

    // Check if the current node is the master node
    if (currentNodeId == m_meshTest->GetMasterNodeId()) {
      // Master node: provide a positive reward
      std::cerr << __func__
                << "() -- Found master node! Increasing reward by +100"
                << std::endl;
      // TODO acá tengo que hacer una sobrecarga del updateQTable, porque no hay
      // nextHop si ya encontró el master
      m_meshTest->UpdateQTable(currentNodeId, nextHop, 100.0);
      // TODO acá finaliza el episodio, hay que hacer que termine acá la
      // ejecución
      //  en vez de que sea por tiempo, y que haya un contador que vaya bajando
      //  los episodios
    } else {
      // Non-master node: provide a negative reward
      std::cerr << __func__ << "() -- Non master node, decreasing reward by -1"
                << std::endl;
      m_meshTest->UpdateQTable(currentNodeId, nextHop, -1.0);
    }

    // Forward the packet to the next hop
    std::cerr << __func__ << "() -- Sending packet to: " << nextHop
              << std::endl;
    m_socket->Connect(
        InetSocketAddress(m_meshTest->interfaces.GetAddress(nextHop), m_port));
    m_socket->Send(packet);
  }
}

/* ******************************************************************************
 */

NS_LOG_COMPONENT_DEFINE("MeshExample");

// Declaring these variables outside of main() for use in trace sinks
uint32_t g_udpTxCount = 0; //!< Rx packet counter.
uint32_t g_udpRxCount = 0; //!< Tx packet counter.

/**
 * Transmission trace sink.
 *
 * \param p The sent packet.
 */
void TxTrace(Ptr<const Packet> p) {
  NS_LOG_DEBUG("Sent " << p->GetSize() << " bytes");
  g_udpTxCount++;
}

/**
 * Reception trace sink,
 *
 * \param p The received packet.
 */
void RxTrace(Ptr<const Packet> p) {
  NS_LOG_DEBUG("Received " << p->GetSize() << " bytes");
  g_udpRxCount++;
}

MeshTest::MeshTest()
    : m_xSize(2), m_ySize(2), m_step(50.0), m_randomStart(0.1),
      m_totalTime(100.0), m_packetInterval(1), m_packetSize(1024), m_nIfaces(1),
      m_chan(true), m_pcap(false), m_ascii(false), m_stack("ns3::Dot11sStack"),
      m_root("ff:ff:ff:ff:ff:ff"), m_alpha(0.1), m_gamma(0.9), m_epsilon(0.1),
      m_epsilonDecay(0.1), m_masterNodeId(0) {}

uint32_t MeshTest::GetMasterNodeId() const { return m_masterNodeId; }

uint32_t MeshTest::GetNodeIdFromAddress(Ipv4Address address) const {
  auto it = addressToNodeIdMap.find(address);
  if (it != addressToNodeIdMap.end()) {
    return it->second;
  }
  return -1;
}

void MeshTest::Configure(int argc, char *argv[]) {
  CommandLine cmd(__FILE__);
  cmd.AddValue("x-size", "Number of nodes in a row grid", m_xSize);
  cmd.AddValue("y-size", "Number of rows in a grid", m_ySize);
  cmd.AddValue("step", "Size of edge in our grid (meters)", m_step);
  // Avoid starting all mesh nodes at the same time (beacons may collide)
  cmd.AddValue("start", "Maximum random start delay for beacon jitter (sec)",
               m_randomStart);
  cmd.AddValue("time", "Simulation time (sec)", m_totalTime);
  cmd.AddValue("packet-interval", "Interval between packets in UDP ping (sec)",
               m_packetInterval);
  cmd.AddValue("packet-size", "Size of packets in UDP ping (bytes)",
               m_packetSize);
  cmd.AddValue("interfaces",
               "Number of radio interfaces used by each mesh point", m_nIfaces);
  cmd.AddValue("channels",
               "Use different frequency channels for different interfaces",
               m_chan);
  cmd.AddValue("pcap", "Enable PCAP traces on interfaces", m_pcap);
  cmd.AddValue("ascii", "Enable Ascii traces on interfaces", m_ascii);
  cmd.AddValue("stack", "Type of protocol stack. ns3::Dot11sStack by default",
               m_stack);
  cmd.AddValue("root", "Mac address of root mesh point in HWMP", m_root);

  cmd.Parse(argc, argv);
  NS_LOG_DEBUG("Grid:" << m_xSize << "*" << m_ySize);
  NS_LOG_DEBUG("Simulation time: " << m_totalTime << " s");
  if (m_ascii) {
    PacketMetadata::Enable();
  }
}

void MeshTest::CreateNodes() {
  std::cerr << std::endl;
  std::cerr << "Creating nodes: y_size: " << m_ySize << ", x_size: " << m_xSize
            << std::endl;
  nodes.Create(m_ySize * m_xSize);
  // TODO the sink should be the "master"?
  m_masterNodeId = nodes.Get(m_ySize * m_xSize - 1)
                       ->GetId(); // Set the last node as the master node
  std::cerr << "Master node id is " << m_masterNodeId << std::endl;

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  wifiPhy.SetChannel(wifiChannel.Create());

  // Set transmission range, so not all nodes are neighbors of all other
  // neighbors and there is not a single hop between sender and master
  // wifiPhy.Set("TxPowerStart", DoubleValue(5.0));
  // wifiPhy.Set("TxPowerEnd", DoubleValue(5.0));

  mesh = MeshHelper::Default();
  if (!Mac48Address(m_root.c_str()).IsBroadcast()) {
    mesh.SetStackInstaller(m_stack, "Root",
                           Mac48AddressValue(Mac48Address(m_root.c_str())));
  } else {
    mesh.SetStackInstaller(m_stack);
  }
  if (m_chan) {
    mesh.SetSpreadInterfaceChannels(MeshHelper::SPREAD_CHANNELS);
  } else {
    mesh.SetSpreadInterfaceChannels(MeshHelper::ZERO_CHANNEL);
  }
  mesh.SetMacType("RandomStart", TimeValue(Seconds(m_randomStart)));
  mesh.SetNumberOfInterfaces(m_nIfaces);
  meshDevices = mesh.Install(wifiPhy, nodes);
  mesh.AssignStreams(meshDevices, 0);

  MobilityHelper mobility;
  mobility.SetPositionAllocator(
      "ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY",
      DoubleValue(0.0), "DeltaX",
      // DoubleValue(m_step * 2), // Increase distance between nodes
      DoubleValue(m_step), "DeltaY",
      // DoubleValue(m_step * 2), // Increase distance between nodes
      DoubleValue(m_step), "GridWidth", UintegerValue(m_xSize), "LayoutType",
      StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  if (m_pcap) {
    wifiPhy.EnablePcapAll(std::string("mp"));
  }
  if (m_ascii) {
    AsciiTraceHelper ascii;
    wifiPhy.EnableAsciiAll(ascii.CreateFileStream("mesh.tr"));
  }

  InstallInternetStack();

  // Populate the addressToNodeIdMap
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    Ptr<Node> node = nodes.Get(i);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    Ipv4Address address = ipv4->GetAddress(1, 0).GetLocal();
    addressToNodeIdMap[address] = node->GetId();
  }
}

void MeshTest::InstallInternetStack() {
  InternetStackHelper internetStack;
  internetStack.Install(nodes);
  Ipv4AddressHelper address;
  address.SetBase("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign(meshDevices);
}

void MeshTest::InstallApplication() {
  uint16_t portNumber = 9;
  UdpEchoServerHelper echoServer(portNumber);
  // uint16_t sinkNodeId = m_xSize * m_ySize - 1;
  uint16_t sinkNodeId = 0;
  ApplicationContainer serverApps = echoServer.Install(nodes.Get(sinkNodeId));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(m_totalTime + 1));
  for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    if (i != sinkNodeId) {
      Ptr<QLearningApplication> app = CreateObject<QLearningApplication>();
      app->Setup(nodes.Get(i), interfaces.GetAddress(sinkNodeId), portNumber,
                 m_packetInterval, m_packetSize, this);
      nodes.Get(i)->AddApplication(app);
      app->SetStartTime(Seconds(1.0));
      app->SetStopTime(Seconds(m_totalTime + 1.5));
    }
  }
}

std::ostream &
operator<<(std::ostream &os,
           const std::map<uint32_t, std::map<uint32_t, double>> &qTable) {
  std::cerr << std::endl;
  for (const auto &outerPair : qTable) {
    os << "Node ID " << outerPair.first << ":\n";
    for (const auto &innerPair : outerPair.second) {
      os << "    Neighbor ID " << innerPair.first << ": " << innerPair.second
         << "\n";
    }
  }
  return os;
}

void MeshTest::InitializeQLearning() {
  std::cerr << std::endl;
  std::cerr << __func__ << "() -- Initializing mf q table" << std::endl;
  for (auto i = nodes.Begin(); i != nodes.End(); ++i) {
    uint32_t nodeId = (*i)->GetId();
    for (auto j = nodes.Begin(); j != nodes.End(); ++j) {
      if (i != j) {
        uint32_t neighborId = (*j)->GetId();
        qTable[nodeId][neighborId] = 0.0;
      }
    }
  }
  std::cerr << __func__ << "() -- Q table is:" << qTable << std::endl;
}

void MeshTest::UpdateQTable(uint32_t stateFrom, uint32_t stateTo,
                            double reward) {
  std::cerr << std::endl;
  double currentQ = qTable[stateFrom][stateTo];
  double maxQ = 0.0;
  for (const auto &action : qTable[stateTo]) {
    if (action.second > maxQ) {
      maxQ = action.second;
    }
  }
  qTable[stateFrom][stateTo] =
      currentQ + m_alpha * (reward + m_gamma * maxQ - currentQ);

  // Log the updated Q-Table
  std::cerr << __func__ << "() -- Updated Q-Table for state " << stateFrom
            << " to " << stateTo << ": " << qTable[stateFrom][stateTo]
            << std::endl;
  std::cerr << __func__ << "() -- Q table is:" << qTable << std::endl;
}

uint32_t MeshTest::ChooseAction(uint32_t state) {
  std::cerr << std::endl;
  if ((double)rand() / RAND_MAX < m_epsilon) {
    // Explore: choose a random action
    auto it = qTable[state].begin();
    std::advance(it, rand() % qTable[state].size());

    std::cerr << __func__ << "() -- Exploring: chosen action " << it->first
              << " for state " << state << std::endl;
    return it->first;
  } else {
    // Exploit: choose the action with the highest Q-value
    double maxQ = -std::numeric_limits<double>::infinity();
    uint32_t bestAction = 0;
    for (const auto &action : qTable[state]) {
      if (action.second > maxQ) {
        maxQ = action.second;
        bestAction = action.first;
      }
    }

    std::cerr << __func__ << "() -- Exploiting: chosen action " << bestAction
              << " for state " << state << std::endl;
    return bestAction;
  }
}

int MeshTest::Run() {
  CreateNodes();
  // InstallInternetStack();
  InitializeQLearning();
  InstallApplication();
  Simulator::Schedule(Seconds(m_totalTime), &MeshTest::Report, this);
  Simulator::Stop(Seconds(m_totalTime + 2));
  Simulator::Run();
  Simulator::Destroy();
  // std::cout << "UDP echo packets sent: " << g_udpTxCount << " received: " <<
  // g_udpRxCount
  //           << std::endl;
  return 0;
}

void MeshTest::Report() {
  unsigned n(0);
  for (auto i = meshDevices.Begin(); i != meshDevices.End(); ++i, ++n) {
    /*
    std::ostringstream os;
    os << "mp-report-" << n << ".xml";
    std::cerr << "Printing mesh point device #" << n << " diagnostics to " <<
    os.str() << "\n"; std::ofstream of; of.open(os.str().c_str()); if
    (!of.is_open())
    {
        std::cerr << "Error: Can't open file " << os.str() << "\n";
        return;
    }
    mesh.Report(*i, of);
    of.close();
     */
  }
}

int main(int argc, char *argv[]) {
  MeshTest t;
  t.Configure(argc, argv);
  return t.Run();
}
