import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SENDER = (ROOT / "firmware/sender-node/sender-node.ino").read_text()
INTERMEDIATE = (ROOT / "firmware/intermediate-node/intermediate-node.ino").read_text()
RECEIVER = (ROOT / "firmware/receiver-node/receiver-node.ino").read_text()
COMMON = (ROOT / "lib/QMeshCommon/src/q_mesh_utils.h").read_text()


def function_body(source: str, signature: str) -> str:
    # Sketches keep explicit declarations near the top; the implementation is
    # the last occurrence of the full signature.
    start = source.rindex(signature)
    brace = source.index("{", start)
    depth = 0
    for pos in range(brace, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : pos]
    raise AssertionError(f"unterminated function: {signature}")


class NodeModel:
    """Small executable model of the firmware's local-row contract."""

    def __init__(self, node_id, eta=0.7):
        self.node_id = node_id
        self.eta = eta
        self.q = {}

    def initialize(self, neighbors):
        for neighbor in neighbors:
            self.q.setdefault(neighbor, 0.0)

    def choose(self, neighbors):
        self.initialize(neighbors)
        return min(neighbors, key=self.q.__getitem__)

    def estimate(self, neighbors):
        self.initialize(neighbors)
        return min(self.q[neighbor] for neighbor in neighbors)

    def feedback(self, via, step_time, estimate_t):
        old_q = self.q.setdefault(via, 0.0)
        self.q[via] = old_q + self.eta * (step_time + estimate_t - old_q)


def direct_neighbors(topology):
    """Model asNodeTree().subs: only first-level connections are actions."""
    return [connection["node_id"] for connection in topology["subs"]]


def make_packet(packet_id, max_hops=24):
    return {
        "type": "PACKET_HOP",
        "current_packet_id": packet_id,
        "hops_remaining": max_hops,
        "packets": [{"packet_id": packet_id, "steps": []}],
    }


class LocalQRoutingBehaviorTests(unittest.TestCase):
    def test_only_first_level_topology_nodes_are_actions(self):
        topology = {
            "node_id": 1,
            "subs": [
                {"node_id": 2, "subs": [{"node_id": 3, "subs": []}]},
                {"node_id": 4, "subs": []},
            ],
        }
        self.assertEqual(direct_neighbors(topology), [2, 4])
        self.assertNotIn(3, direct_neighbors(topology))

    def test_disconnection_and_reconnection_preserve_learned_value(self):
        node = NodeModel(1)
        node.q[4208810825] = 3.5
        self.assertEqual(node.choose([4208810825]), 4208810825)
        self.assertEqual(node.choose([7]), 7)
        self.assertEqual(node.choose([4208810825]), 4208810825)
        self.assertEqual(node.q[4208810825], 3.5)

    def test_unsigned_node_id_remains_canonical(self):
        node_id = 4208810825
        self.assertGreater(node_id, 2**31 - 1)
        self.assertEqual(str(node_id), "4208810825")
        self.assertNotEqual(str(node_id), "-86156471")

    def test_packet_creation_contains_only_current_packet(self):
        first = make_packet(1)
        second = make_packet(2)
        self.assertEqual(first["packets"], [{"packet_id": 1, "steps": []}])
        self.assertEqual(second["packets"], [{"packet_id": 2, "steps": []}])

    def test_packet_ttl_exhaustion_is_bounded(self):
        packet = make_packet(1, max_hops=2)
        forwarded = 0
        while packet["hops_remaining"] > 0:
            packet["hops_remaining"] -= 1
            forwarded += 1
        self.assertEqual(forwarded, 2)
        self.assertEqual(packet["hops_remaining"], 0)

    def test_feedback_persists_and_packet_telemetry_cannot_override_it(self):
        b = NodeModel("B")
        b.q.update({"C": 4.0, "D": 9.0})
        b.feedback("C", step_time=1.0, estimate_t=0.0)
        learned = b.q["C"]

        packet_q_table = {"B": {"C": 99.0, "D": 0.0}}
        self.assertEqual(b.choose(["C", "D"]), "C")
        self.assertEqual(b.q["C"], learned)
        self.assertEqual(packet_q_table["B"]["C"], 99.0)

    def test_nodes_learn_independently(self):
        a = NodeModel("A")
        b = NodeModel("B")
        a.q["B"] = 3.0
        b.q["C"] = 4.0

        b.feedback("C", step_time=1.0, estimate_t=0.0)
        self.assertEqual(a.q, {"B": 3.0})
        self.assertNotEqual(b.q["C"], 4.0)

    def test_b_feedback_to_a_uses_b_local_estimate(self):
        a = NodeModel("A")
        b = NodeModel("B")
        b.q.update({"C": 2.0, "D": 8.0})

        estimate_t = b.estimate(["C", "D"])
        a.feedback("B", step_time=1.0, estimate_t=estimate_t)
        self.assertAlmostEqual(estimate_t, 2.0)
        self.assertAlmostEqual(a.q["B"], 2.1)

    def test_destination_feedback_has_zero_remaining_time(self):
        b = NodeModel("B")
        b.feedback("C", step_time=2.0, estimate_t=0.0)
        self.assertAlmostEqual(b.q["C"], 1.4)


class FirmwareContractTests(unittest.TestCase):
    def test_action_selection_reads_persistent_local_table(self):
        for source in (SENDER, INTERMEDIATE):
            body = function_body(source, "NodeId chooseBestAction(")
            self.assertIn("g_qTable[nodeIdKey(g_nodeId)]", body)
            self.assertNotIn('doc["q_table"]', body)
            self.assertIn("initializeLocalActions(neighbors)", body)

    def test_remaining_time_reads_only_selectable_local_actions(self):
        for source in (SENDER, INTERMEDIATE):
            body = function_body(source, "float estimateRemainingTime(")
            self.assertIn("g_qTable[nodeIdKey(g_nodeId)]", body)
            self.assertIn("for (const auto &neighbor : neighbors)", body)
            self.assertNotIn('doc["q_table"]', body)

    def test_packet_table_is_not_imported_into_persistent_table(self):
        for source in (SENDER, INTERMEDIATE):
            self.assertNotIn("copyQTableToPersistent", source)
            copy_body = function_body(source, "void copyLocalActionsToPacket(")
            self.assertRegex(copy_body, r"packetActions\[.*\]\s*=.*action\.value")
            self.assertNotRegex(copy_body, r"g_qTable\[.*\]\s*=.*packet")

    def test_feedback_is_the_only_local_learning_update(self):
        assignment = re.compile(r"g_qTable\[from_node\]\[via_node\]\s*=\s*new_q")
        for source in (SENDER, INTERMEDIATE):
            self.assertEqual(len(assignment.findall(source)), 1)
            body = function_body(source, "void handleQFeedback(")
            self.assertIn("fromNodeId != g_nodeId", body)
            self.assertIn("viaNodeId != transport_from", body)
            self.assertIn("isDirectMeshNeighbor(mesh, transport_from)", body)
            self.assertIn("time_in_queue = 0.0f", body)
            self.assertIn("old_q + g_eta * (target - old_q)", body)

    def test_destination_reports_zero_and_does_not_forward(self):
        body = function_body(RECEIVER, "void handlePacketHop(")
        self.assertIn("float t = 0.0f", body)
        self.assertIn("sendQFeedback(previousNode, g_nodeId, t, step_time)", body)
        self.assertIn("emitDeliveryData(doc)", body)
        self.assertNotIn("prepareAndSendMessage", body)
        self.assertNotIn("chooseBestAction", body)

    def test_message_and_delivery_formats_remain_present(self):
        all_sources = SENDER + INTERMEDIATE + RECEIVER
        for marker in ("PACKET_HOP", "Q_FEEDBACK", "DELIVERY_DATA:"):
            self.assertIn(marker, all_sources)

    def test_direct_neighbor_helper_uses_only_first_level_subs(self):
        body = function_body(COMMON, "inline std::vector<NodeId> getDirectMeshNeighbors(")
        self.assertIn("mesh.asNodeTree()", body)
        self.assertIn("topology.subs", body)
        self.assertNotIn("getNodeList", body)
        for source in (SENDER, INTERMEDIATE):
            get_neighbors = function_body(source, "std::vector<NodeId> getNeighbors(")
            self.assertIn("getDirectMeshNeighbors(mesh)", get_neighbors)
            self.assertNotIn("getNodeList", get_neighbors)

    def test_firmware_does_not_use_signed_neighbor_ids(self):
        for source in (SENDER, INTERMEDIATE, RECEIVER):
            self.assertNotIn("std::vector<int> getNeighbors", source)
            self.assertNotIn(".toInt()", source)
        self.assertIn("using NodeId = uint32_t", COMMON)
        self.assertIn("UINT32_MAX", COMMON)
        self.assertNotIn("strtoul", COMMON)

    def test_sender_does_not_persist_packet_history(self):
        self.assertNotIn("g_packets", SENDER)
        body = function_body(SENDER, "void buildMessage(")
        self.assertIn('doc.createNestedArray("packets")', body)
        self.assertIn('doc["hops_remaining"] = MAX_Q_HOPS', body)

    def test_intermediate_checks_forwarding_result(self):
        body = function_body(INTERMEDIATE, "void processPacket(")
        self.assertIn("if (prepareAndSendMessage(doc, next_action))", body)
        self.assertIn("g_diagnostics.packets_dropped++", body)

    def test_retry_loop_does_not_block_mesh_updates_with_delay(self):
        for source in (SENDER, INTERMEDIATE, RECEIVER):
            body = function_body(source, "bool sendMessageWithRetries(")
            self.assertNotIn("delay(", body)
            self.assertIn("retry_exhausted", body)
            self.assertIn("isDirectMeshNeighbor(mesh, next_hop)", body)

    def test_routing_messages_require_direct_transport(self):
        for source in (SENDER, INTERMEDIATE):
            forward = function_body(source, "bool prepareAndSendMessage(")
            self.assertIn("updatedJsonString, true", forward)
        for source in (SENDER, INTERMEDIATE, RECEIVER):
            feedback = function_body(source, "void sendQFeedback(")
            self.assertIn("feedbackStr, true", feedback)

    def test_packet_validation_and_size_limit_are_shared(self):
        self.assertIn("validatePacketDocument", COMMON)
        self.assertIn("MAX_PACKET_JSON_SIZE", COMMON)
        self.assertIn("steps.size() <= MAX_Q_HOPS", COMMON)

    def test_diagnostic_request_and_response_contract(self):
        all_sources = SENDER + INTERMEDIATE + RECEIVER
        self.assertIn('"DIAG_REQUEST"', all_sources)
        self.assertIn('"DIAG_RESPONSE"', all_sources)
        self.assertIn('"DIAG_DATA:"', RECEIVER)
        self.assertIn("PendingDiagnosticResponse", COMMON)
        self.assertIn("DiagnosticRequestHistory", COMMON)
        for field in (
            '"direct_neighbors"',
            '"known_nodes"',
            '"q_row"',
            '"drop_ttl"',
            '"send_enqueue_failures"',
            '"feedback_applied"',
        ):
            self.assertIn(field, COMMON)

    def test_diagnostics_are_on_demand_and_staggered(self):
        for source in (SENDER, INTERMEDIATE, RECEIVER):
            schedule = function_body(source, "void scheduleDiagnosticResponse(")
            self.assertIn("g_nodeId % 750", schedule)
            process = function_body(source, "void processPendingDiagnosticResponse(")
            self.assertIn("g_pendingDiagnostic.pending", process)
        self.assertIn("Serial.read() == 'D'", RECEIVER)


if __name__ == "__main__":
    unittest.main()
