import tempfile
import unittest
from pathlib import Path

import server


class ResetEndpointTests(unittest.TestCase):
    def setUp(self):
        self.original_log_file = server.LOG_FILE
        self.temp_dir = tempfile.TemporaryDirectory()
        server.LOG_FILE = Path(self.temp_dir.name) / 'received_data.log'
        server.state = server.SessionState()
        server.sse_listeners.clear()
        self.client = server.app.test_client()

    def tearDown(self):
        server.LOG_FILE = self.original_log_file
        self.temp_dir.cleanup()

    def test_log_path_is_anchored_to_server_directory(self):
        self.assertEqual(
            self.original_log_file.parent,
            Path(server.__file__).resolve().parent,
        )

    def test_local_reset_clears_file_and_memory(self):
        event = {
            'current_packet_id': 1,
            'current_node_id': '2',
            'q_table': {'1': {'2': 0.5}},
            'packets': [{'packet_id': 1, 'steps': []}],
        }
        self.assertEqual(self.client.post('/data', json=event).status_code, 200)
        self.assertEqual(server.state.to_api_state()['total_packets'], 1)
        self.assertGreater(server.LOG_FILE.stat().st_size, 0)

        response = self.client.post('/api/reset')

        self.assertEqual(response.status_code, 200)
        self.assertEqual(server.state.to_api_state()['total_packets'], 0)
        self.assertEqual(server.LOG_FILE.read_text(), '')

    def test_remote_reset_is_rejected(self):
        response = self.client.post(
            '/api/reset',
            environ_base={'REMOTE_ADDR': '192.0.2.10'},
        )
        self.assertEqual(response.status_code, 403)


if __name__ == '__main__':
    unittest.main()
