import unittest
import yaml

from doraflow.dataflow import Dataflow, Node, Operator

class TestDataflow(unittest.TestCase):

    def test_add_node(self):
        with Dataflow() as df:
            node = df.add_node("test-node")
            self.assertEqual(len(df.nodes), 1)
            self.assertEqual(df.nodes[0].id, "test-node")

    def test_to_yaml(self):
        with Dataflow() as df:
            df.add_node("test-node", path="/path/to/node")
            yaml_str = df.to_yaml()
            data = yaml.safe_load(yaml_str)
            self.assertEqual(len(data["nodes"]), 1)
            self.assertEqual(data["nodes"][0]["id"], "test-node")
            self.assertEqual(data["nodes"][0]["path"], "/path/to/node")

    def test_add_edge(self):
        with Dataflow() as df:
            node1 = df.add_node("node1")
            node2 = df.add_node("node2")
            df.add_edge(node1, "output1", node2, "input1")
            self.assertIn("outputs", node1.config)
            self.assertIn("output1", node1.config["outputs"])
            self.assertIn("inputs", node2.config)
            self.assertEqual(node2.config["inputs"]["input1"], "node1/output1")

    def test_send_to(self):
        with Dataflow() as df:
            node1 = df.add_node("node1")
            node2 = df.add_node("node2")
            node1.send("output1").to(node2, "input1")
            self.assertIn("outputs", node1.config)
            self.assertIn("output1", node1.config["outputs"])
            self.assertIn("inputs", node2.config)
            self.assertEqual(node2.config["inputs"]["input1"], "node1/output1")

    def test_node_config(self):
        with Dataflow() as df:
            node = df.add_node("test-node")
            node.path("/path/to/node")
            node.args("-v")
            node.env({"KEY": "VALUE"})
            node.build("make")
            node.git("https://github.com/dora-rs/dora.git")

            config = node.to_dict()
            self.assertEqual(config["path"], "/path/to/node")
            self.assertEqual(config["args"], "-v")
            self.assertEqual(config["env"]["KEY"], "VALUE")
            self.assertEqual(config["build"], "make")
            self.assertEqual(config["git"], "https://github.com/dora-rs/dora.git")

    def test_operator(self):
        op = Operator("op1", python="op.py")
        config = op.to_dict()
        self.assertEqual(config["id"], "op1")
        self.assertEqual(config["python"], "op.py")

    def test_add_operator(self):
        with Dataflow() as df:
            node = df.add_node("runtime-node")
            op = Operator("op1", python="op.py")
            node.add_operator(op)
            config = node.to_dict()
            self.assertIn("operators", config)
            self.assertEqual(len(config["operators"]), 1)
            self.assertEqual(config["operators"][0]["id"], "op1")

if __name__ == '__main__':
    unittest.main()
