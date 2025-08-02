import argparse

from dora import Node
import pyarrow as pa

class ActInference:
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ckpt_dir", action="store", type=str, help="ckpt_dir", required=True)
        self.parser.add_argument(
            "--policy_class", action="store", type=str, help="policy_class, capitalize", required=True
        )
        self.parser.add_argument("--task_name", action="store", type=str, help="task_name", required=True)
        self.parser.add_argument("--batch_size", action="store", type=int, help="batch_size", required=True)
        self.parser.add_argument("--seed", action="store", type=int, help="seed", required=True)
        self.parser.add_argument("--num_epochs", action="store", type=int, help="num_epochs", required=True)
        self.parser.add_argument("--lr", action="store", type=float, help="lr", required=True)

        self.parser.add_argument("--kl_weight", action="store", type=int, help="KL Weight", required=False)
        self.parser.add_argument("--chunk_size", action="store", type=int, help="chunk_size", required=False)
        self.parser.add_argument("--hidden_dim", action="store", type=int, help="hidden_dim", required=False)
        self.parser.add_argument(
            "--dim_feedforward", action="store", type=int, help="dim_feedforward", required=False
        )
        self.parser.add_argument("--temporal_agg", action="store_true")

        self.node = Node()

        self.image = None
        self.qpos = None

    def get_image(self, request=True):
        got_image = False
        while not got_image:
            if request:
                self.node.send_output(output_id="request_image", data=pa.array([]), metadata={})
            event = self.node.next(1)
            if event is not None and event["type"] == "INPUT" and event["id"] == "image":
                self.image = event["value"].to_numpy()
                got_image = True

    def get_qpos(self, request=True):
        got_qpos = False
        while not got_qpos:
            if request:
                self.node.send_output(output_id="request_qpos", data=pa.array([]), metadata={})
            event = self.node.next(1)
            if event is not None and event["type"] == "INPUT" and event["id"] == "qpos":
                self.qpos = event["value"].to_pylist()
                got_qpos = True
    
    def pub_action(self, action):
        self.node.send_output(output_id="action", data=pa.array(action), metadata={})
    
    def make_policy(self, policy_calss, policy_config):
        pass

    def make_optimizer(self, policy_class, policy):
        pass

    def inference(self):
        pass
