import pyarrow as pa
import torch
from dora import Node
from dora.cuda import torch_to_ipc_buffer

torch.tensor([], device="cuda")

node = Node()

for i in range(10):
    tensor = torch.arange(i * 100, (i + 1) * 100, dtype=torch.int64, device="cuda")
    ipc_buffer, metadata = torch_to_ipc_buffer(tensor)
    node.send_output("cuda_data", ipc_buffer, metadata)

    event = node.next()
    if event["type"] != "INPUT":
        break

print("sender done")
