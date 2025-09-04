# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import numpy as np
import torch
from collections.abc import Mapping, Sequence
from huggingface_hub import hf_hub_download


DATAS = ["sample1", "sample1_high_res", "sample1_dino"]


def load(
    name: str = "sonata",
    download_root: str = None,
):
    if name in DATAS:
        print(f"Loading data from HuggingFace: {name} ...")
        data_path = hf_hub_download(
            repo_id="pointcept/demo",
            filename=f"{name}.npz",
            repo_type="dataset",
            revision="main",
            local_dir=download_root or os.path.expanduser("~/.cache/sonata/data"),
        )
    elif os.path.isfile(name):
        print(f"Loading data in local path: {name} ...")
        data_path = name
    else:
        raise RuntimeError(f"Data {name} not found; available models = {DATAS}")
    return dict(np.load(data_path))


from torch.utils.data.dataloader import default_collate


def collate_fn(batch):
    """
    collate function for point cloud which support dict and list,
    'coord' is necessary to determine 'offset'
    """
    if not isinstance(batch, Sequence):
        raise TypeError(f"{batch.dtype} is not supported.")

    if isinstance(batch[0], torch.Tensor):
        return torch.cat(list(batch))
    elif isinstance(batch[0], str):
        # str is also a kind of Sequence, judgement should before Sequence
        return list(batch)
    elif isinstance(batch[0], Sequence):
        for data in batch:
            data.append(torch.tensor([data[0].shape[0]]))
        batch = [collate_fn(samples) for samples in zip(*batch)]
        batch[-1] = torch.cumsum(batch[-1], dim=0).int()
        return batch
    elif isinstance(batch[0], Mapping):
        batch = {
            key: (
                collate_fn([d[key] for d in batch])
                if "offset" not in key
                # offset -> bincount -> concat bincount-> concat offset
                else torch.cumsum(
                    collate_fn([d[key].diff(prepend=torch.tensor([0])) for d in batch]),
                    dim=0,
                )
            )
            for key in batch[0]
        }
        return batch
    else:
        return default_collate(batch)
