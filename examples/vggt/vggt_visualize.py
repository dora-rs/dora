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
"""' Imports Sonata inference results and visualizes sementic segmentation"""

import open3d as o3d
import torch

with torch.no_grad():
    torch.cuda.empty_cache()

point = torch.load("predictions.pt", weights_only=False)
color = point["color"].cpu().detach().numpy()
print(point.keys())

# Visualize
pcd = o3d.geometry.PointCloud()
points = torch.tensor(point["coord"]).cpu().detach().numpy()
# Inverse y axis
points[:, 1] = -points[:, 1]
pcd.points = o3d.utility.Vector3dVector(points)

pcd.colors = o3d.utility.Vector3dVector(color)
o3d.visualization.draw_geometries([pcd])
