

# Docker vehicule node
docker container run --env SRC_LABELS=carla/lane_1/plot_1 --env CARLA_SIMULATOR_HOST=192.168.1.15 --env CARLA_SIMULATOR_PORT=2000 --env ZENOH_HOST=tcp/192.168.157:7447 -d carla_source
docker container run --env SRC_LABELS=carla/lane_1/plot_2 --env CARLA_SIMULATOR_HOST=192.168.1.15 --env CARLA_SIMULATOR_PORT=2000 --env ZENOH_HOST=tcp/192.168.157:7447 -d carla_source

# LaneNet Node
LANE_NUMBER=lane_1 python lane_operator.py

# Copy
NAME=lane_1 python copy_operator.py

# Plot node
SINK=plot_1 python plot_sink.py
SINK=plot_2 python plot_sink.py
