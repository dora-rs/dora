# Docker vehicule node
SRC_LABELS=carla/lane_1/plot_1 python image_source.py
SRC_LABELS=carla/lane_1/plot_1 python image_source.py

# Copy
NAME=lane_1 python copy_operator.py

# Plot node
NAME=plot_1 python plot_sink.py
NAME=plot_2 python plot_sink.py

