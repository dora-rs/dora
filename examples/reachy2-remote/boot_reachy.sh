ssh bedrock@reachy2-pvt02.local


docker exec -it core bash


ros2 service call /SetZuuuSafety zuuu_interfaces/srv/SetZuuuSafety "{safety_on: False}"
