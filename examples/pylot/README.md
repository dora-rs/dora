# Running CARLA

To run Carla, please check the installation guide of pylot: https://github.com/erdos-project/pylot
I advise using the docker container for the installation as there might be some issues with the current erdos python package.

To fix matplotlib backend cairo issue:
```bash
sudo apt-get install python3-gi-cairo
``` 
# Connection to the Carla Simulator

Make sure to use the right Host and Port.

# Dev
To develop you can try to ssh into the container