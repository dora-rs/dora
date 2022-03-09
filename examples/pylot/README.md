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

The command to setup ssh in the container is the following:
```bash
nvidia-docker cp ~/.ssh/id_rsa.pub pylot:/home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t pylot sudo chown erdos /home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t pylot sudo service ssh start
```

Then to launch

```bash
ssh -p 20022 -X erdos@localhost
cd /home/erdos/workspace/pylot/
``` 


- Install Zenoh-Python from the current Git repository as the pip version might not be up to date:
```bash
git clone git@github.com:eclipse-zenoh/zenoh-python.git
cd zenoh-python
python setup.py build
pip install .
``` 

## Docker CARLA Source 

```bash
cd carla_source_docker
docker build --tag carla_source .
docker container run --env SRC_LABELS=carla --env CARLA_SIMULATOR_HOST=192.168.1.15 --env CARLA_SIMULATOR_PORT=2000 --env ZENOH_HOST=tcp/192.168.157:7447 -d carla_source 
```