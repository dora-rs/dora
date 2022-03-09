# Enabling autonomous driving through Zenoh.

Project largely inspired by [Pylot](https://github.com/erdos-project/pylot).

Thank you the Pylot team for your contribution to open autonomous driving.

## Getting Started

To make this project work, you will need to:
- clone and install [pylot](https://github.com/erdos-project/pylot) in the parent directory using the install script.
- Install the requirements with:

```bash
pip install -r requirements.txt
```

- Install Zenoh-Python from the current Git repository as the pip version might not be up to date:
```bash
git clone git@github.com:eclipse-zenoh/zenoh-python.git
cd zenoh-python
python setup.py develop
``` 

> It's possible that cv2 might not have all its dependencies. You should try installing with conda with:

```bash
conda install -c menpo opencv
```
- Get an image

```bash
cd ..
cd data
wget https://www.smartrippers.com/files/2017-11/panneau-feu-usa2.jpg
cd ../traffic_lights
```

- Run the following command:
```bash
export SRC_LABELS=source
export OP_LABELS=traffic_light
python source.py &
python operator.py &
python sink.py
```