# Enabling autonomous driving through Dora.

Project largely inspired by [Pylot](https://github.com/erdos-project/pylot).

Thank you the Pylot team for your contribution to open autonomous driving.

## Getting Started

To make this project work, you will need to:
- clone and install [pylot](https://github.com/erdos-project/pylot) in the parent directory using the install script.
- Make sure to have the environment variable `PYLOT_HOME` set to the root of the cloned Pylot repo.
- Create a conda environment with Python 3.8:
```bash
conda create -n dora3.8 python=3.8   
```
- Install the requirements for this demo with:

```bash
pip install -r requirements.txt
```
- Add this directory to your PYTHONPATH
```bash
export PYTHONPATH=$PYTHONPATH:$(pwd)
```
- Add your pythonlib:
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/miniconda3/envs/dora3.8/lib
```

- Run on different terminal:
```bash
cargo run --release start-python source produce                                             
```
```bash
cargo run --release start-python efficient_detection run image                                             
```
```bash
cargo run --release start-python sink_eval_plot destination                                 
```
