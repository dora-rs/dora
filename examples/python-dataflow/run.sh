python3 -m venv .env
. $(pwd)/.env/bin/activate
# Dev dependencies
pip install maturin
cd ../../apis/python/node
maturin develop
cd ../../../examples/python-dataflow

# Dependencies
pip install --upgrade pip
pip install -r requirements.txt

cargo run -p dora-coordinator --release -- run dataflow_without_webcam.yml
