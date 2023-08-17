set -e

python3 -m venv .env
. $(pwd)/.env/bin/activate

# Dependencies
pip install --upgrade pip
pip install -r requirements.txt
maturin develop -m ../../apis/python/node/Cargo.toml

cargo run -p dora-daemon -- --run-dataflow dataflow.yml
