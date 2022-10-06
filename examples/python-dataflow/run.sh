python3 -m venv .env
. $(pwd)/.env/bin/activate
# Dev dependencies

# Dependencies
pip install --upgrade pip
pip install -r requirements.txt

cargo run -p dora-coordinator --release -- run dataflow.yml