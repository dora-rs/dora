import os

python_path = os.getenv("ISAAC_PYTHON_PATH", "python")

isaac_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "start.py")

config_name = os.getenv("CONFIG_NAME", "stack_cube_act")

os.system(f"{python_path} {isaac_path} --config-name {config_name}")
