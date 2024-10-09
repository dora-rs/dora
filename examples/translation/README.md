# Dora argo example

Make sure to have, dora, pip and cargo installed.

```bash
# Install rerun if it's not done already
dora up

dora build dataflow_cn_terminal.yml
dora start dataflow_cn_terminal.yml

# In another terminal
python pretty_print.py
```
