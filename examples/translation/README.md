# Dora argo example

Make sure to have, dora, pip and cargo installed.

```bash
dora up

## For chinese
dora build dataflow_zh_en_terminal.yml
dora start dataflow_zh_en_terminal.yml --detach

python pretty_print.py

dora stop


## For chinese
dora build dataflow_en_zh_terminal.yml
dora start dataflow_en_zh_terminal.yml --detach

python pretty_print.py

dora stop
```
