import pyarrow as pa

with pa.memory_map("image.arrow", "r") as source:
    df_i = pa.ipc.open_file(source).read_all()

with pa.memory_map("bbox.arrow", "r") as source:
    df_b = pa.ipc.open_file(source).read_all()

df_i = df_i.to_pandas()
df_b = df_b.to_pandas()

df = df_i.merge(df_b, on="trace_id")
