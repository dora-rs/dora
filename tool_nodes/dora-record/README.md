# dora-record

dora data recording using Apache Arrow Parquet.

This nodes is still experimental.

## Getting Started

```bash
cargo install dora-record --locked
```

## Adding to existing graph:

```yaml
- id: dora-record
  custom:
    source: dora-record
    inputs:
      image: webcam/image
      text: webcam/text
      # You can add any input and it is going to be logged.
```

## Output Files

Format: Parquet file

path: `out/<DATAFLOW_ID>/<INPUT>.parquet`

Columns:

- trace_id: String, representing the id of the current trace
- span_id: String, representing the unique span id
- timestamp_uhlc: u64, representing the timestamp in [Unique Hybrid Logical Clock time](https://github.com/atolab/uhlc-rs)
- timestamp_utc: DataType::Timestamp(Milliseconds), representing the timestamp in Coordinated Universal Time.
- `<INPUT>` : Column containing the input in its defined format.

Example:

```json
{
  "trace_id": "2fd23ddf1b5d2aa38ddb86ceedb55928",
  "span_id": "15aef03e0f052bbf",
  "timestamp_uhlc": "7368873278370007008",
  "timestamp_utc": 1715699508406,
  "random": [1886295351360621740]
}
```

## merging multiple file

We can merge input files using the `trace_id` that is going to be shared when using opentelemetry features.

- `trace_id` can also be queried from UI such as jaeger UI, influxDB and so on...
- `trace_id` keep tracks of the logical flow of data, compared to timestamp based merging that might not reflect the actual logical flow of data.
