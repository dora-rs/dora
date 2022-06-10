@0xaf56e553af765dce;

struct Metadata {
  metadataVersion @0 :UInt16;
  watermark @1 :UInt64; 
  deadline @2 :UInt64;
  otelContext @3 :Text; # OpenTelemetry Context allowing shared context between nodes.
}


struct Message {
  id @0 :UInt32;
  metadata @1 :Metadata;
  data @2 :Data;
}
