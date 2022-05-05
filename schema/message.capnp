@0xaf56e553af765dce;

struct Metadata {
  metadataVersion @0 :Text;
  watermark @1 :UInt64; 
  deadline @2 :UInt64;
  opentelemetryContext @3 :Text;
  tracingId @4 :Text;
}


struct Message {
  data @0 :Data;
  id @1 :UInt32;
  metadata @2 :Metadata;
}
