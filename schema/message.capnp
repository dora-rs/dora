@0xaf56e553af765dce;

struct Metadata {
  metadataVersion @0 :Text;
  watermark @1 :UInt64; 
  deadline @2 :UInt64;
  otelContext @3 :Text;
  tracingId @4 :Text;
}


struct Message {
  id @0 :UInt32;
  data @1 :Data;
  metadata @2 :Metadata;
}
