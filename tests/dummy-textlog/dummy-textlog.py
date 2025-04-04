import pyarrow as pa
from dora import Node

def main():
    node = Node()
    print("[dummy-textlog] Node started", flush=True)

    for event in node:
        try:
            print(f"\n[dummy-textlog] Received event with keys: {event.keys()}", flush=True)

            if event.get("type") != "INPUT":
                continue

            if "id" not in event or ("payload" not in event and "value" not in event):
                msg = "[dummy-textlog] Skipped event: missing 'id' or 'payload/value'"
                print(msg, flush=True)
                node.send_output("textlog", pa.array([msg]))
                continue

            event_id = event["id"]
            payload = event.get("payload") or event.get("value")

            # 👇 Log what we’re getting
            print(f"[dummy-textlog] Event ID: {event_id}", flush=True)
            print(f"[dummy-textlog] Payload type: {type(payload)}", flush=True)

            # 👇 NEW: handle StructArray payloads directly
            if isinstance(payload, pa.StructArray):
                try:
                    table = pa.Table.from_arrays([payload], names=["bbox"])
                    detections = table.to_pylist()

                    if detections:
                        for det in detections:
                            msg = f"[dummy-textlog] YOLO Detection: {det}"
                            print(msg, flush=True)
                            node.send_output("textlog", pa.array([msg]))
                    else:
                        msg = "[dummy-textlog] YOLO Detection: No objects detected"
                        print(msg, flush=True)
                        node.send_output("textlog", pa.array([msg]))

                except Exception as arrow_err:
                    print("[dummy-textlog] Failed to parse StructArray", flush=True)
                    print(f"Arrow error: {arrow_err}", flush=True)
                    node.send_output("textlog", pa.array(["[dummy-textlog] Failed to parse StructArray: invalid data"]))
                continue

            # 👇 OLD METHOD: still keep if bbox comes as Arrow stream (future-proofing)
            if isinstance(payload, (bytes, bytearray)):
                try:
                    reader = pa.ipc.open_stream(payload)
                    table = reader.read_all()
                    detections = table.to_pylist()

                    if detections:
                        for det in detections:
                            msg = f"[dummy-textlog] YOLO Detection: {det}"
                            print(msg, flush=True)
                            node.send_output("textlog", pa.array([msg]))
                    else:
                        msg = "[dummy-textlog] YOLO Detection: No objects detected"
                        print(msg, flush=True)
                        node.send_output("textlog", pa.array([msg]))

                except Exception as arrow_err:
                    print("[dummy-textlog] Failed to parse Arrow stream", flush=True)
                    print(f"Arrow error: {arrow_err}", flush=True)
                    node.send_output("textlog", pa.array(["[dummy-textlog] Failed to parse Arrow stream: invalid stream"]))
                continue

            # 👇 Skip anything else
            msg = f"[dummy-textlog] Ignored event with id: {event_id}"
            print(msg, flush=True)

        except Exception as e:
            err_msg = f"[dummy-textlog] Unexpected error: {e}"
            print(err_msg, flush=True)
            node.send_output("textlog", pa.array([err_msg]))

if __name__ == "__main__":
    main()
