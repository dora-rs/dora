#!/usr/bin/env python3
"""
Simple log receiver to debug node outputs
"""

import sys
from dora import Node, DoraStatus

def main():
    node = Node("log-display")
    
    print("[LOG RECEIVER] Started, waiting for logs...", flush=True)
    sys.stdout.flush()
    
    event_count = 0
    for event in node:
        event_count += 1
        
        try:
            if event["type"] == "INPUT":
                input_id = event.get("id", "unknown")
                value = event.get("value", None)
                
                if value is not None:
                    # Try to extract the actual value
                    try:
                        if hasattr(value, 'to_numpy'):
                            data = value.to_numpy()
                            print(f"[LOG RECEIVER] Event #{event_count}: {input_id} - numpy array with {len(data)} elements", flush=True)
                        elif hasattr(value, '__getitem__'):
                            data = value[0].as_py() if hasattr(value[0], 'as_py') else str(value[0])
                            print(f"[LOG RECEIVER] Event #{event_count}: {input_id} - {data}", flush=True)
                        else:
                            print(f"[LOG RECEIVER] Event #{event_count}: {input_id} - {value}", flush=True)
                    except Exception as e:
                        print(f"[LOG RECEIVER] Event #{event_count}: {input_id} - Error extracting value: {e}", flush=True)
                else:
                    print(f"[LOG RECEIVER] Event #{event_count}: {input_id} - No value", flush=True)
            else:
                print(f"[LOG RECEIVER] Event #{event_count}: {event.get('type', 'unknown')} event", flush=True)
                
        except Exception as e:
            print(f"[LOG RECEIVER] Error processing event #{event_count}: {e}", flush=True)
        
        sys.stdout.flush()
    
    return DoraStatus.STOP

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[LOG RECEIVER] Fatal error: {e}", flush=True)
        sys.exit(1)