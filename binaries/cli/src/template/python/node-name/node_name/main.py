from dora import Node
import pyarrow as pa


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "TICK":
                print(
                    f"""Node received:
                id: {event["id"]},
                value: {event["value"]},
                metadata: {event["metadata"]}"""
                )

            else:
                # Warning: Make sure to add the output event_id within the dataflow.
                node.send_output(
                    output_id="event_id", data=pa.array([1, 2, 3]), metadata={}
                )


if __name__ == "__main__":
    main()
