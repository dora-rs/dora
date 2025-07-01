from dora import Node

node = Node()

index_dict = {}
i = 0

LEAD_TOPIC = "vggt_depth"

for event in node:
    if event["type"] == "INPUT":
        if LEAD_TOPIC in event["id"]:
            storage = event["value"]
            metadata = event["metadata"]
            encoding = metadata["encoding"]
            width = metadata["width"]
            height = metadata["height"]

            # Save to file
            filename = f"out/{event['id']}_{i}.{encoding}"
            with open(filename, "wb") as f:
                f.write(storage.to_numpy())
            for key, value in index_dict.items():
                filename = f"out/{key}_{i}.{value['metadata']['encoding']}"
                with open(filename, "wb") as f:
                    f.write(value["value"])
            i += 1
        else:
            # Store the event in the index dictionary
            index_dict[event["id"]] = {
                "type": event["type"],
                "value": event["value"].to_numpy(),
                "metadata": event["metadata"],
            }
