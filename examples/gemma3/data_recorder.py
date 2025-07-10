from datasets import Dataset, Features, Image, Value, Sequence
import pandas as pd
import numpy as np
from PIL import Image as PILImage

def coco_to_xyxy(coco_bbox):
    x, y, width, height = coco_bbox
    x1, y1 = x, y
    x2, y2 = x + width, y + height
    return [x1, y1, x2, y2]


def convert_to_detection_string(bboxs, image_width, image_height, name_label):
    def format_location(value, max_value):
        return f"<loc{int(round(value * 1024 / max_value)):04}>"

    detection_strings = []
    for i,bbox in enumerate(bboxs):
        x1, y1, x2, y2 = bbox
        name_label_list = name_label.replace('\n', '').strip()[2:-2].split("' '")
        name = name_label_list[i]
        locs = [
            format_location(y1, image_height),
            format_location(x1, image_width),
            format_location(y2, image_height),
            format_location(x2, image_width),
        ]
        detection_string = "".join(locs) + f" {name}"
        detection_strings.append(detection_string)

    return " ; ".join(detection_strings)


def format_objects(example): # add attribute bbox_location_label to dataset, which is of format "<loc0000><loc0000><loc0000><loc0000> bbox ; <loc0000><loc0000><loc0000><loc0000> bbox"
    height = example["height"]
    width = example["width"]
    bboxs = example["objects"]["bbox"]
    name_label = example["name_label"]
    formatted_objects = convert_to_detection_string(bboxs, width, height, name_label)
    # return {"label_for_paligemma": formatted_objects}
    return {"bbox_location_name_label": formatted_objects}




from dora import Node
import cv2
node = Node()
img_id = 0
bboxs = []

data_dict = {}

for event in node:
    if "image" in event["id"]: # save images through this type of event from camera
        storage = event["value"]
        metadata = event["metadata"]
        encoding = metadata["encoding"]
        width = metadata["width"]
        height = metadata["height"]

        if (
            encoding == "bgr8"
            or encoding == "rgb8"
            or encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]
        ):
            channels = 3
            storage_type = np.uint8
        else:
            raise RuntimeError(f"Unsupported image encoding: {encoding}")

        if encoding == "bgr8":
            frame = (
                storage.to_numpy()
                .astype(storage_type)
                .reshape((height, width, channels))
            )
            frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
        elif encoding == "rgb8":
            frame = (
                storage.to_numpy()
                .astype(storage_type)
                .reshape((height, width, channels))
            )
        elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
            storage = storage.to_numpy()
            frame = cv2.imdecode(storage, cv2.IMREAD_COLOR)
            frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
        else:
            raise RuntimeError(f"Unsupported image encoding: {encoding}")
        image = PILImage.fromarray(frame)
    elif event["id"] == "bbox": # save bboxes throught this type of event from parse_bbox
        # Assuming the event contains text data
        text = event["value"].to_numpy().reshape((-1,4))
        labels = event["metadata"].get("labels")
        # Process the text as needed
        bboxs.append({"bbox_value":text, "label_value": labels})
        image.save(f"out/{img_id}.jpeg")
        img_id += 1

if img_id != len(bboxs):
    raise ValueError(f"Number of images ({img_id - 1}) does not match number of bboxes ({len(bboxs)})")

# combine the data to form a dataset
data_dict = {
    "image_id": [img_id for img_id in range(img_id)],
    "image": [f"out/{img_id}.jpeg" for img_id in range(img_id)],
    "width": [image.width for image in [PILImage.open(f"out/{img_id}.jpeg") for img_id in range(len(bboxs))]],
    "height": [image.height for image in [PILImage.open(f"out/{img_id}.jpeg") for img_id in range(len(bboxs))] ],
    "objects": [
        {"id":{img_id}, "bbox": bbox["bbox_value"], "category": ["0"]*len(bbox["bbox_value"]) } for img_id, bbox in enumerate(bboxs)
    ],
    "name_label": [bbox["label_value"] for bbox in bboxs],
}

# print(data_dict)

# turn data_dict into a format of huggingface dataset, and add the last attribute bbox_location_name_label
dataset = Dataset.from_dict(data_dict)
dataset = dataset.map(format_objects)




TEXT= "xyz"


# format of mock data , 
features = Features({
    "image_id": Value("int64"),
    "image": Image(),
    "width": Value("int32"),
    "height": Value("int32"),
    "objects": Sequence({
        "id": Value("int64"),
        "bbox": Sequence(Value("float32"), length=4),
        "category": Value("string")
    }),
    "name_label": Value("string"),
    "bbox_location_name_label": Value("string")
})

dataset = dataset.cast(features)
# print("AAAAAAAAAAAAAAAAAA: ", dataset.features)


##########
# normalise the names
def label_replace(example, keyword, new_name):
    if keyword in example["name_label"]:
        example["bbox_location_name_label"] = example["bbox_location_name_label"][:37] + new_name
        example["name_label"] = new_name
    return example
dataset = dataset.map(label_replace, fn_kwargs={"keyword": "green", "new_name": "green leafy vegetables"})
dataset = dataset.map(label_replace, fn_kwargs={"keyword": "carrot", "new_name": "shredded carrots"})

def remove_brackets(example):
    if example["name_label"].startswith("['") and example["name_label"].endswith("']"):
        example["name_label"] = example["name_label"][2:-2]
    return example
dataset = dataset.map(remove_brackets)


##########
# combine with existing dataset 
from datasets import load_dataset, concatenate_datasets, DatasetDict
old_dataset = load_dataset("zhiyingzou0202/object_detection_bbox_paligemma", split="train+validation+test", features=features)
print("existing data:", len(old_dataset))

combined_dataset = concatenate_datasets([old_dataset, dataset])


##########
# do the train-test-validation split
train_test_split = combined_dataset.train_test_split(test_size=0.2)
training_splits = train_test_split["train"]
testing_splits = train_test_split["test"]
test_val_split = testing_splits.train_test_split(test_size=0.5)
validation_splits = test_val_split["train"]
testing_splits = test_val_split["test"]


##########
# push to hub
from datasets import DatasetDict
DatasetDict({"train": training_splits, "validation": validation_splits, "test": testing_splits}).push_to_hub("zhiyingzou0202/testtt")

