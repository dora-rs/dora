import numpy as np
import pyarrow as pa


# Marker Message Example
TEST_ARRAYS = [
    ("std_msgs", "UInt8", pa.array([{"data": np.uint8(2)}])),
    (
        "std_msgs",
        "String",
        pa.array([{"data": "hello"}]),
    ),
    (
        "std_msgs",
        "UInt8MultiArray",
        pa.array(
            [
                {
                    "data": np.array([1, 2, 3, 4], np.uint8),
                    "layout": {
                        "dim": [
                            {
                                "label": "a",
                                "size": np.uint32(10),
                                "stride": np.uint32(20),
                            }
                        ],
                        "data_offset": np.uint32(30),
                    },
                }
            ]
        ),
    ),
    (
        "std_msgs",
        "Float32MultiArray",
        pa.array(
            [
                {
                    "data": np.array([1, 2, 3, 4], np.float32),
                    "layout": {
                        "dim": [
                            {
                                "label": "a",
                                "size": np.uint32(10),
                                "stride": np.uint32(20),
                            }
                        ],
                        "data_offset": np.uint32(30),
                    },
                }
            ]
        ),
    ),
    (
        "visualization_msgs",
        "Marker",
        pa.array(
            [
                {
                    "header": {
                        "frame_id": "world",  # Placeholder value (String type, no numpy equivalent)
                    },
                    "ns": "my_namespace",  # Placeholder value (String type, no numpy equivalent)
                    "id": np.int32(1),  # Numpy type
                    "type": np.int32(0),  # Numpy type (ARROW)
                    "action": np.int32(0),  # Numpy type (ADD)
                    "lifetime": {
                        "sec": np.int32(1),
                        "nanosec": np.uint32(2),
                    },  # Numpy type
                    "pose": {
                        "position": {
                            "x": np.float64(1.0),  # Numpy type
                            "y": np.float64(2.0),  # Numpy type
                            "z": np.float64(3.0),  # Numpy type
                        },
                        "orientation": {
                            "x": np.float64(0.0),  # Numpy type
                            "y": np.float64(0.0),  # Numpy type
                            "z": np.float64(0.0),  # Numpy type
                            "w": np.float64(1.0),  # Numpy type
                        },
                    },
                    "scale": {
                        "x": np.float64(1.0),  # Numpy type
                        "y": np.float64(1.0),  # Numpy type
                        "z": np.float64(1.0),  # Numpy type
                    },
                    "color": {
                        "r": np.float32(1.0),  # Numpy type
                        "g": np.float32(0.0),  # Numpy type
                        "b": np.float32(0.0),  # Numpy type
                        "a": np.float32(1.0),  # Numpy type (alpha)
                    },
                    "frame_locked": False,  # Boolean type, no numpy equivalent
                    "points": [  # Numpy array for points
                        {
                            "x": np.float64(1.0),  # Numpy type
                            "y": np.float64(1.0),  # Numpy type
                            "z": np.float64(1.0),  # Numpy type
                        }
                    ],
                    "colors": [
                        {
                            "r": np.float32(1.0),  # Numpy type
                            "g": np.float32(1.0),  # Numpy type
                            "b": np.float32(1.0),  # Numpy type
                            "a": np.float32(1.0),  # Numpy type (alpha)
                        }  # Numpy array for colors
                    ],
                    "texture_resource": "",
                    "uv_coordinates": [{}],
                    "text": "",
                    "mesh_resource": "",
                    "mesh_use_embedded_materials": False,  # Boolean type, no numpy equivalent
                }
            ]
        ),
    ),
    (
        "visualization_msgs",
        "MarkerArray",
        pa.array(
            [
                {
                    "markers": [
                        {
                            "header": {
                                "frame_id": "world",  # Placeholder value (String type, no numpy equivalent)
                            },
                            "ns": "my_namespace",  # Placeholder value (String type, no numpy equivalent)
                            "id": np.int32(1),  # Numpy type
                            "type": np.int32(0),  # Numpy type (ARROW)
                            "action": np.int32(0),  # Numpy type (ADD)
                            "lifetime": {
                                "sec": np.int32(1),
                                "nanosec": np.uint32(2),
                            },  # Numpy type
                            "pose": {
                                "position": {
                                    "x": np.float64(1.0),  # Numpy type
                                    "y": np.float64(2.0),  # Numpy type
                                    "z": np.float64(3.0),  # Numpy type
                                },
                                "orientation": {
                                    "x": np.float64(0.0),  # Numpy type
                                    "y": np.float64(0.0),  # Numpy type
                                    "z": np.float64(0.0),  # Numpy type
                                    "w": np.float64(1.0),  # Numpy type
                                },
                            },
                            "scale": {
                                "x": np.float64(1.0),  # Numpy type
                                "y": np.float64(1.0),  # Numpy type
                                "z": np.float64(1.0),  # Numpy type
                            },
                            "color": {
                                "r": np.float32(1.0),  # Numpy type
                                "g": np.float32(0.0),  # Numpy type
                                "b": np.float32(0.0),  # Numpy type
                                "a": np.float32(1.0),  # Numpy type (alpha)
                            },
                            "frame_locked": False,  # Boolean type, no numpy equivalent
                            "points": [  # Numpy array for points
                                {
                                    "x": np.float64(1.0),  # Numpy type
                                    "y": np.float64(1.0),  # Numpy type
                                    "z": np.float64(1.0),  # Numpy type
                                }
                            ],
                            "colors": [
                                {
                                    "r": np.float32(1.0),  # Numpy type
                                    "g": np.float32(1.0),  # Numpy type
                                    "b": np.float32(1.0),  # Numpy type
                                    "a": np.float32(1.0),  # Numpy type (alpha)
                                }  # Numpy array for colors
                            ],
                            "texture_resource": "",
                            "uv_coordinates": [{}],
                            "text": "",
                            "mesh_resource": "",
                            "mesh_use_embedded_materials": False,  # Boolean type, no numpy equivalent
                        }
                    ]
                }
            ]
        ),
    ),
    (
        "visualization_msgs",
        "ImageMarker",
        pa.array(
            [
                {
                    "header": {
                        "stamp": {
                            "sec": np.int32(123456),  # 32-bit integer
                            "nanosec": np.uint32(789),  # 32-bit unsigned integer
                        },
                        "frame_id": "frame_example",
                    },
                    "ns": "namespace",
                    "id": np.int32(1),  # 32-bit integer
                    "type": np.int32(0),  # 32-bit integer, e.g., CIRCLE = 0
                    "action": np.int32(0),  # 32-bit integer, e.g., ADD = 0
                    "position": {
                        "x": np.float64(1.0),  # 32-bit float
                        "y": np.float64(2.0),  # 32-bit float
                        "z": np.float64(3.0),  # 32-bit float
                    },
                    "scale": np.float32(1.0),  # 32-bit float
                    "outline_color": {
                        "r": np.float32(255.0),  # 32-bit float
                        "g": np.float32(0.0),  # 32-bit float
                        "b": np.float32(0.0),  # 32-bit float
                        "a": np.float32(1.0),  # 32-bit float
                    },
                    "filled": np.uint8(1),  # 8-bit unsigned integer
                    "fill_color": {
                        "r": np.float32(0.0),  # 32-bit float
                        "g": np.float32(255.0),  # 32-bit float
                        "b": np.float32(0.0),  # 32-bit float
                        "a": np.float32(1.0),  # 32-bit float
                    },
                    "lifetime": {
                        "sec": np.int32(300),  # 32-bit integer
                        "nanosec": np.uint32(0),  # 32-bit unsigned integer
                    },
                    "points": [
                        {
                            "x": np.float64(1.0),  # 32-bit float
                            "y": np.float64(2.0),  # 32-bit float
                            "z": np.float64(3.0),  # 32-bit float
                        },
                        {
                            "x": np.float64(4.0),  # 32-bit float
                            "y": np.float64(5.0),  # 32-bit float
                            "z": np.float64(6.0),  # 32-bit float
                        },
                    ],
                    "outline_colors": [
                        {
                            "r": np.float32(255.0),  # 32-bit float
                            "g": np.float32(0.0),  # 32-bit float
                            "b": np.float32(0.0),  # 32-bit float
                            "a": np.float32(1.0),  # 32-bit float
                        },
                        {
                            "r": np.float32(0.0),  # 32-bit float
                            "g": np.float32(255.0),  # 32-bit float
                            "b": np.float32(0.0),  # 32-bit float
                            "a": np.float32(1.0),  # 32-bit float
                        },
                    ],
                }
            ]
        ),
    ),
]


def is_subset(subset, superset):
    """
    Check if subset is a subset of superset, to avoid false negatives linked to default values.
    """
    if isinstance(subset, pa.Array):
        return is_subset(subset.to_pylist(), superset.to_pylist())

    match subset:
        case dict(_):
            return all(
                key in superset and is_subset(val, superset[key])
                for key, val in subset.items()
            )
        case list(_) | set(_):
            return all(
                any(is_subset(subitem, superitem) for superitem in superset)
                for subitem in subset
            )
        # assume that subset is a plain value if none of the above match
        case _:
            return subset == superset
