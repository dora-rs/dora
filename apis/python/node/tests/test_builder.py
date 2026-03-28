"""Tests for adora.builder (Pythonic dataflow definition API)."""

import yaml
import pytest

from adora.builder import DataflowBuilder, Node, Operator, Output


def test_empty_dataflow():
    builder = DataflowBuilder()
    output = yaml.safe_load(builder.to_yaml())
    assert output == {"nodes": []}


def test_single_node():
    builder = DataflowBuilder()
    builder.add_node("camera").path("camera.py")
    output = yaml.safe_load(builder.to_yaml())
    assert len(output["nodes"]) == 1
    assert output["nodes"][0]["id"] == "camera"
    assert output["nodes"][0]["path"] == "camera.py"


def test_node_with_output():
    builder = DataflowBuilder()
    cam = builder.add_node("camera").path("camera.py")
    img = cam.add_output("image")
    assert isinstance(img, Output)
    assert str(img) == "camera/image"
    output = yaml.safe_load(builder.to_yaml())
    assert "image" in output["nodes"][0]["outputs"]


def test_node_with_input_string_source():
    builder = DataflowBuilder()
    builder.add_node("camera").path("camera.py").add_output("image")
    builder.add_node("detector").path("detector.py").add_input("image", "camera/image")
    output = yaml.safe_load(builder.to_yaml())
    assert output["nodes"][1]["inputs"]["image"] == "camera/image"


def test_node_with_input_output_object():
    builder = DataflowBuilder()
    cam = builder.add_node("camera").path("camera.py")
    img = cam.add_output("image")
    builder.add_node("detector").path("detector.py").add_input("image", img)
    output = yaml.safe_load(builder.to_yaml())
    assert output["nodes"][1]["inputs"]["image"] == "camera/image"


def test_node_with_queue_options():
    builder = DataflowBuilder()
    builder.add_node("camera").path("camera.py").add_output("image")
    builder.add_node("detector").path("detector.py").add_input(
        "image", "camera/image", queue_size=10, queue_policy="drop_oldest"
    )
    output = yaml.safe_load(builder.to_yaml())
    inp = output["nodes"][1]["inputs"]["image"]
    assert inp["source"] == "camera/image"
    assert inp["queue_size"] == 10
    assert inp["queue_policy"] == "drop_oldest"


def test_invalid_queue_policy():
    node = Node("n")
    with pytest.raises(ValueError, match="queue_policy"):
        node.add_input("x", "a/b", queue_policy="invalid")


def test_invalid_queue_size():
    node = Node("n")
    with pytest.raises(ValueError, match="queue_size"):
        node.add_input("x", "a/b", queue_size=0)


def test_node_chaining():
    builder = DataflowBuilder()
    node = builder.add_node("n").path("n.py").args("--verbose").env({"KEY": "val"}).build("make")
    output = yaml.safe_load(builder.to_yaml())
    n = output["nodes"][0]
    assert n["path"] == "n.py"
    assert n["args"] == "--verbose"
    assert n["env"] == {"KEY": "val"}
    assert n["build"] == "make"


def test_node_git_source():
    builder = DataflowBuilder()
    builder.add_node("n").git("https://github.com/example/repo", branch="main", tag="v1.0")
    output = yaml.safe_load(builder.to_yaml())
    n = output["nodes"][0]
    assert n["git"] == "https://github.com/example/repo"
    assert n["branch"] == "main"
    assert n["tag"] == "v1.0"


def test_operator():
    builder = DataflowBuilder()
    op = Operator("detect", python="detect_op.py", send_stdout_as="logs")
    builder.add_node("runtime").add_operator(op)
    output = yaml.safe_load(builder.to_yaml())
    ops = output["nodes"][0]["operators"]
    assert len(ops) == 1
    assert ops[0]["id"] == "detect"
    assert ops[0]["python"] == "detect_op.py"
    assert ops[0]["send_stdout_as"] == "logs"


def test_duplicate_output_not_added_twice():
    node = Node("n")
    node.add_output("out")
    node.add_output("out")
    assert node.config["outputs"].count("out") == 1


def test_context_manager():
    with DataflowBuilder() as builder:
        builder.add_node("n").path("n.py")
    output = yaml.safe_load(builder.to_yaml())
    assert len(output["nodes"]) == 1


def test_to_yaml_file(tmp_path):
    builder = DataflowBuilder()
    builder.add_node("n").path("n.py")
    path = str(tmp_path / "dataflow.yml")
    result = builder.to_yaml(path)
    assert result is None
    with open(path) as f:
        output = yaml.safe_load(f)
    assert output["nodes"][0]["id"] == "n"
