"""Run with the workspace Python bindings: python test_replay.py <dora-binary>."""

from contextlib import ExitStack
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import tempfile
import time
import unittest


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for(read, description):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        value = read()
        if value:
            return value
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {description}")


def stop(child):
    child.terminate()
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait(timeout=5)


class ParamReplay(unittest.TestCase):
    def test_replay(self):
        for mode in ("restart", "crash", "replace"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as tmp:
                self.exercise(mode, Path(tmp))

    def exercise(self, mode, work):
        cli = str(Path(sys.argv[1]).absolute())
        journal = work / "events.jsonl"
        journal.touch()
        crash_pid = work / "crash-pid"
        node = {
            "id": "probe", "path": sys.executable,
            "args": shlex.quote(str(Path(__file__).with_name("observer.py").absolute())),
            "env": {"PARAM_JOURNAL": str(journal), "CRASH_PID": str(crash_pid)},
            "inputs": {"tick": "dora/timer/millis/100"},
            "restart_policy": "on-failure", "max_restarts": 3, "restart_delay": 0.05,
        }
        flow = work / "flow.yml"
        flow.write_text(json.dumps({"nodes": [node]}))
        replacement = work / "node.yml"
        replacement.write_text(json.dumps(node))
        port = free_port()
        env = dict(os.environ, HOME=str(work), USERPROFILE=str(work),
                   XDG_CONFIG_HOME=str(work), DORA_COORDINATOR_PORT=str(port),
                   DORA_COORDINATOR_ADDR="127.0.0.1", TOKIO_WORKER_THREADS="2")

        def command(*args, check=True):
            proc = subprocess.run([cli, *args], env=env, cwd=work, text=True,
                                  capture_output=True, timeout=30)
            if check:
                self.assertEqual(proc.returncode, 0, f"{args}: {proc.stderr}")
            return proc.stdout

        def states():
            return dict(json.loads(line) for line in journal.read_text().splitlines(keepends=True)
                        if line.endswith("\n"))

        def state(pid):
            return states().get(pid, {})

        with ExitStack() as cleanup:
            def service(name, *args):
                child = subprocess.Popen([cli, name, *args], env=env, cwd=work)
                cleanup.callback(stop, child)
                def registered():
                    status = command("status", "--format", "json", check=False)
                    return status and json.loads(status)[name]["status"] == "running"
                wait_for(registered, f"{name} registration")

            service("coordinator", "--port", str(port), "--store", "memory")
            service("daemon", "--local-listen-port", str(free_port()), "--zenoh-no-multicast")
            command("start", str(flow), "--detach", "--name", "replay")
            cleanup.callback(command, "stop", "replay", "--grace-duration", "1s", check=False)
            old = wait_for(lambda: next(iter(states()), None), "original subscriber")
            command("param", "set", "probe", "gain", "1", "-d", "replay")
            command("param", "set", "probe", "gain", "2", "-d", "replay")
            command("param", "set", "probe", "obsolete", "3", "-d", "replay")
            wait_for(lambda: state(old) == {"gain": 2, "obsolete": 3}, "original updates")
            command("param", "delete", "probe", "obsolete", "-d", "replay")
            wait_for(lambda: state(old) == {"gain": 2}, "original deletion")

            if mode == "crash":
                crash_pid.write_text(str(old))
            elif mode == "restart":
                command("node", "restart", "probe", "-d", "replay", "--grace", "1s")
            else:
                command("node", "replace", "probe", "-d", "replay", "--grace", "1s",
                        "--from-yaml", str(replacement))
            new = wait_for(lambda: next((pid for pid in states() if pid != old), None),
                           f"new subscriber after {mode}")
            self.assertEqual(json.loads(command("param", "list", "probe", "-d", "replay",
                                                "--format", "json")), {"gain": 2})
            command("param", "set", "probe", "watermark", "4", "-d", "replay")
            wait_for(lambda: state(new).get("watermark") == 4, f"live update on PID {new}")
            wait_for(lambda: state(new) == {"gain": 2, "watermark": 4},
                     f"stored gain and deletion on PID {new} after {mode}")


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
