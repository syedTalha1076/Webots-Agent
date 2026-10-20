"""Shared helpers for generated controllers (standard library only; runs inside Webots' Python)."""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))


def load_config():
    with open(os.path.join(HERE, "config.json"), "r", encoding="utf-8") as fh:
        return json.load(fh)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pose2d(node):
    """Ground-truth (x, y, heading) of a node from the Supervisor API (ENU: z up, heading about z)."""
    p = node.getPosition()
    o = node.getOrientation()
    return p[0], p[1], math.atan2(o[3], o[0])


def result_path(name):
    return os.path.join(PROJECT_DIR, "result_%s.json" % name)


def write_result(name, success, reason, metrics):
    data = {"robot": name, "success": bool(success), "reason": reason, "metrics": metrics}
    with open(result_path(name), "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)
    print("[RESULT] %s" % json.dumps(data))
    return data


def all_results_present(names):
    return all(os.path.exists(result_path(n)) for n in names)
