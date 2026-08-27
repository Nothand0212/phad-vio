#!/usr/bin/env python3

import json
import os
from pathlib import Path
import subprocess
import sys


PACKAGES = (
    "cmake",
    "g++-13",
    "libboost-serialization-dev",
    "libeigen3-dev",
    "libgtest-dev",
    "libopencv-dev",
    "libssl-dev",
    "libyaml-cpp-dev",
    "ninja-build",
    "nlohmann-json3-dev",
    "python3",
    "python3-numpy",
)


def package_version(package: str) -> str:
    return subprocess.check_output(
        ["dpkg-query", "-W", "-f=${Version}", package], text=True
    ).strip()


output = Path(sys.argv[1])
versions = {package: package_version(package) for package in PACKAGES}
versions["gtsam_commit"] = os.environ["GTSAM_COMMIT"]
versions["python_packages"] = subprocess.check_output(
    ["/opt/phad-venv/bin/pip", "freeze", "--all"], text=True
).splitlines()
output.write_text(
    json.dumps(versions, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
