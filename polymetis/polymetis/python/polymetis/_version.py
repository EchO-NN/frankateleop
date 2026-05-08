import os
import json

import polymetis

__version__ = ""

# Conda installed: Get version of conda pkg (assigned $GIT_DESCRIBE_NUMBER during build)
if "CONDA_PREFIX" in os.environ and os.environ["CONDA_PREFIX"] in polymetis.__file__:
    # Search conda pkgs for polymetis & extract version number
    stream = os.popen("conda list | grep polymetis")
    for line in stream:
        info_fields = [s for s in line.strip("\n").split(" ") if len(s) > 0]
        if info_fields[0] == "polymetis":  # pkg name == polymetis
            __version__ = info_fields[1]
            break

# Built locally: Retrive git tag description of Polymetis source code
else:
    # Navigate to polymetis pkg dir, which should be within the git repo
    original_cwd = os.getcwd()
    os.chdir(os.path.dirname(polymetis.__file__))

    # Git describe output (fallback-safe when no tag is present)
    stream = os.popen("git describe --tags --always 2>/dev/null")
    version_lines = [line.strip("\n") for line in stream if line.strip("\n")]
    if version_lines:
        version_string = version_lines[0]
        # Common tagged format: v0.1-12-gabc1234 -> 12_gabc1234
        version_items = version_string.split("-")
        if len(version_items) >= 3:
            __version__ = f"{version_items[-2]}_{version_items[-1]}"
        else:
            # Untagged repo may return only commit hash, keep it valid and non-empty
            __version__ = version_string

    # Reset cwd
    os.chdir(original_cwd)

if not __version__:
    raise Exception("Cannot locate Polymetis version!")
