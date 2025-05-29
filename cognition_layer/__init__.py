import importlib
import sys

__import__("pkg_resources").declare_namespace(__name__)

# Added to support backward compatibility for older versions of the package
# This enables the imports:
#
# from cognition_layer.<agent_name>.<dir> import ...
#
# While now the current structure is:
#
# from cognition_layer.agents.<agent_name>.<dir> import ...
#

submodules = [
    "visual_fast_react",
    "again",
    "planex",
    "fast_react",
    "planexv2",
    "xplore",
    "RePlan",
]
for name in submodules:
    full = f"cognition_layer.agents.{name}"
    sys.modules[f"cognition_layer.{name}"] = importlib.import_module(full)
