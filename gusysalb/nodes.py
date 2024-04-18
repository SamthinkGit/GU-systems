"""
NodeRegistry
==============================

This module serves as a simple proxy registry for initialized nodes in a system,
providing a central reference point to access and manage these nodes dynamically
and avoiding circular imports.
"""
from typing import Any
from typing import Dict


class NodeRegistry:
    inited_nodes: Dict[str, Any] = {}
