"""
Core Engine for ECM Launch
=========================
This module provides the core engine for the ECM launch process, allowing for
the initialization, execution, and control of the core logic in a multiprocessing environment.

Please import the `Core` class from this module to use it in your application.
"""
from ecm.launch.core.nova_core_2 import CoreConfig as CoreConfig  # noqa
from ecm.launch.core.nova_core_2 import NovaCore2 as NovaCore  # noqa
