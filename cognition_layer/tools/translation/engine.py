# flake8: noqa
"""
Translate Module
==============================
This module aims to be a mediator to select the module with the best Translation Mechanism.
It provides an interface to work with the current translation engine.

To import a Translation Engine use:
from cognition_layer.tools.translation.engine import translate

Do not try to import directly from the engine directory, so we can keep backward
compatibility
"""
from cognition_layer.tools.translation.zeroshot_translate.zeroshot_translate import (
    translate as translate,
)
