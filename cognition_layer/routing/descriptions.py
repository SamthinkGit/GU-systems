from cognition_layer.agents.fast_react.description import (
    DEPLOY_MODEL as FR_DEPLOY_MODEL,
)
from cognition_layer.agents.minimal_vfr.darkvfr_description import (
    DEPLOY_MODEL as DVFR_DEPLOY_MODEL,
)
from cognition_layer.agents.visual_fast_react.description import (
    DEPLOY_MODEL as VFR_DEPLOY_MODEL,
)
from cognition_layer.experts.small_vision_agent.description import (
    DEPLOY_MODEL as SVA_DEPLOY_MODEL,
)

DEPLOY_MODELS = [
    DVFR_DEPLOY_MODEL,
    FR_DEPLOY_MODEL,
    VFR_DEPLOY_MODEL,
    SVA_DEPLOY_MODEL,
]
