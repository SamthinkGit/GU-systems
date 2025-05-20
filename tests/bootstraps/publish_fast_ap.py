import random
import time

from cognition_layer.protocols.broadcaster import publish_step_to_api
from cognition_layer.protocols.fast_ap import FastAPStep

if __name__ == "__main__":
    while True:
        step = FastAPStep(
            name="test",
            content="test_content",
            is_last=random.randint(0, 10),
        )
        publish_step_to_api(step, port=9301)
        time.sleep(2)
