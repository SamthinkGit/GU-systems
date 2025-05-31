from ecm.launch.core.engine import CoreConfig
from ecm.launch.core.status import CoreStatus
from ecm.launch.handler import CoreHandler
from ecm.shared import get_logger

if __name__ == "__main__":
    logger = get_logger("main")
    config = CoreConfig(
        schema="nova",
        tts=False,
        stt=False,
        language="spanish",
    )
    handler = CoreHandler(config)

    handler.start()
    try:
        while handler.is_alive():

            step: CoreStatus | str = handler.get_next_step()
            logger.info(f"Core feedback: {step}")

            if step == CoreStatus.INPUT_REQUIRED.value:
                user_input = input("Prompt: ")
                handler.send(user_input)

            if step == CoreStatus.STEP_COMPLETED.value:
                confirmation = input("Continue execution? (y/n): ")
                if confirmation.lower() != "y":
                    handler.soft_stop()
                    break
                handler.continue_execution()
    finally:
        if handler.is_alive():
            handler.hard_stop()
    print("Process finished.")
