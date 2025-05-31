import os

from action_space.tools.image import load_image
from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry
from ecm.tools.registry import Storage
from ecm_communications.bootstraps.autodiscover_pair import autodiscover
from ecm_communications.tools.server import enable_server_mode

logger = get_logger("main")


if __name__ == "__main__":
    autodiscover(role="host", timeout=10)

    ItemRegistry().autoload("screenshot")
    ItemRegistry().autoload("keyboard")
    ItemRegistry().autoload("mouse-simple")
    ItemRegistry().autoload("apps-management")
    ItemRegistry().autoload("spotlight")
    ItemRegistry().autoload("request-selection")
    ItemRegistry().autoload("simple-read-ocr")
    ItemRegistry().autoload("return-response")
    ItemRegistry().autoload("_private_interpreter-tools")

    enable_server_mode()

    Storage("RETURN_RESPONSE_CONFIG")["voice"] = True

    input("Press Enter to start testing actions...")
    logger.info("Testing screenshot")
    action = ItemRegistry().get("screenshot", type="tool")
    screenshot = load_image(action.content())
    screenshot.show()
    input("Press Enter to continue...")

    input("Testing interpreter actions... Press Enter to continue...")
    logger.info("Testing interpreter")
    action = ItemRegistry().get("_invoke_interpreter_with_animation", type="tool")
    action.content(
        query="Check the hour",
        openai_api_key=os.getenv("OPENAI_API_KEY"),
        auto_run=False,
        model="gpt-4o-mini",
    )
    logger.info("Wrote 'Hello World!'")
    input("Press Enter to continue...")

    logger.info("Testing keyboard")
    action = ItemRegistry().get("press_keys", type="action")
    action.content(["A", "B", "C"])
    logger.info("Pressed keys A, B, C")
    input("Press Enter to continue...")

    logger.info("Testing mouse click")
    action = ItemRegistry().get("click")
    action.content(100, 100)
    logger.info("Clicked at (100, 100)")
    input("Press Enter to continue...")

    logger.info("Testing application management")
    action = ItemRegistry().get("get_opened_windows", type="action")
    windows = action.content()
    logger.info(f"Opened windows: {windows}")
    input("Press Enter to continue...")

    action = ItemRegistry().get("search_for_installed_app", type="action")
    result = action.content("edge")
    logger.info(f"Search result for 'edge': {result}")
    input("Press Enter to continue...")

    logger.info("Testing spotlight")
    action = ItemRegistry().get("show_in_screen", type="action")
    action.content(100, 100, 200)
    input("Press Enter to continue...")

    logger.info("Testing request selection")
    action = ItemRegistry().get("request_user_input", type="action")
    action.content("Please select an option", ["Option 1", "Option 2", "Option 3"])
    input("Press Enter to continue...")

    logger.info("Testing OCR")
    action = ItemRegistry().get("read_screen", type="action")
    ocr_result = action.content()
    logger.info(f"OCR Result: {ocr_result}")
    input("Press Enter to continue...")

    logger.info("Testing Response")
    action = ItemRegistry().get("send_response_to_user", type="action")
    action.content("This is a test response to the user.")
    input("Press Enter to finish testing actions...")
