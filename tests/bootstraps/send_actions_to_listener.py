from ecm_communications.bootstraps.autodiscover import autodiscover
from ecm_communications.tools.server import enable_server_mode

from action_space.tools.image import load_image
from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry

logger = get_logger("main")


if __name__ == "__main__":
    autodiscover(allow_localhost=True)

    ItemRegistry().autoload("screenshot")
    ItemRegistry().autoload("keyboard")
    ItemRegistry().autoload("mouse-simple")
    ItemRegistry().autoload("apps-management")
    ItemRegistry().autoload("spotlight")
    ItemRegistry().autoload("request-selection")
    ItemRegistry().autoload("simple-read-ocr")
    ItemRegistry().autoload("return-response")

    enable_server_mode()

    logger.info("Testing screenshot")
    action = ItemRegistry().get("screenshot", type="tool")
    screenshot = load_image(action.content())
    screenshot.show()
    input("Press Enter to continue...")

    action = ItemRegistry().get("write", type="action")
    action.content("Hello World!")
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
