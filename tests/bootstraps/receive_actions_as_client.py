from ecm_communications.bootstraps.autodiscover import autodiscover
from ecm_communications.tools.listener import Listener

from ecm.tools.registry import ItemRegistry

if __name__ == "__main__":

    ItemRegistry().autoload("screenshot")
    ItemRegistry().autoload("keyboard")
    ItemRegistry().autoload("mouse-simple")
    ItemRegistry().autoload("apps-management")
    ItemRegistry().autoload("spotlight")
    ItemRegistry().autoload("request-selection")
    ItemRegistry().autoload("molmo_element_finder_multiple_actions")
    ItemRegistry().autoload("simple-read-ocr")
    ItemRegistry().autoload("return-response")
    autodiscover(allow_localhost=True)
    listener = Listener()
    listener.listen()
    exit()
