from ecm_communications.bootstraps.autodiscover import autodiscover
from ecm_communications.tools.listener import Listener

from ecm.tools.registry import ItemRegistry
from ecm.tools.registry import Storage

if __name__ == "__main__":

    Storage("ITEM_REGISTRY_CONFIG")["autoload-static"] = True
    ItemRegistry().autoload("screenshot")
    ItemRegistry().autoload("keyboard")
    ItemRegistry().autoload("mouse-simple")
    ItemRegistry().autoload("apps-management")
    ItemRegistry().autoload("spotlight")
    ItemRegistry().autoload("request-selection")
    ItemRegistry().autoload("molmo_element_finder_multiple_actions")
    ItemRegistry().autoload("simple-read-ocr")
    ItemRegistry().autoload("return-response")
    autodiscover(allow_localhost=False)
    listener = Listener()
    listener.listen()
    exit()
