from ecm.constants import PATCH_ITEM_REGISTRY_V1

if PATCH_ITEM_REGISTRY_V1:
    from ecm.tools.item_registry_v2 import ItemRegistry
else:
    from ecm.tools.item_registry_v1 import ThreadRegistry # noqa
    from ecm.tools.item_registry_v1 import ItemRegistry # noqa
    from ecm.tools.item_registry_v1 import ItemEncoder # noqa
