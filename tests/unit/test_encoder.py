import pytest # noqa

from gusyscore.constants import ITEM_ENCODED_PREFIX
from gusysros.tools.registry import ItemEncoder


def test_encoder():
    print("\n\n----- Testing Encoder -----")
    num = 4
    item = object()

    num_code = ItemEncoder.autoencode(num)
    item_code = ItemEncoder.autoencode(item)
    assert num_code == 4
    assert item_code.startswith(ITEM_ENCODED_PREFIX)

    print(f"An integer autoencoded is: {num_code}")
    print(f"The object autoencoded is: {item_code}")

    num_decoded = ItemEncoder.autodecode(num_code)
    item_decoded = ItemEncoder.autodecode(item_code)
    assert num_decoded == num
    assert item_decoded == item

    print(f"Decodification of num: {num}")
    print(f"Decodification of object: {item}")
