from gusysros.tools.registry import ItemEncoder

if __name__ == '__main__':
    print("\n\n----- Testing Encoder -----")
    num = 4
    item = object()

    num_code = ItemEncoder.autoencode(num)
    item_code = ItemEncoder.autoencode(item)

    print(f"An integer autoencoded is: {num_code}")
    print(f"The object autoencoded is: {item_code}")

    num = ItemEncoder.autodecode(num_code)
    item = ItemEncoder.autodecode(item_code)

    print(f"Decodification of num: {num}")
    print(f"Decodification of object: {item}")
