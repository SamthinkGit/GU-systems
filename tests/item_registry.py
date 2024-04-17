from gusysros.tools.registry import ItemRegistry

if __name__ == "__main__":

    print("----- Testing ItemRegistry -----")
    registry = ItemRegistry()

    @ItemRegistry.register_function
    def myfunc(message):
        print("Hello Item Registry!")
        print(f"I will print a message: {message}")

    id = ItemRegistry.get_id(myfunc)
    print(f"The new function myfunc has id: {id}")
    print("Executing action...")

    registry.call(id, message="Hi! It looks like it is working ^^")
