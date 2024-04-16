from gusysros.tools.registry import ItemRegistry


@ItemRegistry.register_function
def print_two_keywords(keyword_1, keyword_2):
    print("My keyword_1 is", keyword_1)
    print("My keyword_2 is", keyword_2)


@ItemRegistry.register_function
def hello_world():
    print("Hello World!")
