from gusysros.tools.registry import ThreadRegistry


def my_function(num):
    print(f"Num received is {num}")
    print(f"Executed function with task_id: {ThreadRegistry.get_task_id()}")


if __name__ == '__main__':

    task_id = 'my_task'

    tr = ThreadRegistry()
    tr.watch(
        task_id=task_id,
        target=my_function,
        num=5
    )
    print(f"Father task_id is {ThreadRegistry.get_task_id()}")
    print("Waiting for task...")
    thread = tr.wait(task_id)
    exit()
