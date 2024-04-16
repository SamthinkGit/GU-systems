from gusysalb.alb import ALB
from gusyscore.gateway.mocks.debug import append_to_list
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import SequenceType
from gusysros.types.basic import SimpleSequence


def main():
    # Build the environment
    alb = ALB()
    alb.load_types()
    alb.load_mocks()

    # Build a mock
    my_list = []

    action_1 = ActionPackage(
        action_id=ItemRegistry.get_id(append_to_list),
        list=my_list,
        value="A"
    )
    action_2 = ActionPackage(
        action_id=ItemRegistry.get_id(append_to_list),
        list=my_list,
        value="B"
    )

    pkg = SequencePackage(
        task_id='my_task',
        type=SimpleSequence.get_type(),
        priority=SequencePriority.NORMAL,
        actions=[
            action_1,
            action_2
        ]
    )

    # Execute the package (Assuming we don't know about the content)
    seq_type = SequenceType.from_pkg(pkg)
    seq_type.run()

    # Check execution
    expected_list = ['A', 'B']
    assert my_list == expected_list, (
        f'Target has not been properly modified.\nExpected: {expected_list}\nObtained:{my_list}'
    )

    print(f"List properly modified to {my_list}")


if __name__ == '__main__':
    main()
