from sequence_action_server.sequence_publisher import SequencePublisher

from icecream import ic  # noqa

from gusysalb.alb import ALB
from gusysalb.nodes import NodeRegistry
from gusyscore.gateway.mocks.debug import test_function
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import SimpleSequence


def main():
    alb = ALB()
    alb.build_all()

    print("Starting node...")
    publisher: SequencePublisher = NodeRegistry.inited_nodes["sequence_publisher"]

    actions = [
        ActionPackage(action_id=ItemRegistry.get_id(test_function), num=num, sleep=1)
        for num in range(4)
    ]

    seq = SequencePackage(
        task_id="my_task",
        type=SimpleSequence.get_type(),
        priority=SequencePriority.NORMAL,
        actions=actions,
    )

    print("Starting node...")
    print("Use ENTER to send mock packages")
    while True:
        input()
        publisher.send_package(seq)


if __name__ == "__main__":
    main()
