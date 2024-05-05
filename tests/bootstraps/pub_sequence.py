from sequence_action_server.sequence_publisher import SequencePublisher

from icecream import ic  # noqa

from execution_layer.rosa.gateway import test_function
from execution_layer.rosa.interfaces.alb import ALB
from execution_layer.rosa.interfaces.nodes import NodeRegistry
from execution_layer.rosa.ros2 import ActionPackage
from execution_layer.rosa.ros2 import ItemRegistry
from execution_layer.rosa.ros2 import SequencePackage
from execution_layer.rosa.ros2 import SequencePriority
from execution_layer.rosa.ros2 import SimpleSequence


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
