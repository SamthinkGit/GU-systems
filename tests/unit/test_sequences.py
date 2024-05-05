import json

import rclpy

from execution_layer.rosa.build.alb import ALB
from execution_layer.rosa.constants import ITEM_ENCODED_PREFIX
from execution_layer.rosa.gateway.mocks import gateway
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.tools.packages import TaskRegistry
from execution_layer.rosa.ros2.tools.packages import TaskStatus
from execution_layer.rosa.ros2.tools.registry import ItemRegistry
from execution_layer.rosa.ros2.types.basic import SequenceType
from execution_layer.rosa.ros2.types.basic import SimpleSequence
from tests.mocks.packages import PackageMock


def test_sequence_building():
    with gateway.OneFileWorkspaceMock(temporal=True) as ws:

        # --- Initializing Building ---
        action_1 = ActionPackage(
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{"workspace": ws, "text": "Hello World!"}
        )
        encoded_ws: str = action_1.to_dict()["kwargs"]["workspace"]
        assert encoded_ws.startswith(ITEM_ENCODED_PREFIX)

        action_2 = ActionPackage(
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{"workspace": ws, "text": "Hello Action Package!"}
        )

        seq = SequencePackage(
            task_id="my_sequence",
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=[action_1, action_2],
        )

        tr = TaskRegistry()
        tr.append(seq)
        tr.append(seq)
        tr.append(seq)

        # --- Testing Formation ---
        pkg = seq.to_json()
        print(" JSON Package Format ".center(60, "-"))
        print(pkg)
        try:
            json.loads(pkg)
            assert True
        except Exception:
            assert False

        decoded_seq = SequencePackage.from_json(pkg)
        print(" Decoded Package ".center(60, "-"))
        print(decoded_seq)
        assert decoded_seq.task_id == seq.task_id
        assert decoded_seq.priority == seq.priority
        assert decoded_seq.actions[0].kwargs["workspace"] == ws

        print(" TaskRegistry ".center(60, "-"))
        print(json.dumps(tr.get_log(), indent=4))
        assert len(tr.tasks) == 1
        assert tr.tasks["my_sequence"].status == TaskStatus.READY


def test_sequence_types():

    alb = ALB()
    alb.build_all()

    # Check AutoType detection
    mock = PackageMock()
    seq_type = SequenceType.from_pkg(mock.pkg)
    assert seq_type.get_type() == mock.type

    seq_type.run()
    assert mock.list_properly_modified()
    rclpy.shutdown()
