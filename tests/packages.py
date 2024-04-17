import json

from gusyscore.gateway.mocks import gateway
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.packages import SequenceType
from gusysros.tools.packages import TaskRegistry
from gusysros.tools.registry import ItemRegistry

if __name__ == '__main__':
    with gateway.OneFileWorkspaceMock(temporal=True) as ws:
        action_1 = ActionPackage(
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{
                'workspace': ws,
                'text': 'Hello World!'
            }
        )

        action_2 = ActionPackage(
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{
                'workspace': ws,
                'text': 'Hello Action Package!'
            }
        )

        seq = SequencePackage(
            task_id='my_sequence',
            type=SequenceType.SIMPLE_SEQUENCE,
            priority=SequencePriority.NORMAL,
            actions=[action_1, action_2]
        )

        tr = TaskRegistry()
        tr.append(seq)
        tr.append(seq)
        tr.append(seq)

        pkg = seq.to_json()
        print(" JSON Package Format ".center(60, "-"))
        print(pkg)

        decoded_seq = SequencePackage.from_json(pkg)
        print(" Decoded Package ".center(60, "-"))
        print(decoded_seq)

        print(" TaskRegistry ".center(60, "-"))
        print(json.dumps(tr.get_log(), indent=4))
