from aiuda import aiuda

import ecm.exelent.parser as parser
from ecm.shared import get_root_path


if __name__ == "__main__":
    path = get_root_path() / "tests" / "resources" / "hello_world.xlnt"
    task = parser.parse(path)
    linear_task = parser.linerize_task(task)
    aiuda.tree(task)
    aiuda.tree(linear_task)
