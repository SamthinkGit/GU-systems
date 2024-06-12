from aiuda import aiuda

import ecm.exelent.parser as parser
from execution_layer.rosa.shared import get_root_path


if __name__ == "__main__":
    path = get_root_path() / "tests" / "resources" / "build_pickaxe.xlnt"
    aiuda.tree(parser.parse(path))
