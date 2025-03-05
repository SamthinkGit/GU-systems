import subprocess
from ecm.shared import get_root_path

RUN_ROS2_DEPENDENT_TESTS = False
RUN_INDEPENDENT_TEST = True
RUN_PYXCEL_TEST = True

GU_SOURCE_DIR = get_root_path().absolute()

if RUN_INDEPENDENT_TEST:
    subprocess.run(
        ["python", "-m", "pytest", "-n0", f"{GU_SOURCE_DIR}/tests/unit/independent"]
    )

if RUN_PYXCEL_TEST:
    subprocess.run(
        ["python", "-m", "pytest", "-n0", f"{GU_SOURCE_DIR}/tests/unit/pyxcel"]
    )

if RUN_ROS2_DEPENDENT_TESTS:
    subprocess.run(
        ["python", "-m", "pytest", "-n0", f"{GU_SOURCE_DIR}/tests/unit/rosa"]
    )
