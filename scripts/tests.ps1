$RUN_ROS2_DEPENDENT_TESTS = $false
$RUN_INDEPENDENT_TEST = $true
$RUN_PYXCEL_TEST = $true

if ($RUN_INDEPENDENT_TEST) {
    python -m pytest -n0 "$env:GU_SOURCE_DIR\tests\unit\independent"
}

if ($RUN_PYXCEL_TEST) {
    python -m pytest -n0 "$env:GU_SOURCE_DIR\tests\unit\pyxcel"
}

if ($RUN_ROS2_DEPENDENT_TESTS) {
    python -m pytest -n0 "$env:GU_SOURCE_DIR\tests\unit\rosa"
}
