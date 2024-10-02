RUN_ROS2_DEPENDENT_TESTS=false
RUN_INDEPENDENT_TEST=true
RUN_PYXCEL_TEST=true

if $RUN_INDEPENDENT_TEST; then
    python -m pytest -n0  $GU_SOURCE_DIR/tests/unit/independent
fi

if $RUN_INDEPENDENT_TEST; then
    python -m pytest -n0  $GU_SOURCE_DIR/tests/unit/pyxcel
fi

if $RUN_ROS2_DEPENDENT_TESTS; then
    python -m pytest -n0  $GU_SOURCE_DIR/tests/unit/rosa
fi
