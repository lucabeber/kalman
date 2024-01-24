rm -rf build && mkdir build \
    && cd build && cmake .. \
    && make -j && ./kalman_test && cd .. \
    && python3 ./examples/Robot1/visualization.py --path_to_exec build --makeplot
