#!/usr/bin/env bash
# Run in the Jazzy validation image, with the sibling repositories mounted at /repos.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
mkdir -p /ws/src
for name in compliant_controllers ros2_kortex frank-robot-ros-2 frank_gello; do
  test -d "/repos/$name"
  ln -sfn "/repos/$name" "/ws/src/$name"
done
# Only the model is needed; avoid building unrelated external Robotiq hardware drivers.
ln -sfn /repos/robotiq-ros/grippers/robotiq_description /ws/src/robotiq-ros
cd /ws
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select kortex_api kortex_driver kortex_description compliant_controllers \
  robotiq_description frank_description frank_moveit_config frank_kortex frank_interface frank_ik frank_gello \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=OFF
source /ws/install/setup.bash
colcon test --packages-select compliant_controllers kortex_driver frank_description frank_kortex frank_ik \
  --event-handlers console_direct+
colcon test-result --all --verbose
python3 -m pytest -q /repos/frank_gello/test/test_calibration.py /repos/frank_gello/test/test_lowpass.py \
  /repos/frank_gello/test/test_driver.py /repos/frank_gello/test/test_safety.py /repos/frank_gello/test/test_grippers.py
