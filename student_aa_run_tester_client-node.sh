#! /bin/bash


# To use: run bash student_aa_run_tester_client-node.sh
# After running the student_aa_run_tester.sh script, run this script to simulate the calls to your action servers.


if [ ! -d "/tmp/aa_ws_test/devel/" ]; then
  echo "You must run the student_aa_run_tester.sh script before running this! Run this node in a new Apptainer terminal once the simulator is running and you have localised the robot."
  exit 1
fi
source /tmp/aa_ws_test/devel/setup.bash

sleep 60
echo "Calling the /find_object service to find a bottle..."
rosservice call /find_object "object_name: 'bottle'"
sleep 60

echo "If you your code ran correctly and without errors, it *should* all be okay (setup-wise)."
echo "If you see any errors, please fix them and resubmit your coursework."