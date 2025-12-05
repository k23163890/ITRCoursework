# Testing Guide for Second Coursework

## Prerequisites
- ROS environment must be set up (source /opt/env.sh or your ROS setup.bash)
- Catkin workspace must be initialized
- All required packages available (YOLOObjectDetection, move_base, etc.)

## Step 1: Build the Package

First, navigate to your catkin workspace and build:

```bash
cd /path/to/your/catkin_ws  # Navigate to your catkin workspace root
catkin_make
source devel/setup.bash
```

## Step 2: Test Individual Components

### Test 1: Verify Services and Actions are Generated

```bash
# Check if services are available
rosservice list | grep find_object

# Check if actions are available
rostopic list | grep check_rules
rostopic list | grep find_object_action
```

### Test 2: Run Main Launch File (Full System)

In one terminal, launch the main system:

```bash
# Make sure you have the video folder path and YOLO coco data path
roslaunch itr_CS_cw2526 itr_CS_cw2526.launch video_folder:=/path/to/videos use_rviz:=true
```

Or if using the supplementary launch file directly:

```bash
roslaunch second_coursework itr_CS_cw2526.launch video_folder:=/path/to/videos
```

### Test 3: Test the FindObject Service

In a new terminal (after sourcing setup.bash):

```bash
# Test the service
rosservice call /find_object "object_name: 'bottle'"
```

Expected response: `request_accepted: True` or `False` depending on if bottle is detected.

### Test 4: Test CheckRules Action

In a new terminal:

```bash
# Send a goal to check_rules action
rosrun actionlib axclient.py /check_rules
```

Or programmatically:

```bash
# Create a simple test client
rostopic pub /check_rules/goal second_coursework/CheckRulesActionGoal "{}" --once
```

You should see:
- Robot navigating between kitchen and bedroom
- Feedback messages when rules are violated (broken_rule: 1 or 2)
- Log messages showing the checking behavior

### Test 5: Test FindObject Action

In a new terminal:

```bash
# Send a goal to find_object_action
rosrun actionlib axclient.py /find_object_action
```

Or using rostopic:

```bash
# Set goal
rostopic pub /find_object_action/goal second_coursework/FindObjectActionGoal "goal:
  object_name: 'bottle'" --once
```

Expected behavior:
- Robot searches for the object using YOLO
- When found, navigates to living room (Room E)
- Announces the room and object using TTS

### Test 6: Test Service Stops CheckRules

```bash
# Terminal 1: Start check_rules action (use actionlib client)
rosrun actionlib axclient.py /check_rules

# Terminal 2: Call the find_object service - should stop check_rules
rosservice call /find_object "object_name: 'bottle'"
```

## Step 3: Monitor Topics and Services

```bash
# List all topics
rostopic list

# Monitor action feedback
rostopic echo /check_rules/feedback

# Monitor action status
rostopic echo /check_rules/status

# Monitor robot position
rostopic echo /odom

# Check YOLO detections
rosservice call /detect_frame
```

## Step 4: Check Logs

All nodes output to screen. Look for:
- "Waiting 10 seconds for localization..."
- "CheckRules action server ready"
- "FindObject action server ready"
- "FindObject service is ready"
- Navigation and detection messages

## Troubleshooting

### If catkin_make fails:
```bash
# Make sure all dependencies are installed
# Check CMakeLists.txt and package.xml for missing dependencies
```

### If nodes don't start:
```bash
# Check ROS master is running
roscore

# Check package is found
rospack find second_coursework
```

### If actions/services not found:
```bash
# Make sure you sourced setup.bash after building
source devel/setup.bash

# Rebuild if needed
catkin_make clean
catkin_make
```

### If YOLO service not available:
```bash
# Check if YOLO node is running
rosnode list | grep yolo
# Make sure YOLOObjectDetection package is built
```

## Using the Test Scripts

You can also use the provided test scripts:

```bash
# Make script executable
chmod +x Supplementary\ files-20251205/student_aa_run_tester.sh
chmod +x Supplementary\ files-20251205/student_aa_run_tester_client-node.sh

# Run tester (if you have a zip file)
bash Supplementary\ files-20251205/student_aa_run_tester.sh your_package.zip /path/to/videos /path/to/coco.data
```

