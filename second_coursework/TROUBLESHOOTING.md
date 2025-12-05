# Troubleshooting Guide

## Issue: "Waiting for /detect_frame service..." hangs

**Problem:** The code hangs waiting for the YOLO detection service.

**Solution:**
1. **Check if YOLO node is running:**
   ```bash
   rosnode list | grep yolo
   rosservice list | grep detect_frame
   ```

2. **The YOLO node should be launched automatically** in `student_nodes.launch`. If not, manually start it:
   ```bash
   rosrun YOLOObjectDetection YoloNavigationNode.py
   ```

3. **Check if video/image feed is available:**
   The YOLO node needs images from `/usb_cam/image_raw` topic:
   ```bash
   rostopic echo /usb_cam/image_raw
   ```

4. **Verify the launch file** includes the YOLO node:
   Check that `student_nodes.launch` includes:
   ```xml
   <node pkg="YOLOObjectDetection" type="YoloNavigationNode.py" name="yolo_detection_node" output="screen">
       <param name="coco_data" value="$(find YOLOObjectDetection)/config/coco.data"/>
   </node>
   ```

## Issue: Service timeout errors

**Problem:** Timeout waiting for services.

**Solutions:**
- Make sure all required nodes are launched
- Check ROS master is running: `roscore`
- Verify package is built: `catkin_make`
- Source workspace: `source devel/setup.bash`

## Issue: YOLO service returns empty detections

**Problem:** `/detect_frame` service works but returns no detections.

**Solutions:**
- Check if images are being published: `rostopic hz /usb_cam/image_raw`
- Verify YOLO node received images (check logs)
- Make sure coco.data path is correct
- Check YOLO model files exist

## Issue: Nodes not starting

**Problem:** Nodes fail to start or exit immediately.

**Solutions:**
- Check ROS master is running
- Verify all dependencies are installed
- Check package is built: `catkin_make`
- Look at node logs for error messages
- Ensure scripts are executable: `chmod +x scripts/*.py`

## Issue: Action servers not responding

**Problem:** Actions don't respond or timeout.

**Solutions:**
- Check if action servers started: `rostopic list | grep action`
- Verify action definitions are built
- Check if dependencies (move_base, etc.) are available
- Review node logs for errors

