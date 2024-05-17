#!/bin/bash
GRASPNET_DATA_DIR="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/"
GRASPNET_COLOR_PATH="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/color.png"
GRASPNET_DEPTH_PATH="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/depth.png"
GRASPNET_WORKSPACE_PATH="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/workspace_mask.png"
TRANSFORMATION_PATH="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/transformation_matrix.npy" 
WIDTH_PATH="/home/yara/camera_ws/src/graspnet-baseline/doc/example_data/width.txt"

EGOHOS_IMGS_DIR="/home/yara/camera_ws/src/EgoHOS/testimages/images"
EGOHOS_IMG_PATH="/home/yara/camera_ws/src/EgoHOS/testimages/images/color.png"
EGOHOS_OBJ_PATH="/home/yara/camera_ws/src/EgoHOS/testimages/pred_obj1/workspace_mask.png"

LATEST_COLOR_PATH="/home/yara/camera_ws/src/visual_tools/docs/latest_imgs/color.png"
LATEST_DEPTH_PATH="/home/yara/camera_ws/src/visual_tools/docs/latest_imgs/depth.png"
COMMAND_FILE_PATH="/home/yara/camera_ws/src/visual_tools/docs/latest_imgs/command.txt"

trap 'cleanup && echo "clean exit"' EXIT

echo "================ Initializing Process"

# cleaning the graspnet and egohos dir
rm -f "$GRASPNET_COLOR_PATH"
rm -f "$GRASPNET_DEPTH_PATH"
rm -f "$GRASPNET_WORKSPACE_PATH"
rm -f "$TRANSFORMATION_PATH"
rm -f "$WIDTH_PATH"
rm -f "$EGOHOS_OBJ_PATH"
rm -f "$EGOHOS_IMG_PATH"

# Initialize RealSense
python /home/yara/camera_ws/src/visual_tools/scripts/capture_img.py 2>&1 &

# Initialize ROS Publisher
rosrun visual_tools broadcast_transform.py 2>&1 &

# Initialize EGOHOS Models and server
source /"home/yara/camera_ws/src/EgoHOS/seg_venv/bin/activate"
python /home/yara/camera_ws/src/EgoHOS/mmsegmentation/predict_image_served.py 2>&1 &
deactivate

# Initialize GRASPNET
source /home/yara/camera_ws/src/graspnet-baseline/graspnet_venv/bin/activate
python /home/yara/camera_ws/src/graspnet-baseline/demo_served.py 2>&1 &
deactivate

cleanup(){
    echo "Terminating all processes..."
    pkill -f "/home/yara/camera_ws/src/visual_tools/scripts/capture_img.py"
    pkill -f "/home/yara/camera_ws/src/EgoHOS/mmsegmentation/predict_image_served.py"
    pkill -f "/home/yara/camera_ws/src/graspnet-baseline/demo_served.py"
    rosnode kill transform_publisher
    exit 0
}

# Infinite loop
while true; do
    # wait for user input
    echo "Press any key to start the handover process. or c to terminate."
    read userInput
    if [[ $userInput == "c" ]]; then
        cleanup
    fi
    echo "Got it. Starting..."

    # cleaning the graspnet and egohos dir
    rm -f "$GRASPNET_COLOR_PATH"
    rm -f "$GRASPNET_DEPTH_PATH"
    rm -f "$GRASPNET_WORKSPACE_PATH"
    rm -f "$TRANSFORMATION_PATH"
    rm -f "$WIDTH_PATH"
    rm -f "$EGOHOS_OBJ_PATH"
    rm -f "$EGOHOS_IMG_PATH"

    # getting the latest captured image
    touch "$COMMAND_FILE_PATH"
    sleep 3
    # move color to graspnet
    cp "$LATEST_COLOR_PATH" "$GRASPNET_DATA_DIR"
    # move color to egohos
    cp "$LATEST_COLOR_PATH" "$EGOHOS_IMGS_DIR"
    # move depth to graspnet
    cp "$LATEST_DEPTH_PATH" "$GRASPNET_DATA_DIR"
    echo "============== Image captured."

    # wait till the segmented image exists
    while [ ! -f "$EGOHOS_OBJ_PATH" ]; do
        sleep 1  # Adjust the sleep duration as needed
    done
    sleep 2

    cp "$EGOHOS_OBJ_PATH" "$GRASPNET_DATA_DIR"
    echo "============== Image segmented"
    feh "$GRASPNET_COLOR_PATH" "$GRASPNET_DEPTH_PATH" "$GRASPNET_WORKSPACE_PATH" &

    # timeout meechanism
    SUCCESS=0
    start_time=$(date +%s)
    end_time=$((start_time + 15))
    while [ $(date +%s) -lt $end_time ]; do
        if [ -f "$TRANSFORMATION_PATH" ]; then
            SUCCESS=1
            break
        fi
        sleep 1
    done

    if [ "$SUCCESS" -eq 0 ]; then
        echo "GRASPNET ERROR: timeout"
        continue
    fi

    while [ ! -f "$WIDTH_PATH" ]; do
        sleep 1  # Adjust the sleep duration as needed
    done
    echo "============== Grasp Pose Generated."

    # read the generated pose and publish
    echo "Broadcasting the generated transform from frame camera_link to frame obj_grasp..."
    rosparam set /handover_status false
    echo "============== DONE."

done





