# rdj2025_potato_disease_detection

A ROS 2 Humble package for **potato disease detection** using a machine learning model.  
The package subscribes to an image topic (`/image`), runs inference, and publishes results to `/inference_result` and '/inference_image'

## 📦 Package Overview

This package contains:

- **`potato_disease_detection_node`**  
  - Subscribes to `/image` (`sensor_msgs/Image`)  
  - Converts the image into OpenCV format (via `cv_bridge`)  
  - Runs inference using a provided ML model (you can replace with your own)
  - Publishes results on `/inference_result` (`std_msgs/String`)
  - Publishes results on `/inference_image` (`sensor_msgs/String`)

- **`test_image_publisher`**  
  - Publishes a static test image (`potato.jpg` or another local image)  
  - Sends it to `/image` for testing the detection node
    
- **`rtsp_image_publisher`** 
  - Connects to an RTSP stream from the Raspberry Pi camera  
  - Displays a live preview in an OpenCV window  
  - On pressing **`d`**, saves the current frame to a configurable directory and publishes it to `/image`  
  - On pressing **`q`**, exits cleanly  
  - Accepts the Pi’s IP address as a ROS 2 parameter (`pi_ip`)

## Dependencies

⚠️ Note: `cv_bridge` currently only supports **NumPy < 2.0**.  
If you see `_ARRAY_API not found` errors, downgrade NumPy:

```bash
pip install "numpy<2" --force-reinstall
```

Also, install PyTorch (Used for Inference)

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

## Build Instructions

# Go back to root (parent) workspace directory
cd ~/your_root_directory

colcon build
source install/setup.bash
```

## Running the Nodes

### 1. Run the Inference Node

```bash
ros2 run rdj2025_potato_disease_detection potato_disease_detection_node
```

This will start the potato disease detection node. It will wait for images on `/image`.
On receiving an image it will run an inference displaying a text on the terminal. 
An annotated image will also be published to the '/inference_image' topic which can be view in RVIZ2 under the 'Image' display plugin

### 2. Publish a Test Image

In a new terminal (with workspace sourced):

```bash
 ros2 run rdj2025_potato_disease_detection publish_test_image  --ros-args -p pi_ip:=<your_pi_ip>

```

This will publish `xxxxxx.jpg` (make sure the file exists in the working directory).  
It will also automatically publish an image on the '/Image' topic every time 'd' is pressed.

### 3. Inspect the Inference Results

Use another terminal to view the published inference results:

```bash
ros2 topic echo /inference_result
```


##  Visualizing Images

You can view the published images with:

```bash
ros2 run rqt_image_view rqt_image_view
```


## Next Steps

    
-   Deploy on a robot with a camera publishing to `/image`.
    

