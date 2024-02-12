# The block_detector package

This part was done by Fabio Giovanazzi.

## What is it

This *catkin* package exposes a ROS service that detects blocks in the images passed as input using a YOLOv8 model, and returns the detected labeled bounding boxes as output. 

that takes an `sensor_msgs/Image` as input, detects boxes in the image, and
returns `block_detector/DetectBlocks`, i.e. the boxes detected by YOLOv8, as output.

A service was
preferred, instead of a publisher reading directly from the camera and publishing boxes from time
to time, for the following reasons:
- to allow making this component more standalone
- to allow the input image to be chosen by the caller, e.g. by cropping it or passing a custom image
- to avoid wasting CPU on calculating boxes that would never be used, because the subscriber (if any) may not need them

You can start the server with `rosrun block_detector block_detector_server.py`, and after you do that you visualize results from the service using `rosrun block_detector inspect_detected_blocks.py`.

## Message format

The [DetectBlocks.srv](./srv/DetectBlocks.srv) file provides the interface (request/response) of the service:
```srv
sensor_msgs/Image image
---
block_detector/BlockDetectorBox[] boxes
```

Each detected box has this data, see [BlockDetectorBox.msg](./msg/BlockDetectorBox.msg):
```
float64 x1
float64 y1
float64 x2
float64 y2
string label
float64 confidence
```

## Model

See [vision_training/README.md](../../vision_training/README.md) for information about how the model was trained. The model is placed in `./block_detector_model/model.pt` for loading at runtime by python.
