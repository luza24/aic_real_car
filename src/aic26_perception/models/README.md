# Model files

Place the int8 ONNX models here.

## traffic_int8.onnx

Class order used by traffic_light_detector.py:

    0: red_off
    1: red_on
    2: green_off
    3: green_on
    4: yellow_off
    5: yellow_on

Each traffic light shows three lens patches (red, green, yellow) at the same
time, so the detector labels every patch with its own on/off state. The light
state is then derived from which patch is on.

## people_int8.onnx

Class order used by people_detector.py:

    p1 .. p16, pF1, pF2

Every person model has its own class. Each class is counted at most once per
detection. pF1 and pF2 are the non-community people, all the others belong to
the community.

## car_int8.onnx

Class order used by car_detector.py:

    car

A single class. The cars in the scene are rear view pictures, they all look
the same and only the license plate differs, so the plate cannot be told apart
from the class name. The detector therefore crops the plate region out of the
detected car box and reads it with easyocr (Chinese simplified + English), so
the result is real UTF-8 text such as 苏A·B8Q62.

The plate box is a fixed fraction of the car box (`~plate_roi`, default
[0.30, 0.40, 0.70, 0.60]); keep it in sync with models/car1, car2, car3 in
aic26_gazebo.

    models/car1 -> 苏A·B8Q62
    models/car2 -> 鄂D·7B5Q2
    models/car3 -> 苏A·PL12A

## Common

Input : 640x640 letterboxed BGR image, scaled to [0, 1]
Output: YOLOv5/v8 style tensor [1, 4 + num_classes, num_anchors]

The nodes read the fixed input size straight from the model, so the
``~input_size`` parameter (default 640) is only a fallback and is overridden
by the value exported in the ONNX graph.

Backend: OpenVINO is preferred on Intel CPUs.  The three models are int8,
which OpenCV 4.2 `dnn` cannot load (it fails in `getMatFromTensor`), so at
least one of OpenVINO or onnxruntime must be installed:

    pip install openvino          # preferred on this machine
    pip install onnxruntime       # portable CPU fallback

Measured on the AIC26 Intel N100 (4 cores, 640x640 int8, full
letterbox + inference + decode pipeline):

    OpenVINO CPU, 2 threads: ~94-98 ms/frame  (~10 FPS)
    onnxruntime CPU:         ~253-325 ms/frame (~3-4 FPS)

`AIC26_OV_THREADS` selects the OpenVINO thread count (default 2, leaving two
cores for ROS/navigation).  The N100 has no NVIDIA GPU and the Intel iGPU has
no OpenCL runtime, so OpenVINO's GPU plugin is not available; everything runs
on the CPU.

The detectors only run while `/perception/detect` requests their target and
are limited to `~max_rate` (default 5 Hz), so 10 FPS inference is enough and
does not build a 30 FPS queue.  The debug images are published at that rate.

The three `*_int8.onnx` files are tracked in this repository. The nodes log a
warning and publish empty / `UNKNOWN` results when a model file is absent or the
runtime backend cannot load it.

## Runtime dependency

car_detector.py needs easyocr at runtime (CPU is fine):

    pip install easyocr

easyocr downloads its own detector and recognition weights on first use
(`~/.EasyOCR`), so the first plate read needs network access or a pre-populated
cache. The node starts without easyocr and simply reports an empty plate.
