# Model files

Place the int8 ONNX models here.

## traffic_light_int8.onnx

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

## Common

Input : 320x320 letterboxed BGR image, scaled to [0, 1]
Output: YOLOv5/v8 style tensor [1, 4 + num_classes, num_anchors]

These files are intentionally missing. The nodes log a warning and publish
empty/UNKNOWN results when a model file is absent.
