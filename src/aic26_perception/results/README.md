# Perception results

Live output of mission_controller's result viewer (`~gui:=true`).

Each detection target writes its latest result here on every update, so the
files are always overwritten with the current reading:

    people.json        people result (total / community / intruder / boxes)
    car.json           car result (plate / plate_conf / boxes)
    traffic_light.json traffic light state (model_name / state)
    people.jpg         latest annotated people debug frame (with boxes)
    car.jpg            latest annotated car debug frame (with boxes)
    traffic_light.jpg  latest annotated traffic light debug frame
    summary.json       everything at once, for a quick look at the last state

The `.jpg` files come straight from the detector debug topics and already
have the detection boxes drawn on them.
