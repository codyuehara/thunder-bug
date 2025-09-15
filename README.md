# thunder-bug

## Quadrotor Sim
The simulator adheres to the following dynamics:
INSERT
Then publishes to ros2 topic /drone_state where the controller node will receive it. 
To control the quadrotor, publish to /thrust_and_body_rates

### Running the Sim
```
ros2 launch racetrack quadrotor_sim.launch.py
```

## Sensors
Mono camera and IMU

## Gate Detection
### Before flight
Dataset with images of gates that are annotated for 4 corners + edges (can also have multiple gates in same frame) using CVAT and outputs COCO Json file.
UNet model is trained using annotated dataset.

### During live flight
Subscribes to /image_raw where the camera is outputting its frames to and plugs into UNet model to extract 4 corners of gate + PAFs.
Determines where the next gate is and publishes positions of the next gate's 4 corners to /gate_obs

## RL Controller
Subscribes to both /drone_state and /gate_obs and concatenates together to create R^31 observation vector
Outputs action (thrust and body rates) to /thrust_and_body_rates
