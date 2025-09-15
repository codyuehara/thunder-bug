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
test

## RL Controller
