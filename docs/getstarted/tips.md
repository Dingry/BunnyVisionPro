# Tips and Troubleshooting

## Tips for Initialization

??? tip "Good Human Hand Pose for Initialization"

    Extend your arms forward with your palms facing downwards, maintaining a slight separation between your hands. Keep
    your arms and hands in the same position until the calibration is completed. An example can be
    found [here](https://dingry.github.io/BunnyVisionPro/advanced/initialization/#initializing-the-human-frame).

??? tip "Aligning Gravity Direction by Default"

    During this process, the system automatically aligns with the gravity direction of your hands (`align_gravity_dir`
    set to `True`). This means that one of the xyz axis for initial human frame is always aligned with gravity direction. 

## Tips for Teleoperation

??? tip "Initial Use Caution"

    When operating the system for the first time, move your arms slowly to familiarize yourself with its response.
    The robots have velocity limits which can introduce latency, so slow movements help avoid potential collisions.

??? tip "Calibration and Positioning"

    Hand position calibration is based on the world coordinate system and largely independent of the Vision Pro’s location.
    To reach positions beyond your immediate action space, adjust your stance (step back, forward, left, or right) 
    without changing the pose of your hands, preventing unintended hand movements.

??? tip "Hand Pose Strategy"

    Typically, keep your hands parallel. However, for tasks requiring different heights, start with one hand lower
    during calibration—this allows more movement range upward. 
    For example, lower your right hand initially if you need it to move higher during operation.

??? tip "Visibility for Accurate Tracking"

    Ensure all fingers are fully visible to the Vision Pro’s camera for precise hand tracking. Partial visibility can
    result in inaccurate tracking, causing oscillations or unintended robot finger movements.

## Troubleshooting

???+ question "IP of Vision Pro"

    If the IP address displayed in the Tracking Streamer app is incorrect or appears as an IPv6 address, navigate to the
    settings menu on your Vision Pro device: `Settings > WiFi > Details > IP Configuration`. Ensure you obtain the
    correct IPv4 address from this menu.

???+ question "Restarting the Teleop Server After Extended Use"

    The teleoperation `robot_server` is designed to operate continuously, refreshing automatically with each new `InitializationConfig`
    received from the client. However, due to the unpredictable nature of communication, the server may occasionally fail after prolonged use.
    If you observe multiple error messages in the server tab, terminate the server process and restart the `robot_server`. 
    Restart the `vision_server` only if the connection between the computer and VisionPro is disrupted.
