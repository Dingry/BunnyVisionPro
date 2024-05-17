# Hand Retargeting

This example demonstrates how to use data exclusively from Apple Vision Pro to manipulate bimanual robot hands, omitting
the robot arm component. We capture some offline Vision Pro data and employ it to create retargeting visualizations as
shown below.

![bimanual_retargeting](../assets/videos/teaser.webp)

## Installation

We provide an offline data file, `data/offline_avp_stream.pkl`, which contains
recorded Vision Pro streams. This data is used to generate trajectories for robot hand joints. To process the data, you
will need specific robot hand
models.

```shell
cd examples/retargeting

# Download robot hand models
git clone https://github.com/dexsuite/dex-urdf

# Install additional Python dependencies needed for this example
pip install sapien dex_retargeting tyro opencv-python
```

## Retargeting Human Hand Data

Use the following commands to convert recorded human hand data into robot hand joint
trajectories:

```shell
python retargeting.py --robot-name allegro --data-path data/offline_avp_stream.pkl --output-path data/allegro.pkl

# You can also do that for other robot hands.
python retargeting.py --robot-name ability --data-path data/offline_avp_stream.pkl --output-path data/ability.pkl
python retargeting.py --robot-name svh --data-path data/offline_avp_stream.pkl --output-path data/svh.pkl 
```

## Visualize Retargeting Results

We also provide scripts to convert the retargeting data into videos for visualization, such
as `data/allegro.mp4`. Check out the video to see the results!

```shell
python render_retargeting.py --pickle-path data/allegro.pkl --output_video_path data/allegro.mp4 

# You can also do that for other robot hands.
python render_retargeting.py --pickle-path data/ability.pkl --output_video_path data/ability.mp4 
python render_retargeting.py --pickle-path data/svh.pkl --output_video_path data/svh.mp4 
```

!!! tip "Record Your Own Vision Pro Data"

    If you have a Vision Pro, you can record your own data by opening the Tracking Stream App as described in the [Basic Usage Section](https://dingry.github.io/BunnyVisionPro/getstarted/usage/#3-launch-tracking-streamer-app).
    Then, run the command below to save your data: `python save_offline_avp_stream.py --avp_ip=YOUR_VISION_PRO_IP`
    This will generate a file `data/your_offline_avp_stream.pkl` in the provided format.
