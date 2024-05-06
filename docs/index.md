# BunnyVisionPro

BunnyVisionPro is a robot teleoperation system integrated with Apple Vision Pro as pose tracking device, specifically
tailored for bimanual dexterous manipulation. The system boasts a highly modular architecture, simplifying deployment
and enhancing flexibility. Key features of the system include:

- Portability: The system only requires a Vision Pro as hand pose tracking device, making it highly portable and easy to
  set up.
- Scalability: The system is designed to be compatible with a wide range of robot platforms, including different robot
  arms and hands.
- Containization: The server and client components of the system are containerized, ensuring easy deployment and
  management.

Begin using BunnyVisionPro by following [the simple installation guide](./getstarted/install.md).

## Citation

If you find this codebase helpful in your work, please consider cite:

```bibtex
@article{bunny-visionpro,
  author  = {Runyu Ding, Yuzhe Qin, Jiyue Zhu, Chengzhe Jia, Shiqi Yang, Ruihan Yang, Xiaojuan Qi, and Xiaolong Wang},
  title   = {Bunny-VisionPro: Bimanual Dexterous Teleoperation with Real-Time Retargeting using Vision Pro},
 year   = {2024},
}
```

## Acknowledge

Our code is built upon
[VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop),
[dex-retargeting](https://github.com/dexsuite/dex-retargeting),
[sim-web-visualizer](https://github.com/NVlabs/sim-web-visualizer).
We thank all these authors for their nicely open sourced code and their great contributions to the community.
