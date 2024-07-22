<!-- PROJECT LOGO -->

<p align="center">

  <h1 align="center"><img src="docs/assets/logo/bunny.png" width="80">Bunny-VisionPro: Bimanual Dexterous Teleoperation with Real-Time Retargeting using VisionPro</h1>
  <p align="center">
    <a href="https://dingry.github.io/"><strong>Runyu Ding</strong></a>
    ·
    <a href="https://yzqin.github.io/"><strong>Yuzhe Qin</strong></a>
    ·
    <a href="https://jiyuezh.github.io/"><strong>Jiyue Zhu</strong></a>
    ·
    <a href="https://www.researchgate.net/profile/Chengzhe-Jia"><strong>Chengzhe Jia</strong></a>
    <br>
    <a href="https://aaronyang1223.github.io/"><strong>Shiqi Yang</strong></a>
    ·
    <a href="https://rchalyang.github.io/"><strong>Ruihan Yang</strong></a>
    .
    <a href="https://xjqi.github.io/"><strong>Xiaojuan Qi</strong></a>
    .
    <a href="https://xiaolonw.github.io/"><strong>Xiaolong Wang</strong></a>
  </p>
  <h3 align="center"><a href="https://dingry.github.io/projects/bunny_visionpro.html">Project Page</a> | <a href="https://dingry.github.io/BunnyVisionPro/">Documentation</a> | <a href="https://github.com/Dingry/bunny_teleop_server">Teleop Server Code</a> </h3>
  <div align="center"></div>
</p>

<img src="docs/assets/images/Multi_Robot.webp" alt="Teleoperation with various robots.">

## :wrench: Installation and Usage

Please refer to the [documentation](https://dingry.github.io/BunnyVisionPro/) for detailed installation and usage
instructions.

## :tada: News
- [2024/07/16] :fire: Release of real robot control code for the XArm7 and Ability Hand. Check out [README](real_control/README.md).
- [2024/05/06] Initial release with basic usage of bimanual teleoperation with Vision Pro. Check out
  the [documentation](https://dingry.github.io/BunnyVisionPro/).

## :white_check_mark: TODO

- [x] Tutorial for initialization
- [x] Tutorial for tips and troubleshooting
- [x] Tutorial for retargeting
- [x] Release haptic feedback device design 
- [x] Real control code of XArm7 and Ability Hand
- [ ] Robot collision avoidance
- [ ] Robot arm singularity avoidance
- [ ] Collision-free retargeting

## :seedling: Acknowledgement

Our code is built
upon [VisionProTeleop](https://github.com/Improbable-AI/VisionProTeleop), [dex-retargeting](https://github.com/dexsuite/dex-retargeting), [sim-web-visualizer](https://github.com/NVlabs/sim-web-visualizer).
We thank all these authors for their nicely open sourced code and their great contributions to the community.

## :rabbit: Citation

```
@article{bunny-visionpro,
    title   = {Bunny-VisionPro: Real-Time Bimanual Dexterous Teleoperation for Imitation Learning}, 
    author  = {Runyu Ding and Yuzhe Qin and Jiyue Zhu and Chengzhe Jia and Shiqi Yang and Ruihan Yang and Xiaojuan Qi and Xiaolong Wang},
    year    = {2024},
    url     = {https://arxiv.org/abs/2407.03162}, 
}
```
