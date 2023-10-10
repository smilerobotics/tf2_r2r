# tf2_r2r

[![Build Status](https://img.shields.io/github/actions/workflow/status/smilerobotics/tf2_r2r/ci.yml?branch=main&logo=github)](https://github.com/smilerobotics/tf2_r2r/actions) [![crates.io](https://img.shields.io/crates/v/tf2_r2r.svg?logo=rust)](https://crates.io/crates/tf2_r2r) [![docs](https://docs.rs/tf2_r2r/badge.svg)](https://docs.rs/tf2_r2r)

This project is copied from [arjo129 rustros_tf](https://github.com/arjo129/rustros_tf),[MaxiMaerz rustros_tf](https://github.com/MaxiMaerz/rustros_tf), [smilerobotics tf_rosrust](https://github.com/smilerobotics/tf_rosrust).

This is a rust port of the [ROS tf library](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html). It is intended for being used in robots to help keep track of multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.


## Features
So far the only the following have been implemented:
* `TfListener` with `lookup_transform` and time traversal.
* `TfBroadcaster` to publish `/tf`

I am still working on the following:
* More efficient cache data structure.
* Weed out `unwrap()`s

## Supported platforms
Currently only Ubuntu 22.04 running ROS2 Humble on x86_64 is tested. It should work on any linux based system with a proper ROS2 installation.

## Getting Started
Install [ROS2](https://docs.ros.org/en/humble/Installation.html) first. On ubuntu, this can be done like so:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install ros-humble-desktop
```

After installing ROS2, you may simply add this crate as a dependency to your cargo project:

```toml
[dependencies]
tf2_r2r = "0.1.0"
```
