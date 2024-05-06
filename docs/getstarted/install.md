# Installation

BunnyVisionPro is designed as a distributed system to facilitate robot teleoperation. The system consists of a server
and a client that communicate via data streaming, leveraging the Apple Vision Pro as hand pose tracking device. To
interact with Vision Pro, download the [Tracking Streamer](https://apps.apple.com/us/app/tracking-streamer/id6478969032)
app on your device.

The server can be installed and run inside a Docker container, while the client is super
light-weight can be installed using pip.

## Server Installation

Before running the server code inside a Docker container, you need to either pull an existing Docker image or build one
yourself.

### Pulling an Existing Docker Image

For quick setup, pull an existing Docker image from Docker Hub using the following command:

```bash
docker pull yzqin/bunny_teleop_server
```

### Building a Docker Image from a Dockerfile (Optional)

If you need a custom setup, you can also build your own Docker image using the provided Dockerfile from
our [Bunny Teleop Server Codebase](https://github.com/Dingry/bunny_teleop_server).

```bash
git clone https://github.com/Dingry/bunny_teleop_server.git
cd bunny_teleop_server/docker/minimal

docker build -t bunny_teleop_server .
```

## Client Installation

The client can be easily installed using Python's pip package manager:

```bash
pip install bunny_teleop
```
