bring up docker + d415 commands once [Jetson setup](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) is finished:

```
sudo apt update && sudo apt upgrade
sudo apt install docker-compose
git clone https://github.com/JetsonHacksNano/installLibrealsense
cd installLibrealsense
./installLibrealsense.sh
docker build -t nano-jazzy -f Dockerfile.jazzy .
docker compose up -d
```
