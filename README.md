# AI.R-ATV
This is ROS2-based software program designed to control ATVs.

## Build the docker image
docker build -t atv_docker .

## Run the docker image
docker run -p 8888:8888/udp -it --rm atv_docker