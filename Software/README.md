# Software
This folder contains the full ROS 2 workspace and supporting scripts for the main computing unit (Jetson) of the autonomous robot. It includes ROS 2 packages for navigation, sensor data collection, and robot localization, along with configuration files, and systemd services to automate startup. The workspace is organized for use inside a Docker environment for portability and ease of deployment. First You have to install Docker Engine, then you can easily build and run the project.

## Docker: Build & Run
To launch the software program, navigate to the Software directory and execute the following command:
```bash
sudo docker compose build && sudo docker compose up
```
This will build all required images and start the ROS 2 system along with its dependencies.

Click the thumbnail below to watch the demo video:
[![Watch the video](../Documents/readme_resource/video_thumbnail2.png)](https://youtu.be/bKZ6i2iMwEY?si=wWfmDxZEH7T19WWM)

## Warning

> **Warning:**
> If you have a older docker composer version(Compose v1) use **docker-compose** command 
```bash
sudo docker-compose build && sudo docker-compose up
```

<br>

> **Warning:**
> Ensure you are using Docker Engine directly for compatibility and performance reasons. If you use Docker Desktop you might find issues with serial communication. 

eg :-
```
error    | TermiosAgentLinux.cpp | init                     | open device error      | device: /dev/ttyUSB0, errno: 21
Error while starting serial agent!
```
```
Serial port not found. | device: /dev/ttyUSB0, error 2, waiting for connection...
```

> But if you able to solve that issue using Docker Desktop, please let us know by updating this README.md file.