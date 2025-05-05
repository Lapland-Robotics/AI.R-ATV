# Main Brain - Jetson
High-level functionality of the Robot. Running this part of the project is peace of cake. You just have to install Docker Engine and run two docker command to make this work!!!

## Build & Run

```bash
sudo docker compose build && sudo docker compose up
```

## Warning

> **Warning:**
> If you have a older docker composer version(Compose v1) use **docker-compose** command 
```bash
sudo docker-compose build && sudo docker-compose up
```

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