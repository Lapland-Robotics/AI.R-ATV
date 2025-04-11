# SystemCTL service
If you want the softwares to execute automatically on the robot after the main brain switch on, you can use the Linux systemctl approach. 

# useful commands in Linux systemctl

``` bash
sudo cp robot.service /etc/systemd/system/robot.service
```

reload the daemon after you change the code in .service file
``` bash
sudo systemctl daemon-reload
```

start the service manually
``` bash
sudo systemctl start robot.service
``` 

restart the service manually
``` bash
sudo systemctl restart robot.service
``` 

stop the service manually
``` bash
sudo systemctl stop robot.service
``` 

view the status of the service
``` bash
sudo systemctl status robot.service
```

view the log of the service
``` bash
sudo journalctl -u robot.service
```

To start a service at boot, use the enable
``` bash
sudo systemctl enable robot.service
``` 
