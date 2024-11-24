# ATV service
If you want the softwares to execute automatically on the ATV after the main brain switch on, you can use the Linux systemctl approach. 

# useful commands in Linux systemctl

``` bash
sudo cp atv.service /etc/systemd/system/atv.service
```

reload the daemon after you change the code in .service file
``` bash
sudo systemctl daemon-reload
```

start the service manually
``` bash
sudo systemctl start atv.service
``` 

stop the service manually
``` bash
sudo systemctl stop atv.service
``` 

view the status of the service
``` bash
sudo systemctl status atv.service
```

view the log of the service
``` bash
sudo journalctl -u atv.service
```

To start a service at boot, use the enable
``` bash
sudo systemctl enable atv.service
``` 
