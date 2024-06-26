> LDROBOT Sensor Team, generate: 2024-06-26,01:02
## 1. 系统设置 
- 通过CP2102模块连接激光雷达,然后将CP2102模块的USB端接入你的Linux主机,然后给设备-x权限 
- 以/dev/ttyUSB0为例,($ sudo chmod 777 /dev/ttyUSB0)  
 
## 2. 编译方法 
 
```bash
$ mkdir build
$ cd build
$ cmake ..
$ make
``` 
## 3. 运行方法 
```bash 
$ cd build
$ ./ldlidar_ssl <product_type> <port_name>
# SSL-20L eg:
$ ./ldlidar_ssl SSL20L /dev/ttyUSB0
# SSL-20N eg:
$ ./ldlidar_ssl SSL20N /dev/ttyUSB0
# SSL-20P eg:
$ ./ldlidar_ssl SSL20N /dev/ttyUSB0
``` 
## 4.激光雷达驱动库介绍 
- 静态库libldlidar_driver_ssl.a为release模式组建 
