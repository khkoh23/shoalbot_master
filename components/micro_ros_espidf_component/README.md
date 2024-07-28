# micro-ROS component for ESP-IDF

This component is available from https://github.com/micro-ROS/micro_ros_espidf_component/tree/humble


## Rebuild

At the component directory: 
```bash
source ~/esp/<V5.2.2/esp-idf>/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
```

At the project directory: (e.g. ~/esp/shoalbot_master/)
To clean and rebuild all the micro-ROS library:
```bash
idf.py clean-microros
```

The following will download required library and compile the libmicroros.a
```bash
idf.py set-target esp32s3
```



## Usage

At the project directory: (e.g. ~/esp/shoalbot_serial/)

```bash
idf.py menuconfig
idf.py build
idf.py flash
idf.py monitor
```

Or perform the similar using vscode esp-idf extension.