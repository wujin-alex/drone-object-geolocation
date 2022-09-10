# 无人机目标定位库

关于原理说明请参考：[基于无人机吊舱单目相机的目标定位算法](基于无人机吊舱单目相机的目标定位算法.md)。

## 编译

通过如下命令进行编译：

```shell
$ cd drone_objlocation
$ mkdir build && cd build
$ cmake ..
$ make
```

编译后会生成库文件**libDroneObjlocation.so**。

## 安装

编译后进行安装

```shell
$ sudo make install
```

安装内容如下：

```shell
Install the project...
-- Install configuration: ""
-- Up-to-date: /usr/local/lib/libDroneObjlocation.so
-- Up-to-date: /usr/local/include/geolocation
-- Installing: /usr/local/include/geolocation/drone_objlocation.h
-- Up-to-date: /usr/local/lib/cmake/DroneObjlocation/DroneObjlocationTargets.cmake
-- Installing: /usr/local/lib/cmake/DroneObjlocation/DroneObjlocationTargets-noconfig.cmake
-- Up-to-date: /usr/local/lib/cmake/DroneObjlocation/DroneObjlocationConfig.cmake
-- Up-to-date: /usr/local/lib/cmake/DroneObjlocation/DroneObjlocationConfigVersion.cmake
```

## Demo

本项目在`/demo`提供了一个简单的使用样例，可以进行测试：

```shell
$ cd demo
$ mkdir build && cd build
$ cmake ..
$ make
```

运行

```shell
$ ./demo_geolocation
```

