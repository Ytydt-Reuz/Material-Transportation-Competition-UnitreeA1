# example_walk.cpp

部署在后板，后板包含机器狗的运动相关控制模块，通过接受前板信息，执行预定的机器控制，实现机器狗任务过程中的整体运动及调控。

## 配置及环境：

* [Boost](http://www.boost.org) (version 1.5.4 or higher)

* [CMake](http://www.cmake.org) (version 2.8.3 or higher)

* [LCM](https://lcm-proj.github.io) (version 1.4.0 or higher)

* [Socket通信模块](https://github.com/socketio/socket.io-client/tree/master/dist)

* A1机器狗

  

  ### 环境安装

  ```bash
  cd lcm-x.x.x
  mkdir build
  cd build
  cmake ../
  make
  sudo make install
  ```

  

## 运行方法：

### 连接

将A1狗开机，电脑连接到A1的WIFI，输入密码进行登录。

通过终端对前板进行SSH连接 ：`ssh unitree@193.168.123.12` 

输入A1狗的密码，即可将电脑连接到A1狗前板。

### 运行程序



cd robocom2022/examples 

选择控制A1狗的文件 ls example_walk.cpp

返回上一层目录 cd.. 

进入build文件夹下 cd build

对传输完成的文件执行cmake

运行生成的可执行程序。 make -js

执行此指令之后，机器狗将自动识别，输出对应信息，完成物资运输的任务

在机器狗过程中，如果想停止任务可以用Ctrl+C
