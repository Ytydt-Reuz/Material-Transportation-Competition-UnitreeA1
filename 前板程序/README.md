# area_detect.py

部署在前板，前板性能较好，包含机器狗视觉识别相关文件，负责任务区域及障碍物图像识别处理等相关功能的实现。

## 配置及环境：

* A1机器狗

* RealSense 深度摄像头  D435i 

* [OpenCV](https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/)

* [Socket通信模块](https://github.com/socketio/socket.io-client/tree/master/dist)

* [Yolov5](https://github.com/bubbliiiing/yolov5-v6.1-pytorch)

* torch==1.2.0

* CUDA 10.0

  

  ### 环境安装

  ```bash
  # CUDA 10.0 版本下
  pip install torch===1.2.0 torchvision===0.4.0 -f https://download.pytorch.org/whl/torch_stable.html
  ```

  

## 运行方法：

### 连接

将A1狗开机，电脑连接到A1的WIFI，输入密码进行登录。

通过终端对前板进行SSH连接 ：`ssh unitree@193.168.123.12` 

输入A1狗的密码，即可将电脑连接到A1狗前板。

### 运行程序

进入文件目录	`cd cv_robocom`

运行程序	`python area_detect.py`

## 实现原理：

利用已经训练好的Yolov5权重，和Yolov5的目标检测方法，对运输过程中的各类任务区域进行识别。

## 识别效果：

针对于本次识别用例，本次使用的数据集包含四种类型的标签。

```python
area   	 	
stairs  	
obs     	
begin   
```

如下图所示，由左上到右下分别对应四种标签类型的识别效果，置信度均在0.9以上。

![avatar](PIC\image-20221101133153313.png)



当area_detect.py 识别到任务目标之后，将通过Socket，将深度信息传递到后板，进行对应的运动控制操作，如：避障、倾倒物品、结束运动、上楼梯。

![avatar](PIC\image-20221101132433454.png)

```
mid_pos = [(result[1] + result[3])//2, (result[2] + result[4])//2] *#确定索引深度的中心像素位置*

  min_val = min(abs(result[3] - result[1]), abs(result[4] - result[2])) *#确定深度搜索范围*

return dist
```

