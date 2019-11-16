# APA_QT_Assistant


## 概述
项目主要验证APA的功能算法，主要涉及车辆信息的解码与车辆控制、超声波的车位检测和障碍物定位、泊车轨迹规划等。

##  软件功能说明
如下图所示，是目前软件界面的功能划分UI,从图中可以看出，目前主要分为控制、检测和路径三个功能模块。
<img src="https://raw.githubusercontent.com/zgh551/FigureBed/master/img/UI.gif" />

## 感知算法
### 超声波重定位
如下图所示，根据超声波采集的库位两边数据，重新计算库位信息。
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/GIF.gif)
### 轨迹规划算法