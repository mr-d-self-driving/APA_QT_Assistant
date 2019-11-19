# APA_QT_Assistant


## 概述
项目主要验证APA的功能算法，主要涉及车辆信息的解码与车辆控制、超声波的车位检测和障碍物定位、泊车轨迹规划等。

##  软件功能说明
如下图所示，是目前软件界面的功能划分。从图中可以看出，目前主要分为控制算法、感知检测和轨迹规划三个功能模块。
<img src="https://raw.githubusercontent.com/zgh551/FigureBed/master/img/UI.gif" />

## 感知算法
### 超声波重定位
#### 垂直泊车入库过程中的重定位
如下图所示，是垂直泊车入库过程。重定位算法在此过程中采集数据点，当车辆停止后根据采集的数据计算库位的外边沿角点，并拟合前车斜率信息。

![车辆入库过程描述](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/ParkingEnterEdgeDetect-%E7%AC%AC%201%20%E9%A1%B5.png)

如下图所示，根据实际采集的超声数据进行库位角点的重定位，拟合前车边沿数据，进行前车斜率信息的计算。

![实际数据采集和计算](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/%E6%B3%8A%E8%BD%A6%E5%85%A5%E5%BA%93%E8%BF%87%E7%A8%8B%E5%AE%9A%E4%BD%8D.gif)

#### 垂直车位进库重定位
如下图所示，是垂直泊车进库过程。重定位算法在此过程中采集数据点，当车辆停止后根据采集的边沿数据，计算两边车辆的斜率信息。

![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/ParkingEnterEdgeDetect-%E5%9E%82%E7%9B%B4%E8%BF%9B%E5%BA%93%E5%AE%9A%E4%BD%8D.png)

如下图所示，根据实际采集的超声波库位两边数据，重新计算库位信息。
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/GIF.gif)

### 环视定位

## 轨迹规划算法
### 平行规划
#### 一次入库情况
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/%E5%B9%B3%E8%A1%8C%E4%B8%80%E6%AC%A1%E5%85%A5%E5%BA%93.gif)
#### 多次尝试入库情况
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/%E5%B9%B3%E8%A1%8C%E5%A4%9A%E6%AC%A1%E5%B0%9D%E8%AF%95%E5%85%A5%E5%BA%93.gif)
### 垂直规划
#### 一次入库情况
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/%E5%9E%82%E7%9B%B4%E4%B8%80%E6%AC%A1%E5%85%A5%E5%BA%93.gif)
#### 多次尝试入库情况
![](https://raw.githubusercontent.com/zgh551/FigureBed/master/img/%E5%9E%82%E7%9B%B4%E5%A4%9A%E6%AC%A1%E5%B0%9D%E8%AF%95%E5%85%A5%E5%BA%93.gif)