# FE_RMUL2027_Navigation

## 基于COD_RMUL2026_Navigation项目进行修改

## 对于通信部分：

```bash
对于达妙H7下位机，电控采用921600波特率，增加了该波特率实现与下位机通信
增加了通信报错
```

### 串口通信协议：

```bash
byte 0       : 帧头 0xA5
byte 1~4     : linear.x
byte 5~8     : linear.y
byte 9~12    : linear.z
byte 13~14   : checksum
```

## 对于适应RMUL27赛季增加过洞的修改：

### 对于雷达点云过滤器的修改，原本过滤器只是把车体周围的点云过滤掉，对于过洞需求，我的想法是直接把一定高度的点云全部过滤掉，把过洞路段当作一个普通路段去处理，先只保留一定高度区间的点云，然后再去除车体的点云。

```nestedtext
在 base_link 坐标系中删除 z 高于 max_height 的点，并保留其余点；
随后再使用 CropBox 删除车身区域点云。
过滤节点会先将 Livox 点云从 `livox_frame` 转换到 `base_link`，在 `base_link` 坐标系中删除高于 `max_height` 的点，再删除车身区域点云，最后将结果转换回 `livox_frame` 发布。

```

```nestedtext
`small_point_lio` 当前使用原始 `/livox/lidar`；
`pointcloud_to_laserscan` 当前应使用 `/livox/lidar_filtered`，请通过运行时确认；
Nav2 local/global costmap 使用 `/livox/lidar_filtered`；

```

## 已知问题与环境要求

历史版本曾因缺少 STVL 包导致 Nav2 代价地图无法正常加载，进而影响避障和导航。

当前已完成 STVL 环境配置。local/global costmap 使用：


```text
/livox/lidar_filtered
```

## 基于 map 坐标触发过洞云台收缩

过洞时可根据机器人在 `map` 坐标系中的位置判断是否进入过洞区域，并向下位机发送云台收缩标志位。

### 坐标与定位来源

`small_point_lio` 使用原始 `/livox/lidar` 和 IMU 进行激光惯性里程计，发布 `/Odometry`，其 `header.frame_id` 为 `odom`、`child_frame_id` 为 `base_link`，同时发布 `odom -> base_link` TF。区域判断使用机器人底盘 `base_link` 在 `map` 坐标系中的位置，不直接使用雷达坐标：通过 TF 查询 `map -> base_link`，再读取位置的 `x`、`y` 坐标。

过洞区域坐标以地图原点为参考，例如：

```yaml
passage:
  x_min: 10.0
  x_max: 12.0
  y_min: 3.0
  y_max: 5.0
```

当 `x_min <= x <= x_max` 且 `y_min <= y <= y_max` 时，机器人处于过洞区域。实际区域边界应根据已保存地图和现场位置标定。

### 云台收缩触发

建议增加独立的过洞区域控制节点，订阅 `/Odometry` 或查询 TF，将 `base_link` 位姿转换到 `map` 后判断区域：

```text
TF(map -> base_link) -> XY 区域判断 -> gimbal_shrink 标志位 -> 串口 -> 下位机
```

进入区域时发送 `gimbal_shrink = 1`，通知下位机收缩云台；离开区域时发送 `gimbal_shrink = 0`，通知下位机恢复云台。区域边界应增加迟滞或连续多帧确认，避免定位噪声导致标志位反复切换。

云台控制建议使用独立串口控制帧，不直接修改现有底盘速度帧，以免影响下位机对速度数据包的解析。例如：

```text
byte 0: 0xB6       # 云台控制帧帧头
byte 1: 0x01/0x00   # 1：收缩，0：恢复
byte 2~3: checksum
```

该功能目前是设计方案，尚未实现区域判断节点、云台控制话题和对应的下位机控制帧。实现时还应加入位姿/TF 超时保护；失去定位时应进入下位机约定的安全状态。
