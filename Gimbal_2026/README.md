用于存放沧海战队步兵电控代码

这里是云台代码



云台控制逻辑主要是分两类

遥控和鼠标

遥控：

&#x09;遥控的右手两个通道控制抬头低头，向左向右。

&#x09;遥控左手拨杆最上是全部失能，中间是遥控控制，最下是键鼠控制（需要连接客户端）

&#x09;遥控右手拨杆最上是开启摩擦轮，拨弹正转，中间是发射失能，最下是开启摩擦轮拨弹反转



鼠标：
和FPS游戏一样，左键单点单发，长按连发，右键开启/关闭自瞄



这个项目名称就是最早建立的后来懒得新建都是复制下来的懒得改名了，原本只想试试东哥那块C板

主要代码放在E:\\Git\\robomaster\\CangHai\\RoboMaster\_Electrical\\Gimbal\\User

E:\\Git\\robomaster\\CangHai\\RoboMaster\_Electrical\\Gimbal\\User\\bsp

是Bsp库有些没用上，是直接移植官套的，很多用不上

E:\\Git\\robomaster\\CangHai\\RoboMaster\_Electrical\\Gimbal\\User\\control

控制函数都在里，有状态机、云台、发射机构等

E:\\Git\\robomaster\\CangHai\\RoboMaster\_Electrical\\Gimbal\\User\\devices

板载的陀螺仪和磁力计在这里，今年没咋用你们可以试试好不好用

E:\\Git\\robomaster\\CangHai\\RoboMaster\_Electrical\\Gimbal\\application

这里有封装好的CAN的发送电机电流和接收电机数据，便于后续控制电机

还有接收遥控数据，和转发出去的

还有维特陀螺仪接收数据

以及一个usb发送板载陀螺仪的数据





底盘小陀螺时，云台Yaw轴会有一点点不稳，尤其是开启或关闭时。可以考虑加一个慢启动，慢停止。还有切换小陀螺方向

