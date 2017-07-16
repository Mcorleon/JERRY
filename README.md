﻿#NCU第十二届智能车电磁双车
**JERRY**


---

  **1. 硬件电路**
  
[主板+运放+驱动][1]


  *PS：lmv358运放,mos管+hip4082驱动,通信采用nrf2401,测距用freecar的鸳鸯*
  
 **2. 机械结构**
  采用硬连接，前瞻45cm，3水平电感在前3垂直电感在后，前轮前束，内倾。超声波和标志位均是前车发送后车接收。
 ![此处输入图片的描述][2]
  
  **3.基本循迹原理**
  3个水平电感：取赛道中央磁场最大点的AD值为基准，中间电感越小偏差越大，左右电感用于判断方向,往AD值大的方向走，偏差绝对值大于一定值时锁定方向。3个竖直电感进只辅助判断圆环和坡道。


  [1]: https://github.com/Mcorleon/SmartCar_Board
  [2]: http://c.hiphotos.baidu.com/image/pic/item/03087bf40ad162d9f84c46861bdfa9ec8b13cd8d.jpg