# Esphil

使用esp32-c2作为本次项目的主控

该芯片为RISCV 32 位单核处理器，四级流水线架构，主频高达120MHz

576 KBROM 272 KBSRAM（其中16KB专用于cache）

14×GPIO口

此设备有一定的装饰作用，带一点的赛博风格

带有一个电源led，一个状态led，一个1.14寸的彩屏（这里有点大材小用了）

**eg:**

![](/img/IMG_1241.JPG)

由上图可知，字库使用的是极具个性的草书字体

显示的label分两部分，一为句子主体，二为句子出处排布如上

使用smartconfig来配置WiFi，句子api有三种（详细见代码）

有很多懒得介绍了，详细看代码，写了少量注释，代码写得很烂:poop:

### 演示

<video width="640" height="360" controls>
<source src="/img/IMG_1232.mp4" type="video/mp4">
</video>

**Bug:** :warning:

因为lvgl与WiFi任务的冲突，第二次按下时会导致整个设备重启

不想修了，欢迎提交Issue，谢谢

喜欢就给个星吧:happy:

