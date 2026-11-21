# ESP32-S3 车轮动平衡仪

0.6.7 单平面动平衡原型。通过轮上的白标获取转速和相位，用 MPU6050 的振动数据比较原轮、试重和校正后状态。手机直连设备热点，可看三轴波形、同速 1× 振动对比和相对白标的贴块位置。会话报告支持 JSON，原始记录支持 CSV。

***本项目是一套支持装车测量的自行车，电动车，摩托车车轮单平面动平衡仪。车轮架空后即可手拨，空转测量，减少拆装车轮和依靠经验反复试贴配重的操作。
系统以光电白标作为每圈的角度基准，从松手后的自然滑行记录中筛选有效数据，并按实际转速匹配原轮与试重记录。即使两次转动的力度和起始转速不同，只要有足够的共同转速数据，就能比较同速下的振动幅值和相位变化。
系统通过单平面影响系数法估算校正所需的克数与相对白标的贴装角度，并在手机页面上用圆盘图标出位置。贴块后可在共同转速范围内复测，直观看到校正前后的振动变化。

目前测试对象涵盖数百到数万的自行车，都有比较好的效果，感兴趣的话可以参考一下。后续将加入更多电动车，摩托车的测量数据，优化算法。从初始的0.1版本到目前0.6.7，制作不易！恳请大家多多留言发表意见！

<img width="1080" height="1920" alt="12e146658cf4015bb8720b7b47b8d8af" src="https://github.com/user-attachments/assets/c428e531-c881-4327-bc98-3da2cbd79802" /><img width="2276" height="1280" alt="1cc7cb42087e455193bdf794764fa773" src="https://github.com/user-attachments/assets/ce0baaa5-3f3b-41e3-ad45-43975899e4dc" />
<img width="5712" height="3213" alt="831779fd9783670d1112c6375b3567ee" src="https://github.com/user-attachments/assets/579dde4e-ca3c-4cc1-bb65-f790d3b5db20" />


<img width="2275" height="1279" alt="c0606c3d52fc0d941029b9b64c55a6f9" src="https://github.com/user-attachments/assets/908177d0-1d81-45f0-b4a6-8dbe59f4be8e" />
<img width="2276" height="1280" alt="0fba63788e1c64c6ed0bdca59fed8a3c" src="https://github.com/user-attachments/assets/7b165a7b-076b-47fd-a41d-3fe869834490" />



## 接线

开发板为 Seeed XIAO ESP32-S3 Sense（8 MB Flash、8 MB PSRAM）。MPU6050 和 TCRT5000 均使用 3.3 V 供电并和开发板共地。

| 模块 | 信号 | XIAO |
| --- | --- | --- |
| MPU6050 | SDA | D4 / GPIO5 |
| MPU6050 | SCL | D5 / GPIO6 |
| MPU6050 | AD0 | GND |
| TCRT5000 | DO | D0 / GPIO1 |

固件通过 I²C 轮询读取 MPU6050 FIFO，INT 可留空。TCRT5000 的 AO 留空，白标对应 DO 低电平。

## 构建和使用

安装 ESP-IDF 5.5.5 后，在 ESP-IDF 终端进入以下目录。首次刷机前，在菜单的 **Phone diagnostic web server** 中设置自己的热点名称和密码。仓库提供的演示配置为 **WheelBalance-S3** 和 **wheelbalance-demo**。

~~~sh
cd firmware
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
~~~

把 PORT 替换为开发板的串口。烧录后手机连接设备热点，打开 http://192.168.4.1。依次测量原轮和已知试重，按页面图示查看相对白标的角度，再做校正后复测。输入的试重和校正半径为轮轴中心到贴块中心的距离，单位 mm。

试重仅用于车轮可靠架空、贴块牢靠的受控低速测试。固件会在数据不足或候选配重分歧时保留结果报告。实轮克数重复性和配重精度仍待验证。

## 软件验证

安装 Python 和 Zig，把 zig 加入 PATH，在仓库根目录运行。

~~~sh
python tests/balance_core/run_tests.py --sanitize
python tests/product_session/run_tests.py --assessment --sanitize
python tests/product_session/run_tests.py --retest --sanitize
python tests/product_session/test_window_end_to_end.py
~~~

这些测试覆盖复数影响系数、候选一致性、复测状态和合成 FIFO 数据到配重结果的端到端流程。

## 外壳

hardware 目录含 TCRT5000 和 MPU6050 的 OpenSCAD 外壳模型。打印前按实际模块尺寸检查装配。
