# ESP32-S3 车轮动平衡仪

0.6.7 单平面动平衡原型。通过轮上的白标获取转速和相位，用 MPU6050 的振动数据比较原轮、试重和校正后状态。手机直连设备热点，可看三轴波形、同速 1× 振动对比和相对白标的贴块位置。会话报告支持 JSON，原始记录支持 CSV。

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
