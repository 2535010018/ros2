# 地面小车横向路径跟踪控制器对比评测

纯 Python 运动学仿真，不依赖 ROS / Gazebo / CARLA。以独轮（阿克曼等效）模型为被控对象，小车以恒定纵向速度沿一条双移线参考路径行驶，在**同一参考路径、同一初始位姿**下依次接入 P 比例、PID、Stanley 三种横向控制器，分别在「无扰动」和「定位噪声 + 侧风偏航扰动」两组条件下完成跟随，统计最大横向误差、横向误差均方根、收敛时间、最大控制量四个指标，量化不同控制器在理想工况与扰动工况下的路径跟随能力。

配套教程文档：`docs/ground/lateral_control_benchmark.md`

## 运行环境

- 操作系统：Windows 10 / 11（Linux 亦可，脚本为纯 Python 后台出图，无需图形界面）
- Python：3.10 ~ 3.12
- 依赖：见 `requirements.txt`（`numpy`、`matplotlib`、`Pillow`）

## 运行步骤

1. 创建并激活虚拟环境，安装依赖：

   ```bash
   python -m venv ctrl_env
   # Windows: ctrl_env\Scripts\activate
   # Linux:  source ctrl_env/bin/activate
   pip install -r requirements.txt
   ```

2. 在本目录下运行：

   ```bash
   python main.py
   ```

   脚本会依次跑 3 种控制器 × 2 组条件共 6 段仿真，每段结束后把轨迹、误差曲线和指标写入结果目录；clean 工况下再逐帧合成跟随演示动图。

## 输出结果

运行结束后在脚本同级的 `results/` 目录下得到：

| 文件 | 说明 |
| --- | --- |
| `lateral_control_trajectory_clean.png` / `lateral_control_trajectory_disturbed.png` | 两组条件下参考路径与各控制器实际轨迹俯视图 |
| `lateral_control_error_clean.png` / `lateral_control_error_disturbed.png` | 两组条件下横向误差随时间曲线 |
| `lateral_control_metrics.png` | RMSE 与最大横向误差分组对比柱状图 |
| `metrics.csv` | 四指标数据表（控制器 × 工况） |
| `lateral_control_demo.gif` | clean 工况下三控制器沿双移线跟随的演示动图 |

> 输出图片的命名与教程图片目录 `docs/img/ground/` 中的文件名一致，提交文档时将 `results/` 下的图片直接复制过去即可，无需再改名。

## 注意事项

- 仿真使用固定随机种子（`SEED`），定位噪声序列可复现；修改种子会改变扰动工况下的具体数值，但不改变控制器之间的相对优劣。
- 控制器输出角速度做了限幅（±1.2 rad/s），Stanley 内部前轮转角限幅 ±0.6 rad，避免单步控制量过大导致模型发散。
- 本实验为运动学层面的横向控制对比，未引入车辆纵向动力学、轮胎侧偏和执行机构延迟，指标结论用于控制器选型的初步参考。
