# 正式绘制符合电采暖负荷MPC调度的系统图
# 参考你的MPC示意图结构，但内容和符号换成你的论文要求

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体为黑体
plt.rcParams['axes.unicode_minus'] = False    # 正常显示负号

# 绘制图4.1 电采暖柔性负荷调度模型总体架构（标准版）

import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots(figsize=(14, 8))

# 知识图谱推理模块（第三章）
ax.text(0.4, 0.9, '用户知识图谱推理模块\n(第三章可调节能力 I1~I4)', ha='center', va='center', fontsize=10, bbox=dict(boxstyle="round", fc="lightyellow"))

# 上层贝叶斯优化模块
ax.text(0.4, 0.7, '上层优化器模块\n(贝叶斯优化参考轨迹)', ha='center', va='center', fontsize=10, bbox=dict(boxstyle="round", fc="lightblue"))

# 下层MPC控制模块
ax.text(0.4, 0.5, '下层滚动控制器模块\n(MPC调度控制)', ha='center', va='center', fontsize=10, bbox=dict(boxstyle="round", fc="lightskyblue"))

# 电采暖实际响应模块
ax.text(0.4, 0.3, '电采暖负荷响应模块\n(电锅炉/空调/热泵设备)', ha='center', va='center', fontsize=10, bbox=dict(boxstyle="round", fc="lightgreen"))

# 外部扰动模块
ax.text(0.75, 0.5, '外部扰动输入\n(外界气温 T_out(k))', ha='center', va='center', fontsize=10, bbox=dict(boxstyle="round", fc="mistyrose"))

# 箭头
ax.annotate('', xy=(0.4, 0.84), xytext=(0.4, 0.76), arrowprops=dict(arrowstyle='->'))
ax.annotate('', xy=(0.4, 0.64), xytext=(0.4, 0.56), arrowprops=dict(arrowstyle='->'))
ax.annotate('', xy=(0.4, 0.44), xytext=(0.4, 0.36), arrowprops=dict(arrowstyle='->'))

# 外部扰动箭头到设备
ax.annotate('', xy=(0.7, 0.5), xytext=(0.55, 0.5), arrowprops=dict(arrowstyle='->'))

# 反馈箭头（响应到MPC）
ax.annotate('', xy=(0.4, 0.32), xytext=(0.4, 0.48), arrowprops=dict(arrowstyle='->', linestyle='dashed'))
ax.text(0.42, 0.4, '状态反馈\n(温度 T(k), 功率 P(k))', ha='left', va='center', fontsize=9)

# 关闭坐标轴
ax.axis('off')

plt.title('图4.1 电采暖柔性负荷调度模型总体架构', fontsize=14)
plt.show()

