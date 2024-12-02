import matplotlib.pyplot as plt
import numpy as np

# Generate time data for 24 hours
hours = np.linspace(0, 24, 100)

# Simulating a more realistic summer load curve with random fluctuations
summer_load = np.piecewise(hours, [hours < 6, (hours >= 6) & (hours < 12), (hours >= 12) & (hours < 18), hours >= 18],
                           [0.3, lambda hours: 0.4 + 0.5*(hours-6)/6 + 0.05*np.random.randn(len(hours[(hours >= 6) & (hours < 12)])),
                            lambda hours: 0.9 + 0.1*np.random.randn(len(hours[(hours >= 12) & (hours < 18)])),
                            lambda hours: 0.9 - 0.5*(hours-18)/6 + 0.05*np.random.randn(len(hours[hours >= 18]))])

# Simulating a more realistic winter load curve with random fluctuations
winter_load = np.piecewise(hours, [hours < 6, (hours >= 6) & (hours < 12), (hours >= 12) & (hours < 18), hours >= 18],
                           [0.1, lambda hours: 0.15 + 0.2*(hours-6)/6 + 0.02*np.random.randn(len(hours[(hours >= 6) & (hours < 12)])),
                            lambda hours: 0.35 + 0.05*np.random.randn(len(hours[(hours >= 12) & (hours < 18)])),
                            lambda hours: 0.35 - 0.2*(hours-18)/6 + 0.03*np.random.randn(len(hours[hours >= 18]))])

# To fix the issue with Chinese font display, we need to use a font that supports Chinese characters
plt.rcParams['font.sans-serif'] = ['SimHei']  # Use SimHei font to display Chinese characters
plt.rcParams['axes.unicode_minus'] = False  # Ensure minus signs appear correctly

# Create the plot with the correct font
plt.figure(figsize=(10, 6))
plt.plot(hours, summer_load, label='夏季负荷', color='orange', linewidth=2)
plt.plot(hours, winter_load, label='冬季负荷', color='blue', linewidth=2, linestyle='--')

# Add labels and title in Chinese
plt.title('新疆地区中央空调典型负荷曲线', fontsize=14)
plt.xlabel('一天中的时间（小时）', fontsize=12)
plt.ylabel('负荷（相对单位）', fontsize=12)
plt.grid(True)
plt.legend()

# Show the plot
plt.show()
