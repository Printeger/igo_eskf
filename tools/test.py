import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

# 假设采集到的传感器输出数据
data = np.array([1.01, 1.02, 0.99, 1.03, 1.00, 1.01, 1.02, 0.98, 1.00, 1.01])

# 计算均值和标准差
mean = np.mean(data)
std_dev = np.std(data)

print(f"噪声均值: {mean}")
print(f"噪声标准差: {std_dev}")

# 绘制直方图和高斯分布拟合曲线
plt.hist(data, bins=10, density=True, alpha=0.6, color='g')

# 拟合高斯分布
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mean, std_dev)
plt.plot(x, p, 'k', linewidth=2)
title = "Fit results: mean = %.2f,  std_dev = %.2f" % (mean, std_dev)
plt.title(title)

plt.show()