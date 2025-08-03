import numpy as np
import matplotlib.pyplot as plt

# 1) 构造示例数据 -----------------------------------------------------------
# 如果你的数据已经在文件里，可以用 np.loadtxt / pandas.read_csv 读进来
# 这里随便造 100 行做演示
data = np.loadtxt('dt.txt')   # 默认按空格/制表符分隔
# 把你要展示的那一行替换成
# data[42] = [0.000269175, 0.000269175]

# 2) 画图 -------------------------------------------------------------------
plt.figure(figsize=(5,3))
plt.plot(data[:,0], label='column 1')   # 第一列
plt.plot(data[:,1], label='column 2')   # 第二列
plt.xlabel('Index')
plt.ylabel('Value')
plt.title('Two-column data')
plt.legend()
plt.tight_layout()
plt.show()
