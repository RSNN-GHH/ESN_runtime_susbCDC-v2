import numpy as np
import matplotlib.pyplot as plt
from spikeActivation import spikeActivation




# ESN参数设置
train_length = 200
test_length = 200
total_length = train_length + test_length

input_size = 1
reservoir_size = 100
output_size = 1
spectral_radius = 0.95
leak_rate = 1.0
sparsity = 0.2

# spikeActivation 设置
# frame_size16 = reservoir_size  # 保持与 reservoir_size 一致
frame_size16 = 1  # 保持与 reservoir_size 一致
count_win_us = 1000
scale = 1.4
max_pulse_count = 6155.0       #脉冲计数最大值，根据count_win_us调整


activate_inputs = []  # 用于调试的输入记录

# 自定义激活函数
# def activate(x):
#     activate_inputs.append(x*scale+2)  # 记录激活函数的输入
#     x = (x*scale+2) / 7.5 * 65535
#     x = np.clip(x, 0, 65535).astype(np.uint16)
#     x_act = sA.activate(x) / max_pulse_count * 7.5 / 10
#     return x_act

def encode(x):
    activate_inputs.append(x*scale+2)  # 记录激活函数的输入
    x = (x*scale+2.5) / 5 * 65535
    x = np.clip(x, 0, 65535).astype(np.uint16)
    x_act = sA.activate(x) / max_pulse_count * 5 / 10
    return x_act


def activate(x):
    x = x / 5 * scale
    activate_inputs.append(x)
    return np.tanh(x) * 5 / scale
    # return x


# 计算 NRMSE
def compute_nrmse(true, pred):
    error = np.sqrt(np.mean((true - pred) ** 2))
    norm = np.std(true)
    return error / norm

sA = spikeActivation(frame_size16, count_win_us)
sA.init()

# 读取数据
data = np.loadtxt('periodic_waveform_sequence.txt')
data = data[:total_length]
u = data[:, 1].reshape(-1, 1)
y = data[:, 2].reshape(-1, 1)
# 对输入进行编码
for i in range(u.shape[0]):
    u[i] = encode(u[i])
np.savetxt('encoded_input.txt', u)


# 初始化权重
np.random.seed(42)
Win = np.random.uniform(0, 1, (reservoir_size, input_size))

W = np.random.uniform(0, 1, (reservoir_size, reservoir_size))
mask = np.random.rand(reservoir_size, reservoir_size) < sparsity
W *= mask
W *= spectral_radius / np.max(np.abs(np.linalg.eigvals(W)))

# 初始化状态
x = np.zeros((reservoir_size, 1))
states = []
# sA = spikeActivation(frame_size16, count_win_us)
# sA.init()

# 训练阶段
for t in range(train_length):
    x = (1 - leak_rate) * x + leak_rate * activate(Win @ u[t].reshape(-1, 1) + W @ x)
    states.append(x.flatten())

states = np.array(states)
extended_states = np.hstack([states, u[:train_length]])
Wout = np.linalg.pinv(extended_states) @ y[:train_length]

# 预测阶段
predictions = []
x = states[-1].reshape(-1, 1)
for t in range(train_length, total_length):
    u_t = u[t].reshape(-1, 1)
    x = (1 - leak_rate) * x + leak_rate * activate(Win @ u_t + W @ x)
    extended = np.hstack([x.flatten(), u_t.flatten()]).reshape(-1, 1)
    y_pred = (Wout.T @ extended).item()
    predictions.append(y_pred)

sA.deinit()
predictions = np.array(predictions).reshape(-1, 1)
np.savetxt('predictions.txt', predictions)

# 计算 NRMSE
nrmse = compute_nrmse(y[train_length:], predictions)
print(f"NRMSE: {nrmse:.4f}")

# # 激活输入范围
# activation_inputs = np.array(activate_inputs)
# print(f"Activation input range: [{activation_inputs.min():.3f}, {activation_inputs.max():.3f}]")

# 绘图比较
plt.figure(figsize=(10, 4))
plt.plot(range(train_length, total_length), y[train_length:], label='True')
plt.plot(range(train_length, total_length), predictions, label='Predicted')
plt.title(f'Mackey-Glass Prediction with ESN (NRMSE={nrmse:.4f})')
plt.xlabel('Time Step')
plt.ylabel('Value')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()