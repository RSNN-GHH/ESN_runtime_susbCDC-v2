import numpy as np
from matplotlib import pyplot as plt

file = 'encoded_input.txt'
data = np.loadtxt(file)
plt.plot(data)
plt.title('Encoded Input')
plt.xlabel('Time')
plt.ylabel('Count')
plt.show()