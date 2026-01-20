import numpy as np
from matplotlib import pyplot as plt

file = 'spike_counts.txt'
data = np.loadtxt(file)
plt.plot(data[0])
plt.title('Spike Counts')
plt.xlabel('Time')
plt.ylabel('Count')
plt.show()
