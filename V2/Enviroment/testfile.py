import numpy as np
# import matplotlib.pyplot as plt

# plt.axis([0, 10, 0, 1])

# for i in range(10):
#     y = np.random.random()
#     plt.scatter(i, y)
#     plt.pause(0.05)

# plt.show()

x = np.array([(2,1)])
y = np.array([(4,5),(7,6)])
dist = np.linalg.norm(y-x,axis=1)
print(dist)