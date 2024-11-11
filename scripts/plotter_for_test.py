import numpy as np
import matplotlib.pyplot as plt

# Verileri yükle
data = np.genfromtxt('/home/furkan/controller_ws/src/ros_controllers/results/mpc_result.csv', delimiter=',', skip_header=1)

# Sütunları ayır
x = data[:, 0]
y = data[:, 1]
ref_x = data[:, 2]
ref_y = data[:, 3]

# Grafiği çiz
plt.figure(figsize=(12, 6))
plt.plot(x, y, 'b-', label='Takip edilen yol')
plt.plot(ref_x, ref_y, 'r--', label='Referans yol')
plt.xlabel('X pozisyonu')
plt.ylabel('Y pozisyonu')
plt.title('Takip Edilen Yol ve Referans Yol')
plt.legend()
plt.grid(True)
plt.show()
