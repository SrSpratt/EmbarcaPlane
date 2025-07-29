import numpy as np
import matplotlib.pyplot as plt

# Carrega os dados, pulando a primeira linha de cabeçalho e usando vírgula como separador
data = np.loadtxt('new_file.txt', delimiter=',', skiprows=1)

tempo = np.arange(data.shape[0])

# Colunas dos giroscópios (G_x, G_y, G_z)
G_x = data[:, 0]
G_y = data[:, 1]
G_z = data[:, 2]

# Colunas dos acelerômetros (A_x, A_y, A_z)
A_x = data[:, 3]
A_y = data[:, 4]
A_z = data[:, 5]

# Gráfico dos giroscópios
plt.figure(figsize=(10,5))
plt.plot(tempo, G_x, label='G_x')
plt.plot(tempo, G_y, label='G_y')
plt.plot(tempo, G_z, label='G_z')
plt.title('Giroscópio (G_x, G_y, G_z)')
plt.xlabel('Índice da amostra (tempo)')
plt.ylabel('Valor do Giroscópio')
plt.legend()
plt.grid()

# Gráfico dos acelerômetros
plt.figure(figsize=(10,5))
plt.plot(tempo, A_x, label='A_x')
plt.plot(tempo, A_y, label='A_y')
plt.plot(tempo, A_z, label='A_z')
plt.title('Acelerômetro (A_x, A_y, A_z)')
plt.xlabel('Índice da amostra (tempo)')
plt.ylabel('Valor do Acelerômetro')
plt.legend()
plt.grid()

plt.show()
