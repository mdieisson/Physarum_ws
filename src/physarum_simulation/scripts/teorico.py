import matplotlib.pyplot as plt
import numpy as np

# Número de robôs (N) de 1 a 100
N = np.arange(1, 101)

# Complexidade Tradicional (Mesh Completa / Flooding): N*(N-1)/2 ou O(N^2)
traditional_complexity = N * (N - 1) / 2

# Complexidade Physarum (Limitada por vizinhos físicos, digamos média de 3 vizinhos estáveis): O(N*k)
# Assumindo k=3 como média de vizinhos na trilha
physarum_complexity = N * 3 

plt.figure(figsize=(10, 6))
plt.plot(N, traditional_complexity, 'r--', label='Standard Mesh Protocol ($O(N^2)$)')
plt.plot(N, physarum_complexity, 'g-', linewidth=3, label='Proposed Physarum Approach ($O(N \cdot k)$)')

plt.xlabel('Number of Robots (N)')
plt.ylabel('Communication/Link Complexity')
plt.title('Theoretical Scalability Analysis')
plt.legend()
plt.grid(True)
plt.yscale('log') # Escala logarítmica destaca a diferença brutal

plt.savefig("theoretical_scalability.png")
print("Gráfico teórico gerado: theoretical_scalability.png")