import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Simulação de dados (caso você não tenha rodado ainda, use isso para testar o plot)
# Quando tiver o arquivo real, use: df = pd.read_csv('/tmp/robot_2_complexity_log.csv')
data = {
    'step': np.arange(0, 500, 10),
    'potential_links': [2] * 50,  # Com 3 robôs, cada um tem 2 vizinhos potenciais
    # Physarum começa baixo e sobe, mas as vezes cai (adaptação)
    'physarum_links': [0]*5 + [1]*10 + [2]*20 + [1]*15, 
}
df = pd.DataFrame(data)

plt.figure(figsize=(10, 6))

# Área sombreada representando o custo de uma rede tradicional
plt.fill_between(df['step'], 0, df['potential_links'], color='gray', alpha=0.3, label='Traditional Network Overhead (Flooding)')

# Linha do seu método
plt.plot(df['step'], df['physarum_links'], color='green', linewidth=3, label='Physarum Adaptive Links')

plt.xlabel('Simulation Steps')
plt.ylabel('Active Communication Channels')
plt.title('Reduction in Communication Complexity: Physarum vs Traditional')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)

plt.text(200, 0.5, "Bandwidth Saving", fontsize=12, color='black', fontweight='bold')

plt.savefig("communication_complexity_proof.png")
print("Gráfico gerado: communication_complexity_proof.png")