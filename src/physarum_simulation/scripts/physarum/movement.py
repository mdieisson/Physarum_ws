# physarum/movement.py

import numpy as np


class MovementSystem:
    def __init__(self, heading_rate=np.pi/6, exploration_chance=0.1, speed=1.0):
        self.heading_rate = heading_rate  # Quanto o agente gira ao decidir virar
        self.exploration_chance = exploration_chance  # Chance de exploração aleatória
        self.speed = speed  # Velocidade de movimento

    def decide_and_move(self, agent, sensor_readings,env):
        """
        Decide a rotação e move o agente com base nas leituras dos sensores.

        sensor_readings: lista [(massa, trilha, comida), ...] para forward, left, right
        """
        # weight_mass = 1  # Peso para a massa de outros agentes
        # weight_trail = 0.1  # Peso para trilhas químicas
        # weight_food = 1.0  # Peso para fontes de alimento
              # Peso dos estímulos (ajustável)

        # # Peso dos estímulos (ajustável)
        weight_mass = 0.1# Peso para a massa de outros agentes
        weight_trail = 0.5# Peso para trilhas químicas
        weight_food = 2.00 # Peso para fontes de alimento
              # Peso dos estímulos (ajustável)
        # weight_mass = 0.1  # Peso para a massa de outros agentes
        # weight_trail = 1.0 # Peso para trilhas químicas
        # weight_food = 1.5 # Peso para fontes de alimento

        # Calcula a "atratividade" para cada direção
        scores = []
        for mass, trail, food in sensor_readings:
            score = weight_mass * mass + weight_trail * trail + weight_food * food
            scores.append(score)

        # Estratégia de decisão
        forward_score, left_score, right_score = scores
        
        if agent.role == "explorer":
            # Pequena rotação aleatória (exploração)
            agent.rotate(np.random.uniform(-self.heading_rate, self.heading_rate))
        else:
            if left_score > right_score and left_score > forward_score:
                # Gira para a esquerda
                agent.rotate(self.heading_rate)
            elif right_score > left_score and right_score > forward_score:
                # Gira para a direita
                agent.rotate(-self.heading_rate)
            else:
                # Segue em frente (pode adicionar leve ruído)
                agent.rotate(np.random.normal(0, self.heading_rate))

        # Move o agente na direção atual
        agent.move(self.speed,env)


        



# ---
# Justificativa Científica para o Sistema de Movimentação:
#
# No Physarum polycephalum, o crescimento da rede é guiado por uma combinação
# de gradientes químicos e flutuações estocásticas do citoplasma.
#
# Este módulo modela a movimentação dos agentes de forma biologicamente plausível:
#   - A rotação é baseada na direção que apresenta maior estímulo (comida, trilha, massa).
#   - Um componente de ruído é introduzido para simular a natureza exploratória e adaptativa.
#   - O agente move-se de forma contínua no espaço bidimensional.
#
# Esta abordagem é inspirada nos estudos de auto-organização, tomada de decisão descentralizada
# e otimização de redes naturais no Physarum, conforme discutido em Alim et al. (2013) e Reid et al. (2016).
#
# Portanto, o sistema de movimentação é cientificamente embasado e essencial para capturar
# o comportamento emergente de formação de trilhas e redes de transporte.
# ---
