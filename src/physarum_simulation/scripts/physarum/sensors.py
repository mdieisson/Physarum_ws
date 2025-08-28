# physarum/sensors.py

import numpy as np

class SensorSystem:
    def __init__(self, sensing_distance=7, sensing_angle=np.pi/4):
        self.sensing_distance = sensing_distance
        self.sensing_angles = [0, sensing_angle, -sensing_angle]  # forward, left, right

    # def sense(self, agent, environment):
    #     """
    #     Realiza a leitura sensorial do ambiente ao redor do agente.
        
    #     Retorna:
    #         Uma lista com tuplas (massa, trilha, comida) para frente, esquerda e direita.
    #     """
    #     readings = []

    #     for angle_offset in self.sensing_angles:
    #         angle = agent.heading + angle_offset
    #         sx = agent.x + self.sensing_distance * np.cos(angle)
    #         sy = agent.y + self.sensing_distance * np.sin(angle)

    #         mass, trail, food = environment.sense(sx, sy)
    #         readings.append((mass, trail, food))

    #     return readings

    def sense(self, agent, environment):
        """
        Realiza a leitura sensorial do ambiente ao redor do agente.
        
        Retorna:
            Uma lista com tuplas (massa, trilha, comida) para frente, esquerda e direita.
        """
        readings = []

        for angle_offset in self.sensing_angles:
            angle = agent.heading + angle_offset
            if agent.role != 'explorer':
                sx = agent.x + self.sensing_distance * np.cos(angle)
                sy = agent.y + self.sensing_distance * np.sin(angle)
            else:
                sx = agent.x + 50 * np.cos(angle)
                sy = agent.y + 50 * np.sin(angle)

            mass, trail, food = environment.sense(sx, sy)
            readings.append((mass, trail, food))
        
       # self.sensing_distance *= 0.8

        return readings
    
    # def sense(self, agent, environment):
    #     readings = []

    #     for angle_offset in self.sensing_angles:
    #         angle = agent.heading + angle_offset

    #         if agent.role != 'explorer':
    #              # Percepção próxima (precisão direcional)
    #             sx_near = agent.x + self.sensing_distance * np.cos(angle)
    #             sy_near = agent.y + self.sensing_distance * np.sin(angle)
    #             mass_n, trail_n, food_n = environment.sense(sx_near, sy_near)
    #             # faz leitura próxima e distante
    #         else:
    #             sx_far = agent.x + 50 * np.cos(angle)
    #             sy_far = agent.y + 50 * np.sin(angle)
    #             _, _, food_f = environment.sense(sx_far, sy_far)
    #             # faz apenas leitura próxima

    #         # # Percepção próxima (precisão direcional)
    #         # sx_near = agent.x + self.sensing_distance * np.cos(angle)
    #         # sy_near = agent.y + self.sensing_distance * np.sin(angle)
    #         # mass_n, trail_n, food_n = environment.sense(sx_near, sy_near)

    #         # Percepção ampla (descoberta estratégica)
    #         sx_far = agent.x + 50 * np.cos(angle)
    #         sy_far = agent.y + 50 * np.sin(angle)
    #         _, _, food_f = environment.sense(sx_far, sy_far)

    #         # Combinar heurística
    #         food_combined = max(food_n, 0.5 * food_f)
    #         readings.append((mass_n, trail_n, food_combined))

    #     return readings
# ---
# Justificativa Científica para o Sistema de Sensores:
#
# No Physarum polycephalum, o organismo detecta gradientes químicos (como nutrientes e atrativos)
# através de mudanças locais no fluxo citoplasmático e contrações da rede tubular.
#
# Para simular essa capacidade de sensoriamento de maneira computacional:
#   - Cada agente avalia o ambiente à sua frente, à esquerda e à direita.
#   - Sensores virtuais percebem localmente: concentração de massa, trilha e comida.
#   - Essas percepções influenciam suas decisões de movimento de forma descentralizada.
#
# Essa modelagem é inspirada em experimentos reais que mostram que Physarum utiliza
# sensoriamento distribuído para guiar seu crescimento (Alim et al., 2013).
#
# Portanto, o sistema de sensores aproxima de maneira biologicamente plausível a percepção
# ambiental descentralizada típica do Physarum.

#ref Alim, K., Peaudecerf, F., Brenner, M. P., & Pringle, A. (2013). Random network peristalsis in Physarum polycephalum.
# ---
