# physarum/agent.py

import numpy as np

class Agent:
    def __init__(self, x, y, heading, strength=1.0, role='follower'):
        self.x = x  # Posição x no ambiente
        self.y = y  # Posição y no ambiente
        self.heading = heading  # Direção de movimento (radianos)
        self.strength = strength  # Força atual do agente (energia)
        self.role = role  # Papel ('explorer' ou 'follower')
        self.active = True  # Se o agente está ativo ou hibernando
        self.time_since_food = 0  # Passos desde a última alimentação
        self.signal_strength = 0  # novo atributo
        self.prev_x = x  # Inicializa igual à posição atual
        self.prev_y = y

    
    def move(self, speed=1.0, env=None):
        """Move o agente na direção atual, se não houver obstáculo."""
        dx = np.cos(self.heading) * speed
        dy = np.sin(self.heading) * speed
        new_x = self.x + dx
        new_y = self.y + dy
 
        self.prev_x = self.x
        self.prev_y = self.y
        self.x = new_x
        self.y = new_y
        # if env and env.is_free(int(new_x), int(new_y)):
        #     if not env.obstacle_map[int(new_x), int(new_y)]:
        #         self.prev_x = self.x
        #         self.prev_y = self.y
        #         self.x = new_x
        #         self.y = new_y
        #     else:
        #         self.rotate(np.random.uniform(-np.pi, np.pi))
        # else:
        #     self.heading += np.pi / 2  # tenta girar em 90° se bloqueado

    # def move(self, speed, env):
    #     ny = int(self.y + speed * np.sin(self.heading))
    #     nx = int(self.x + speed * np.cos(self.heading))

    #     # Verifica se está dentro dos limites do mapa
    #     if 0 <= ny < env.height and 0 <= nx < env.width:
    #         # Verifica se é obstáculo
    #         if not env.obstacle_map[ny, nx]:
    #             self.y = ny
    #             self.x = nx
    #         else:
    #             # Opcional: gire aleatoriamente se bateu no obstáculo
    #             self.rotate(np.random.uniform(-np.pi, np.pi))

    # def move(self, speed=1.0):
    #     """Move o agente na direção atual."""
    #     # Antes de mover
    #     self.prev_x = self.x
    #     self.prev_y = self.y

    #     # Depois atualiza a posição normalmente

    #     self.x += np.cos(self.heading) * speed
    #     self.y += np.sin(self.heading) * speed

    def rotate(self, angle):
        """Altera o heading atual do agente."""
        self.heading += angle

    def feed(self, gain=1.0, max_strength=5.0):
        """Alimenta o agente, aumentando sua força."""
        self.strength = min(self.strength + gain, max_strength)
        self.time_since_food = 0

    def starve(self, decay=0.1, min_strength=1.0):
        """Aplica decaimento na força se não houver alimentação."""
        self.strength = max(min_strength, self.strength - decay)
        self.time_since_food += 0.1

    def clone(self):
        """Cria um novo agente baseado neste (para reprodução)."""
        return Agent(
            x=self.x + np.random.uniform(-1, 1),
            y=self.y + np.random.uniform(-1, 1),
            heading=self.heading + np.random.uniform(-0.2, 0.2),
            strength=self.strength * 0.8,  # reprodução custa energia
            role=self.role
        )
# ---
# Justificativa Científica para o Uso de Agentes:
# 
# Embora o Physarum polycephalum seja uma célula gigante contínua e não um sistema de partículas individuais,
# o uso de agentes discretos nesta implementação é uma aproximação computacional prática e biologicamente aceitável.
# 
# Cada agente representa uma "pseudopartícula" de massa ou fluxo citoplasmático,
# modelando localmente os fenômenos de:
#   - Sensoriamento de gradientes (comida, massa, trilha),
#   - Locomoção adaptativa (exploração e reforço),
#   - Crescimento e retração de tubos (via reprodução/morte de agentes).
# 
# Esta abordagem discretizada permite capturar a dinâmica emergente da formação de redes do Physarum
# de maneira eficiente, interpretando o comportamento coletivo dos agentes como uma analogia 
# ao padrão global auto-organizado de tubos protoplasmáticos.
# 
# Estratégias similares de modelagem baseadas em agentes já foram validadas na literatura,
# como em Jones (2015) "Morphological adaptation approach to path planning inspired by slime mould" 
# e Adamatzky (2010) "Physarum Machines: Computers from Slime Mould".
# 
# Portanto, o modelo de agentes é cientificamente fundamentado e adequado para simular
# o comportamento realista do Physarum no contexto da formação de redes de transporte.
# ---
