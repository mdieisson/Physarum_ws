# physarum/deposition.py
import numpy as np
class DepositionSystem:
    def __init__(self, base_amount=0.05):
        self.base_amount = base_amount  # Quantidade base de feromônio a depositar


    # def deposit(self, agent, environment):
    #     amount = self.base_amount * agent.strength

    #     # Calcula direção do movimento
    #     dx = agent.x - agent.prev_x
    #     dy = agent.y - agent.prev_y

    #     norm = np.hypot(dx, dy) + 1e-8  # Evita divisão por zero

    #     dx /= norm
    #     dy /= norm

    #     # Calcula ponto à frente na direção do movimento
    #     x_dep = int(agent.x + dx)
    #     y_dep = int(agent.y + dy)

    #     # Garante que está dentro dos limites
    #     x_dep = np.clip(x_dep, 0, environment.trail_map.shape[0] - 1)
    #     y_dep = np.clip(y_dep, 0, environment.trail_map.shape[1] - 1)

    #     # Deposita apenas nesse ponto à frente
    #     environment.trail_map[x_dep, y_dep] += amount

    def deposit(self, agent, environment,beta = 2.0,gamma = 1.2,trail_threshold = 0.1, max_trail_value = 0.5 ):
        """
        Deposita sinal no ambiente com base no contexto local do agente.
        """
        mass, trail, food = environment.sense(agent.x, agent.y)


        amount = self.base_amount * agent.strength
        amount = self.base_amount 
      
        if trail < 0:  # está em região repulsiva
            return 
        if food  > 0:
            amount *= beta * food  # reforça proporcionalmente à concentração de comida
        if trail > trail_threshold:
            amount *= gamma  

        if agent.role != "explorer":
            environment.deposit_trail(agent.x, agent.y, self.base_amount)
        else:

            environment.deposit_trail(agent.x, agent.y, amount)
        #np.clip(environment.trail_map, 0, max_trail_value, out=environment.trail_map)

# ---
# Justificativa Científica para o Sistema de Deposição:
#
# No Physarum polycephalum, o reforço dos tubos ocorre onde há maior fluxo citoplasmático
# e onde fontes de alimento estão presentes, levando à formação adaptativa de redes eficientes.
#
# Este módulo modela a deposição de sinais químicos (trilhas) no ambiente:
#   - A quantidade depositada depende da força do agente (simulando o fluxo local).
#   - Deposição é aumentada em regiões com alta massa ou presença de comida.
#   - Áreas pouco exploradas ou sem interesse não são reforçadas significativamente.
#
# Esse comportamento é consistente com a dinâmica emergente de reforço de tubos
# e otimização de redes naturais descrita em Adamatzky (2010) e Alim et al. (2013).
#
# Portanto, o sistema de deposição implementa de maneira biologicamente plausível
# o mecanismo de formação de trilhas adaptativas no Physarum.
# ---
