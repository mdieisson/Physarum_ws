# physarum/mass_update.py

import numpy as np
from scipy.ndimage import gaussian_filter

class MassSystem:
    def __init__(self, blur_sigma=1.0):
        self.blur_sigma = blur_sigma  # Grau de difusão da massa

    def update_mass(self, agents, environment):
        """
        Atualiza o mapa de massa com base na posição dos agentes.
        
        agents: lista de objetos Agent
        environment: objeto Environment
        """

        # Zera o mapa anterior
        environment.reset_mass()

        for agent in agents:
            environment.deposit_mass(agent.x, agent.y, amount=agent.strength)

        # Aplica difusão leve (gaussian blur)
        environment.mass_map = gaussian_filter(environment.mass_map, sigma=self.blur_sigma)


# ---
# Justificativa Científica para o Sistema de Atualização de Massa:
#
# No Physarum polycephalum, a distribuição espacial do fluxo citoplasmático
# reflete a atividade metabólica local e influencia a formação e reforço de tubos.
#
# Este módulo modela o acúmulo de massa:
#   - Agentes depositam massa proporcional à sua força em suas posições.
#   - Um leve desfoque é aplicado para simular a difusão natural do citoplasma.
#
# Assim, áreas de alta atividade metabólica emergem espontaneamente,
# servindo como guia para o reforço e decisão de movimento.
#
# Esta abordagem é inspirada nos estudos sobre shuttle streaming e formação adaptativa
# em Physarum descritos por Kamiya (1950) e Alim et al. (2013).
#
# Portanto, o sistema de atualização de massa implementa de forma biologicamente plausível
# a dinâmica essencial da organização espacial do Physarum.
# ---
