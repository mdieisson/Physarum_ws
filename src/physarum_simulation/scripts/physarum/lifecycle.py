# physarum/lifecycle.py

import numpy as np
from scipy.ndimage import binary_dilation
class LifecycleSystem:
    def __init__(self, hunger_limit=50, reproduction_chance=0.1, max_agents=10000):
        self.hunger_limit = hunger_limit
        self.reproduction_chance = reproduction_chance
        self.max_agents = max_agents

    def redistribute_mass(self,agents, extra_mass):
        active_agents = [ag for ag in agents if ag.active and ag.strength < 5.0]

        if not active_agents or extra_mass <= 0:
            return

        mass_per_agent = extra_mass / len(active_agents)
        for agent in active_agents:
            agent.strength = min(agent.strength + mass_per_agent, 5.0)

    def update_lifecycle(self, agents, environment):
        survivors = []
        reabsorbed_mass = 0

        for agent in agents:
            mass, trail, food = environment.sense(agent.x, agent.y)

            if food+trail > 0.5:
                agent.feed()  # ganha força, se alimenta
                agent.active = True  # reativa agente se estiver em hibernação
                survivors.append(agent) 
            else:
                agent.starve()  # perde força gradativamente
                
                # Se ficar sem alimento por muito tempo, começa a retração
                if agent.time_since_food < self.hunger_limit:
                    survivors.append(agent) 
                else:
                    
                    agent.active = False  # agente fica inativo

                # if not agent.active:
                #     previous_strength = agent.strength
                #     agent.strength *= 0.1  # redução gradual da força
                #     reabsorbed_mass += (previous_strength - agent.strength)

                #         # se atingir força mínima residual, congela nesse estado
                #     agent.strength = max(agent.strength, 0.1)

             # nenhum agente removido diretamente

        # Redistribuir massa reabsorvida para agentes ativos ou reprodução
        self.redistribute_mass(survivors, reabsorbed_mass)

        return survivors


    # def _detect_mass_border(self, mass_map, threshold=0.2):
    #     """Detecta pixels na borda de regiões com massa usando dilatação binária."""
    #     from scipy.ndimage import binary_dilation

    #     border = (mass_map > threshold).astype(np.uint8)
    #     dilated = binary_dilation(border) & (border == 0)
    #     return dilated
    
    def reproduce_on_border(self, agents, mass_map, threshold=0.2, prob=0.9):
        """
        Reproduz agentes apenas nas bordas da massa viva com base na força e aleatoriedade.

        Parâmetros:
            agents: lista atual de agentes.
            mass_map: mapa de massa do ambiente.
            threshold: valor mínimo para considerar uma célula como "ativa".
            prob: probabilidade de reprodução por agente fértil na borda.

        Retorna:
            Lista de novos agentes gerados (adicionados manualmente no main).
        """
       
        if len(agents) == 0 or len(agents) >= self.max_agents:
            return []

        # Identifica as bordas da massa (frente viva)
        border_mask = self._mark_mass_border(mass_map, threshold)
      


        new_agents = []
        for agent in agents:
            yi, xi = int(agent.x), int(agent.y)

            # Verifica se o agente está em uma célula de borda ativa
            if (
                    0 <= xi < border_mask.shape[0] and
                    0 <= yi < border_mask.shape[1] and
                    border_mask[xi, yi] and
                    agent.strength > 0.5 and
                    np.random.rand() < prob
                ):

                #agent.strength = max(agent.strength, 2.0)
                offspring = agent.clone()
                offspring.x += np.random.uniform(-1, 1)
                offspring.y += np.random.uniform(-1, 1)
                offspring.x = np.clip(offspring.x, 0, border_mask.shape[0] - 1)
                offspring.y = np.clip(offspring.y, 0, border_mask.shape[1] - 1)
                offspring.heading += np.random.uniform(-0.2, 0.2)
                #offspring.strength *= 0.8
                offspring.strength = max(agent.strength * 0.8, 1.5)
                offspring.time_since_food = 0
                offspring.role = "explorer"

                new_agents.append(offspring)

        if new_agents:
            print(f"🌱 Gerados {len(new_agents)} novos agentes nas bordas")

        return agents+ new_agents 


    def _mark_mass_border(self, mass_map, threshold=0.2):
        border = (mass_map > threshold).astype(np.uint8)
        dilated = binary_dilation(border) & (border == 0)
        return dilated
    
    # def _mark_mass_border(self, mass_map,num, threshold=0.2):
    #         """Identifica pixels na borda da massa e marca agentes próximos."""
    #         from scipy.ndimage import binary_dilation

    #         border = (mass_map > threshold).astype(np.uint8)
    #         dilated = binary_dilation(border) & (border == 0)  # Borda externa

    #         mask = np.zeros(num, dtype=bool)
    #         x_int = self.x.astype(int)
    #         y_int = self.y.astype(int)

    #         for i in range(num):
    #             if 0 <= x_int[i] < self.height and 0 <= y_int[i] < self.width:
    #                 mask[i] = dilated[x_int[i], y_int[i]]

    #         return mask  # True para agentes na borda


# ---
# Justificativa Científica para o Sistema de Ciclo de Vida:
#
# O Physarum polycephalum demonstra comportamento adaptativo caracterizado por:
#   - Crescimento em direção a regiões ricas em nutrientes,
#   - Retração e morte de partes da rede em regiões inativas,
#   - Expansão preferencial nas bordas de sua massa ativa (frente viva).
#
# Para capturar esse comportamento de forma computacional:
#   - Agentes individuais perdem força progressivamente se não acessarem fontes de alimento,
#     simulando o enfraquecimento local do fluxo citoplasmático.
#   - Agentes morrem quando o tempo de fome excede um limite crítico, simulando retração da rede.
#   - Agentes se reproduzem estocasticamente nas bordas de regiões com alta massa,
#     modelando a expansão adaptativa da frente de crescimento do Physarum.
# 
# Essa dinâmica é inspirada diretamente em observações experimentais descritas por:
#   - Alim et al. (2013) — Random network peristalsis in Physarum polycephalum,
#   - Reid et al. (2016) — Decision-making without a brain,
#   - Nakagaki et al. (2000) — Maze-solving by an amoeboid organism.
#
# Portanto, o sistema de ciclo de vida proposto fornece uma abstração biologicamente plausível
# dos processos de crescimento, adaptação e retração que caracterizam o Physarum.
# ---



# Essa abordagem de reabsorção é amplamente sustentada pela literatura científica, especialmente nos trabalhos de Alim et al. (2013) sobre dinâmica adaptativa e estudos da organização da rede citoplasmática, que mostram claramente a realocação contínua da massa citoplasmática sem perda irreversível.

#     Alim et al. (2013): Random network peristalsis in Physarum polycephalum

#     Adamatzky (2010): Physarum Machines