# physarum/ant_colony.py
import numpy as np

class AntColonySystem:
    def __init__(self, alpha=1.0, beta=2.0, evaporation=0.95):
        self.alpha = alpha 
        self.beta = beta   
        self.evaporation = evaporation

    def apply_heavy_evaporation(self, env, factor=0.5):
        """
        D-ACO: Reduz drasticamente o feromônio quando o ambiente muda (robô anda).
        Isso ajuda a esquecer trilhas antigas (Ghost Trails).
        """
        env.trail_map *= factor

    def update(self, agents, env):
        # 1. Evaporação Global (Ciclo Natural)
        env.trail_map *= self.evaporation
        
        # 2. Reset do mapa de massa (Visualização)
        env.mass_map.fill(0) 

        # 3. Movimento e Interação
        for ant in agents:
            self.move_ant(ant, env)
            self.interact_with_environment(ant, env)
            
            # Desenha a formiga no mapa de massa
            if 0 <= int(ant.x) < env.height and 0 <= int(ant.y) < env.width:
                env.mass_map[int(ant.x), int(ant.y)] += 1.0

    def move_ant(self, ant, env):
        neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]

        probs = []
        valid_moves = []

        current_x, current_y = int(ant.x), int(ant.y)
        nest_x, nest_y = ant.nest_x, ant.nest_y

        for dx, dy in neighbors:
            nx, ny = current_x + dx, current_y + dy

            if 0 <= nx < env.height and 0 <= ny < env.width:
                if not env.obstacle_map[nx, ny]:
                    
                    # --- LÓGICA DE DECISÃO (PESOS) ---
                    if ant.has_food:
                        # MODO RETORNO (VOLTANDO PARA O ROBÔ)
                        # Estratégia: Ignorar feromônio velho, usar "GPS" forte
                        
                        tau = 1.0 # Peso neutro para o rastro (não segue trilhas antigas)
                        
                        # Heurística GPS Fortíssima (50.0)
                        dist = np.hypot(nx - nest_x, ny - nest_y)
                        eta = 50.0 / (dist + 0.1) 
                        
                        # Beta alto (4.0) para ser "guloso" na volta
                        score = (tau ** 0.5) * (eta ** 4.0)

                    else:
                        # MODO BUSCA (PROCURANDO VIZINHOS)
                        # Segue feromônio e sinal de comida
                        tau = env.trail_map[nx, ny] + 0.1 
                        
                        # Heurística: Sinal de Rádio/Comida
                        food_signal = env.food_map[nx, ny]
                        eta = food_signal + 0.1

                        score = (tau ** self.alpha) * (eta ** self.beta)

                    probs.append(score)
                    valid_moves.append((dx, dy))

        if valid_moves:
            total_prob = sum(probs)
            if total_prob > 0:
                probs = [p / total_prob for p in probs] 
                choice_idx = np.random.choice(len(valid_moves), p=probs)
                dx, dy = valid_moves[choice_idx]
                ant.x += dx
                ant.y += dy
            else:
                # Movimento aleatório (Jitter) se probabilidade for zero
                idx = np.random.randint(len(valid_moves))
                dx, dy = valid_moves[idx]
                ant.x += dx
                ant.y += dy
        else:
            # Se presa, fica parada (não reseta para evitar piscar)
            pass

    def interact_with_environment(self, ant, env):
        ix, iy = int(ant.x), int(ant.y)
        
        if not (0 <= ix < env.height and 0 <= iy < env.width):
            ant.reset_to_nest() 
            return

        if ant.has_food:
            # VOLTANDO: Deposita trilha
            env.deposit_trail(ix, iy, amount=0.5) 
            
            # Chegou em casa? (Distância < 3 pixels)
            dist_to_nest = np.hypot(ant.x - ant.nest_x, ant.y - ant.nest_y)
            if dist_to_nest < 3.0: 
                ant.has_food = False
                # Gira 180 graus virtualmente (opcional, aqui tratado pela heurística)
        
        else:
            # PROCURANDO: Encontrou sinal forte de comida/vizinho?
            if env.food_map[ix, iy] > 0.5:
                ant.has_food = True
                
                # REFORÇO IMEDIATO: Marca o ponto da comida com força!
                # Cria um "farol" químico para atrair as outras
                env.deposit_trail(ix, iy, amount=5.0)