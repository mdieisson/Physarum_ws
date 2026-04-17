# physarum/ant_agent.py
import numpy as np

class AntAgent:
    def __init__(self, x, y, nest_x, nest_y):
        self.x = x
        self.y = y
        self.nest_x = nest_x # Coordenada X do robô (Ninho)
        self.nest_y = nest_y # Coordenada Y do robô (Ninho)
        
        self.has_food = False # Estado: False = Procurando, True = Voltando
        self.carrying_value = 0.0 # Quantidade de feromônio que vai depositar
        
        # Heading é mantido apenas para compatibilidade visual se necessário, 
        # mas o movimento real será por grid (vizinhos)
        self.heading = np.random.uniform(-np.pi, np.pi) 
        
        # Compatibilidade com o VisualizationSystem
        self.active = True 
        self.role = 'worker' 

    def reset_to_nest(self):
        """Reinicia a formiga no ninho (usado quando morre ou completa tarefa)"""
        self.x = self.nest_x
        self.y = self.nest_y
        self.has_food = False
        self.carrying_value = 0.0