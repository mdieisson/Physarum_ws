import matplotlib.pyplot as plt
import numpy as np
from skimage.morphology import skeletonize


class VisualizationSystem:
    def __init__(self, environment, agents_ref,robot_name, view_mode='mass'):
        self.env = environment
        self.agents_ref = agents_ref  # lista de agentes
        self.view_mode = view_mode
        self.skeleton_threshold = 0.2
        self.skeleton_alpha = 0.8
        self.robot_name = robot_name
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title(f"{self.robot_name} - Simulation Physarum Polycephalum")

       # self.im = self.ax.imshow(np.zeros_like(self.env.trail_map), cmap='viridis', vmin=0, vmax=1)
        self.im = self.ax.imshow(np.zeros_like(self.env.trail_map), cmap='viridis', vmin=0, vmax=1, zorder=1)

        self.obstacle_plot = self.ax.plot([], [], 'ks', markersize=4, alpha=0.8, label='Obstáculo', zorder=2)[0]
        self.food_plot     = self.ax.plot([], [], 'ro', markersize=5, alpha=0.9, label='Comida', zorder=3)[0]
        self.agent_plot    = self.ax.plot([], [], 'wo', markersize=2.5, alpha=0.9, label='Agente', zorder=4)[0]

        # self.agent_plot = self.ax.plot([], [], 'wo', markersize=2, alpha=0.6)[0]
        # self.food_plot = self.ax.plot([], [], 'ro', markersize=4, alpha=0.9, label='Comida')[0]
        # self.obstacle_plot = self.ax.plot([], [], 'ks', markersize=4, alpha=0.9, label='Obstaculo')[0]
        plt.colorbar(self.im)
        self.last_mouse_click = (0, 0)
        self.pending_action = None  # Pode ser 'add_food' ou 'move_food'

        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_click)

    def compute_skeleton(self, trail_map, threshold=10):
        """
        Converte o trail_map em uma versão binária e aplica esqueletonização.
        Retorna um array binário com a linha fina.
        """
        binary_map = trail_map > threshold
        skeleton = skeletonize(binary_map)
        return skeleton

    def on_mouse_click(self, event):
        if event.xdata is not None and event.ydata is not None:
            self.last_mouse_click = (event.ydata, event.xdata)
            print(f"[visual] Mouse clicado em ({event.ydata:.1f}, {event.xdata:.1f})")

    def on_key(self, event):
        modes = ['mass', 'trail', 'food', 'agents', 'stable']
        if event.key == 'v':
            idx = modes.index(self.view_mode)
            self.view_mode = modes[(idx + 1) % len(modes)]
            print(f"[visual] Modo de visualização: {self.view_mode}")
        elif event.key == 'a':
            self.pending_action = 'add_food'
            print("[visual] Ação agendada: adicionar comida.")
        elif event.key == 'x':
            self.pending_action = 'move_food'
            print("[visual] Ação agendada: mover comida.")

        elif event.key == 'd':
            self.pending_action = 'remove_food'
        elif event.key == 'o':
            self.pending_action = 'add_obstacle'
        

    def consume_pending_action(self):
        action = self.pending_action
        position = self.last_mouse_click
        self.pending_action = None  # Reseta após consumo
        return action, position

    def update(self, agents,step=0):
        self.agents_ref = agents
       
        
        if self.env.food_sources:
            fxo, fyo = zip(*self.env.food_sources)
        else:
            fxo, fyo = [], []

        # if self.env.obstacle_sources:
        #     fx, fy = zip(*self.env.obstacle_sources)
        # else:
        #     fx, fy = [], []

        
        self.food_plot.set_data(fyo, fxo)
        # self.obstacle_plot.set_data(fy, fx)
        if self.view_mode == 'mass':
            display = self.env.mass_map
        elif self.view_mode == 'trail':
            display = self.env.trail_map
        elif self.view_mode == 'food':
            display = self.env.food_map
        elif self.view_mode == 'stable':
            display = self.env.stable_trail_map
        
        elif self.view_mode == 'agents':
            heat = np.zeros_like(self.env.trail_map)
            for agent in self.agents_ref:
                xi, yi = int(agent.x), int(agent.y)
                if 0 <= xi < self.env.height and 0 <= yi < self.env.width:
                    heat[xi, yi] += 1
            display = heat
        else:
            display = self.env.mass_map
    
        self.im.set_array(display)
        self.im.set_clim(vmin=0, vmax=np.max(display) + 1e-8)
        #if np.max(self.env.obstacle_map) > 0:
           # import numpy.ma as ma
            #masked_obs = ma.masked_where(self.env.obstacle_map == 0, self.env.obstacle_map)
            #self.ax.imshow(masked_obs, cmap='gray', alpha=0.6)
            #self.ax.imshow(self.env.obstacle_map, cmap='gray', alpha=0.3)  # Transparente
        # x = [agent.y for agent in self.agents_ref]
        # y = [agent.x for agent in self.agents_ref]
        #self.agent_plot.set_data(x, y)
     
        self.ax.set_title(f"Steps {step} | Agents: {len(self.agents_ref)} | Mode: {self.view_mode}")
        self.fig.canvas.draw_idle()
        self.ax.grid(True, color='gray')
        plt.pause(0.01)
       

        
# ---
# Justificativa Científica para o Sistema de Visualização:
#
# A visualização em tempo real dos dados permite:
#   - Observar padrões emergentes de comportamento coletivo,
#   - Avaliar a formação adaptativa de trilhas entre fontes de comida,
#   - Validar qualitativamente a coerência com experimentos biológicos.
#
# Neste módulo, diferentes camadas de informação são exibidas:
#   - Massa (atividade local),
#   - Trilha (reforço do caminho),
#   - Comida (gradientes atrativos),
#   - Distribuição de agentes (população).
#
# Essa visualização é comparável a técnicas de imageamento usadas em estudos reais
# com *Physarum polycephalum*, como microscopia de contraste de fase ou traçadores fluorescentes.
#
# Assim, a visualização em tempo real contribui não apenas para depuração do modelo,
# mas também como ferramenta científica de análise e interpretação dos resultados.
# ---

