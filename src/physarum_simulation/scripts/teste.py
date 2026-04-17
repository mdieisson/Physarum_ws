import pygame
import numpy as np
import random
import math

# --- Configurações do Artigo ---
LARGURA, ALTURA = 800, 600
TAMANHO_CELULA = 10  # Reduzi para criar uma "imagem" de maior resolução
COLS = LARGURA // TAMANHO_CELULA
LINS = ALTURA // TAMANHO_CELULA

# Cores (Baseadas no RGB descrito no artigo)
PRETO = (0, 0, 0)
BRANCO = (255, 255, 255)
# Artigo define obstáculos como "Amarelo" 
AMARELO_OBSTACULO = (255, 255, 0) 
VERDE_FEROMONIO = (0, 255, 0)
AZUL_START = (0, 0, 255)
VERMELHO_END = (255, 0, 0)

class IPEAC_Algorithm:
    def __init__(self):
        pygame.init()
        self.tela = pygame.display.set_mode((LARGURA, ALTURA))
        pygame.display.set_caption("IPEAC: Image-Based ACO [Space para Processar]")
        self.clock = pygame.time.Clock()
        
        # Estado do Sistema
        self.inicio = (5, LINS // 2)
        self.fim = (COLS - 5, LINS // 2)
        self.obstaculos = np.zeros((COLS, LINS), dtype=bool)
        self.feromonios = np.zeros((COLS, LINS)) # Intensidade do feromônio
        
        # Parâmetros ACO
        self.num_formigas = 50
        self.evaporacao = 0.05
        self.alfa = 1.0
        self.beta = 2.0
        
        # Modo de visualização (ACO ou IPEAC processado)
        self.modo_ipeac = False
        self.imagem_ipeac = None # Guarda o resultado do processamento

    def heuristica(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def run_aco_step(self):
        """Executa um passo da simulação de formigas (Eq. 1, 2, 3 do artigo)"""
        # Evaporação [cite: 112]
        self.feromonios *= (1.0 - self.evaporacao)
        self.feromonios[self.feromonios < 0.01] = 0

        caminhos_encontrados = []
        
        for _ in range(self.num_formigas):
            atual = self.inicio
            caminho = [atual]
            visitados = {atual}
            
            for _ in range(COLS * 2): # Limite de passos
                if atual == self.fim:
                    caminhos_encontrados.append(caminho)
                    break
                
                vizinhos = []
                probs = []
                denominador = 0
                
                # Vizinhos (8 direções para suavidade)
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0: continue
                        nx, ny = atual[0] + dx, atual[1] + dy
                        
                        if (0 <= nx < COLS and 0 <= ny < LINS and 
                            not self.obstaculos[nx, ny] and (nx, ny) not in visitados):
                            
                            # Modelo Probabilístico (Eq. 3) [cite: 121]
                            tau = self.feromonios[nx, ny] if self.feromonios[nx, ny] > 0 else 0.1
                            eta = 1.0 / (self.heuristica((nx, ny), self.fim) + 0.1)
                            
                            prob = (tau ** self.alfa) * (eta ** self.beta)
                            vizinhos.append((nx, ny))
                            probs.append(prob)
                            denominador += prob
                
                if not vizinhos or denominador == 0:
                    break
                
                # Seleção Roleta
                escolha = random.uniform(0, denominador)
                acumulado = 0
                proximo = vizinhos[0]
                for i, p in enumerate(probs):
                    acumulado += p
                    if acumulado >= escolha:
                        proximo = vizinhos[i]
                        break
                
                caminho.append(proximo)
                visitados.add(proximo)
                atual = proximo

        # Depósito de Feromônio (Eq. 1) [cite: 101]
        for caminho in caminhos_encontrados:
            custo = len(caminho)
            deposito = 10.0 / custo
            for (x, y) in caminho:
                self.feromonios[x, y] += deposito

    # --- LÓGICA IPEAC (Processamento de Imagem) ---
    def processar_ipeac(self):
        """
        Implementa o fluxo da Fig. 3 do artigo[cite: 161, 174].
        Transforma o estado atual em imagem e aplica CCA.
        """
        print("Iniciando IPEAC Image Processing...")
        
        # 1. Gerar a Imagem RGB baseada no estado atual
        # O artigo diz: Obstáculos = Amarelo, Fundo = Preto, Trilhas = Variável
        imagem_r = np.zeros((COLS, LINS))
        imagem_g = np.zeros((COLS, LINS))
        imagem_b = np.zeros((COLS, LINS))

        for x in range(COLS):
            for y in range(LINS):
                if self.obstaculos[x, y]:
                    # Obstáculo = Amarelo (R=1, G=1) 
                    imagem_r[x, y] = 1.0
                    imagem_g[x, y] = 1.0
                elif self.feromonios[x, y] > 0.01:
                    # Feromônio = Verde/Brilho
                    intensidade = min(1.0, self.feromonios[x, y] * 5)
                    imagem_g[x, y] = intensidade

        # 2. Filtragem de Obstáculos (Eq. 4) 
        # Se pixel é amarelo (R e G altos), remove.
        # Aqui simplificado: se está no array de obstáculos, zera.
        # O artigo remove obstáculos da imagem para processar só caminhos.
        mask_caminhos_r = np.where(self.obstaculos, 0, imagem_r)
        mask_caminhos_g = np.where(self.obstaculos, 0, imagem_g)
        mask_caminhos_b = np.where(self.obstaculos, 0, imagem_b)

        # 3. Conversão para Grayscale (Eq. 5) [cite: 198]
        # Gray = 0.2989R + 0.5870G + 0.1140B
        img_gray = (0.2989 * mask_caminhos_r) + \
                   (0.5870 * mask_caminhos_g) + \
                   (0.1140 * mask_caminhos_b)

        # 4. Binarização (Thresholding) (Eq. 6) 
        theta = 0.1 # Limiar
        img_bin = np.where(img_gray > theta, 1, 0)

        # 5. Connected Component Analysis (CCA) [cite: 240]
        # O artigo usa CCA para achar o maior componente conectado (caminho principal).
        # Implementação manual de CCA (Labeling) usando BFS para identificar ilhas.
        labels = np.zeros((COLS, LINS), dtype=int)
        label_count = 0
        component_sizes = {}

        for x in range(COLS):
            for y in range(LINS):
                if img_bin[x, y] == 1 and labels[x, y] == 0:
                    label_count += 1
                    count = 0
                    # BFS para preencher a ilha
                    queue = [(x, y)]
                    labels[x, y] = label_count
                    while queue:
                        cx, cy = queue.pop(0)
                        count += 1
                        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                            nx, ny = cx+dx, cy+dy
                            if 0 <= nx < COLS and 0 <= ny < LINS:
                                if img_bin[nx, ny] == 1 and labels[nx, ny] == 0:
                                    labels[nx, ny] = label_count
                                    queue.append((nx, ny))
                    component_sizes[label_count] = count

        # 6. Encontrar o Caminho Mais Forte (Maior Área) [cite: 186]
        if not component_sizes:
            self.imagem_ipeac = np.zeros((COLS, LINS, 3))
            return

        maior_label = max(component_sizes, key=component_sizes.get)
        
        # Gerar imagem final apenas com o componente vencedor (O Caminho Ótimo)
        img_final = np.zeros((COLS, LINS, 3))
        for x in range(COLS):
            for y in range(LINS):
                if labels[x, y] == maior_label:
                    img_final[x, y] = (0, 255, 255) # Ciano para o caminho final IPEAC
                elif self.obstaculos[x, y]:
                    img_final[x, y] = (255, 255, 0) # Mantém obstáculos visíveis
        
        self.imagem_ipeac = img_final
        print(f"IPEAC Concluído. Maior componente tem {component_sizes[maior_label]} pixels.")


    def desenhar(self):
        self.tela.fill(PRETO)

        if self.modo_ipeac and self.imagem_ipeac is not None:
            # Desenha o resultado do algoritmo IPEAC (Imagem Processada)
            surf = pygame.surfarray.make_surface(self.imagem_ipeac)
            self.tela.blit(surf, (0, 0))
            
            # Texto informativo
            font = pygame.font.SysFont('Arial', 20)
            text = font.render("MODO IPEAC (Caminho limpo via Processamento de Imagem)", True, BRANCO)
            self.tela.blit(text, (10, 10))

        else:
            # Desenha a simulação ACO "Crua"
            # Obstáculos
            for x in range(COLS):
                for y in range(LINS):
                    if self.obstaculos[x, y]:
                        pygame.draw.rect(self.tela, AMARELO_OBSTACULO, 
                                         (x*TAMANHO_CELULA, y*TAMANHO_CELULA, TAMANHO_CELULA, TAMANHO_CELULA))
                    elif self.feromonios[x, y] > 0.01:
                        intensidade = min(255, int(self.feromonios[x, y] * 100))
                        pygame.draw.rect(self.tela, (0, intensidade, 0), 
                                         (x*TAMANHO_CELULA, y*TAMANHO_CELULA, TAMANHO_CELULA, TAMANHO_CELULA))

            # Start/End
            pygame.draw.circle(self.tela, AZUL_START, 
                               (self.inicio[0]*TAMANHO_CELULA, self.inicio[1]*TAMANHO_CELULA), 8)
            pygame.draw.circle(self.tela, VERMELHO_END, 
                               (self.fim[0]*TAMANHO_CELULA, self.fim[1]*TAMANHO_CELULA), 8)

            font = pygame.font.SysFont('Arial', 20)
            text = font.render("MODO ACO (Gerando trilhas... [SPACE] para IPEAC)", True, BRANCO)
            self.tela.blit(text, (10, 10))

        pygame.display.flip()

    def run(self):
        rodando = True
        while rodando:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rodando = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if not self.modo_ipeac:
                            self.processar_ipeac()
                            self.modo_ipeac = True
                        else:
                            self.modo_ipeac = False # Volta para treinar mais
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = pygame.mouse.get_pos()
                    gx, gy = mx // TAMANHO_CELULA, my // TAMANHO_CELULA
                    if 0 <= gx < COLS and 0 <= gy < LINS:
                        self.obstaculos[gx, gy] = not self.obstaculos[gx, gy]

            if not self.modo_ipeac:
                self.run_aco_step()
            
            self.desenhar()
            self.clock.tick(60)

        pygame.quit()

if __name__ == "__main__":
    app = IPEAC_Algorithm()
    app.run()
