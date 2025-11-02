# main.py
import pygame
import sys
import random

# -------- Config --------
GRID_W, GRID_H = 15, 15        # cells
CELL = 50                      # pixels per cell
FPS = 30
SEED = 42                      # reproducible spawn, if you randomize later

# Colors (RGB)
BG   = (248, 248, 248)
GRID = (220, 220, 220)
AGNT = (235, 87, 87)           # red-ish
OUTL = (120, 120, 120)

# -------- Core State --------
class Agent:
    def __init__(self, x, y):
        self.x, self.y = x, y

    def move(self, dx, dy):
        # Clamp to grid bounds
        self.x = max(0, min(GRID_W - 1, self.x + dx))
        self.y = max(0, min(GRID_H - 1, self.y + dy))

class World:
    def __init__(self, seed=SEED):
        random.seed(seed)
        self.t = 0
        # Spawn agent at center
        self.agent = Agent(GRID_W // 2, GRID_H // 2)

    def step(self, action: int):
        """
        action: 0=stay, 1=up, 2=down, 3=left, 4=right
        """
        moves = {0: (0, 0), 1: (0, -1), 2: (0, 1), 3: (-1, 0), 4: (1, 0)}
        dx, dy = moves.get(action, (0, 0))
        self.agent.move(dx, dy)
        self.t += 1

# -------- Rendering --------
def draw_grid(surface: pygame.Surface):
    surface.fill(BG)
    # Outer border
    pygame.draw.rect(surface, OUTL, (0, 0, GRID_W * CELL, GRID_H * CELL), width=2)
    # Internal grid lines
    for x in range(1, GRID_W):
        pygame.draw.line(surface, GRID, (x * CELL, 0), (x * CELL, GRID_H * CELL))
    for y in range(1, GRID_H):
        pygame.draw.line(surface, GRID, (0, y * CELL), (GRID_W * CELL, y * CELL))

def draw_agent(surface: pygame.Surface, agent: Agent):
    cx = agent.x * CELL + CELL // 2
    cy = agent.y * CELL + CELL // 2
    r = CELL // 3
    pygame.draw.circle(surface, AGNT, (cx, cy), r)
    pygame.draw.circle(surface, OUTL, (cx, cy), r, width=2)

# -------- Main Loop --------
def main():
    pygame.init()
    try:
        screen = pygame.display.set_mode((GRID_W * CELL, GRID_H * CELL))
    except Exception as e:
        print("Pygame display init failed:", e)
        pygame.quit()
        sys.exit(1)

    clock = pygame.time.Clock()
    world = World()

    while True:
        action = 0  # default: no move this tick

        # --- Input & events ---
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit(0)
                if e.key in (pygame.K_UP, pygame.K_w):    action = 1
                if e.key in (pygame.K_DOWN, pygame.K_s):  action = 2
                if e.key in (pygame.K_LEFT, pygame.K_a):  action = 3
                if e.key in (pygame.K_RIGHT, pygame.K_d): action = 4

        # --- Update world ---
        world.step(action)

        # --- Render ---
        draw_grid(screen)
        draw_agent(screen, world.agent)

        # Title with tick/pos/FPS
        fps = int(clock.get_fps())
        pygame.display.set_caption(
            f"GridWorld | t={world.t} | pos=({world.agent.x},{world.agent.y}) | fps={fps}"
        )
        pygame.display.flip()
        clock.tick(FPS)

if __name__ == "__main__":
    main()
