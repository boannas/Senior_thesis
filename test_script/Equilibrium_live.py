"""
Side-by-side population dynamics in Pygame.
Supports two birth modes:
  - "global": environment-level spawns (Poisson rate per second)
  - "per_agent": each agent reproduces independently (per-second chance)
"""
import math
import random
import pygame
from dataclasses import dataclass, field

# ------------------ Config ------------------
SEED = 42
FPS  = 60
DT   = 20 / FPS                 # fixed-step seconds per tick

# Canvas
W, H = 1200, 720
MARGIN = 40
PANEL_W = 500
PANEL_H = 520
GAP = 40

# Panels (edit here)
LEFT_CFG = dict(
    name="Blue",
    color=(70, 130, 180),
    birth_mode="global",   # "global" or "per_agent"
    env_birth_rate=1.0,    # expected new agents per second (if global)
    birth_rate=1.00,       # per-agent births/sec (if per_agent)
    death_rate=0.10,       # per-agent deaths/sec
    start_n=1
)
RIGHT_CFG = dict(
    name="Orange",
    color=(255, 160, 50),
    birth_mode="global",
    env_birth_rate=0.8,
    birth_rate=0.80,
    death_rate=0.02,
    start_n=1
)

# Motion
SPEED_MIN, SPEED_MAX = 20.0, 60.0    # px/s
JITTER = 30.0                        # random heading jitter (deg/s)
RADIUS = 16

BG      = (28, 31, 36)
PANELBG = (245, 245, 245)
PANELBD = (210, 210, 210)
TEXT    = (245, 235, 120)

random.seed(SEED)

# ------------------ Helpers ------------------
def rate_to_step_prob(rate_per_sec: float, dt: float) -> float:
    """Convert continuous-time Poisson rate λ to per-step event probability."""
    return 1.0 - math.exp(-max(rate_per_sec, 0.0) * dt)

def sample_poisson(lmbda: float) -> int:
    """Knuth's algorithm to sample k ~ Poisson(lambda) without numpy."""
    if lmbda <= 0: 
        return 0
    L = math.exp(-lmbda)
    k, p = 0, 1.0
    while p > L:
        k += 1
        p *= random.random()
    return k - 1

def clamp(v, lo, hi): 
    return max(lo, min(hi, v))

# ------------------ Model ------------------
@dataclass
class Agent:
    x: float
    y: float
    vx: float
    vy: float
    age: float = 0.0

    @staticmethod
    def random(panel_rect):
        x = random.uniform(panel_rect.left + RADIUS, panel_rect.right - RADIUS)
        y = random.uniform(panel_rect.top + RADIUS,  panel_rect.bottom - RADIUS)
        speed = random.uniform(SPEED_MIN, SPEED_MAX)
        ang = random.uniform(0, 2*math.pi)
        return Agent(x, y, speed*math.cos(ang), speed*math.sin(ang))

    def step_motion(self, rect, dt):
        # heading jitter
        ang = math.atan2(self.vy, self.vx)
        ang += math.radians(random.uniform(-JITTER, JITTER) * dt)
        speed = (self.vx**2 + self.vy**2) ** 0.5
        self.vx, self.vy = speed*math.cos(ang), speed*math.sin(ang)

        # move
        self.x += self.vx * dt
        self.y += self.vy * dt

        # bounce on panel edges
        if self.x < rect.left + RADIUS:  self.x = rect.left + RADIUS;  self.vx = abs(self.vx)
        if self.x > rect.right - RADIUS: self.x = rect.right - RADIUS; self.vx = -abs(self.vx)
        if self.y < rect.top + RADIUS:   self.y = rect.top + RADIUS;   self.vy = abs(self.vy)
        if self.y > rect.bottom - RADIUS:self.y = rect.bottom - RADIUS;self.vy = -abs(self.vy)

        self.age += dt

@dataclass
class Population:
    rect: pygame.Rect
    color: tuple
    birth_rate: float                 # per-agent births/sec (used if per_agent)
    death_rate: float                 # per-agent deaths/sec
    birth_mode: str = "per_agent"     # "per_agent" or "global"
    env_birth_rate: float = 0.0       # expected spawns/sec if global mode
    agents: list[Agent] = field(default_factory=list)

    def reset(self, n: int):
        self.agents = [Agent.random(self.rect) for _ in range(n)]

    def step(self, dt):
        # motion & aging
        for a in self.agents:
            a.step_motion(self.rect, dt)

        # per-agent death
        p_death = rate_to_step_prob(self.death_rate, dt)
        survivors: list[Agent] = [a for a in self.agents if random.random() >= p_death]

        newborns: list[Agent] = []

        if self.birth_mode == "per_agent":
            # per-agent reproduction near parent
            p_birth = rate_to_step_prob(self.birth_rate, dt)
            for a in survivors:
                if random.random() < p_birth:
                    jitter = 2 * RADIUS
                    nx = clamp(a.x + random.uniform(-jitter, jitter), self.rect.left+RADIUS, self.rect.right-RADIUS)
                    ny = clamp(a.y + random.uniform(-jitter, jitter), self.rect.top +RADIUS, self.rect.bottom-RADIUS)
                    speed = random.uniform(SPEED_MIN, SPEED_MAX)
                    ang = random.uniform(0, 2*math.pi)
                    newborns.append(Agent(nx, ny, speed*math.cos(ang), speed*math.sin(ang)))
        else:
            # GLOBAL SPAWNS: k ~ Poisson(env_birth_rate * dt), anywhere in panel
            k = sample_poisson(self.env_birth_rate * dt)
            for _ in range(k):
                newborns.append(Agent.random(self.rect))

        self.agents = survivors + newborns

    def stats(self):
        n = len(self.agents)
        avg_age = sum(a.age for a in self.agents)/n if n else 0.0
        birth_txt = (f"{int(self.birth_rate*100)}%" if self.birth_mode=="per_agent"
                     else f"{self.env_birth_rate:.2f}/s")
        return n, avg_age, birth_txt

# ------------------ View ------------------
def draw_panel(surface, rect, pop: Population, title_text, font):
    pygame.draw.rect(surface, PANELBG, rect, border_radius=22)
    pygame.draw.rect(surface, PANELBD, rect, width=2, border_radius=22)

    # agents
    for a in pop.agents:
        pygame.draw.circle(surface, pop.color, (int(a.x), int(a.y)), RADIUS)
        pygame.draw.circle(surface, (0,0,0), (int(a.x), int(a.y)), RADIUS, width=1)

    # stats text
    n, avg_age, birth_txt = pop.stats()
    lines = [
        f"Total: {n}",
        f"Average: {avg_age:.1f}",
        f"Birth: {birth_txt}",
        f"Death chance: {int(pop.death_rate*100)}%",
    ]
    text_x = rect.left
    text_y = rect.top - 110
    for i, line in enumerate(lines):
        surf = font.render(line, True, TEXT)
        surface.blit(surf, (text_x, text_y + i*26))

# ------------------ App ------------------
def main():
    random.seed(SEED)
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("arial", 22, bold=True)

    # Layout two panels
    left_rect  = pygame.Rect(MARGIN, MARGIN+120, PANEL_W, PANEL_H)
    right_rect = pygame.Rect(MARGIN + PANEL_W + GAP, MARGIN+120, PANEL_W, PANEL_H)

    # Populations
    left  = Population(left_rect,
                       LEFT_CFG["color"],
                       LEFT_CFG["birth_rate"],
                       LEFT_CFG["death_rate"],
                       birth_mode=LEFT_CFG["birth_mode"],
                       env_birth_rate=LEFT_CFG["env_birth_rate"])
    right = Population(right_rect,
                       RIGHT_CFG["color"],
                       RIGHT_CFG["birth_rate"],
                       RIGHT_CFG["death_rate"],
                       birth_mode=RIGHT_CFG["birth_mode"],
                       env_birth_rate=RIGHT_CFG["env_birth_rate"])
    left.reset(LEFT_CFG["start_n"])
    right.reset(RIGHT_CFG["start_n"])

    paused = False
    t = 0.0

    while True:
        # ---- events ----
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                pygame.quit(); return
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    pygame.quit(); return
                if e.key == pygame.K_SPACE:
                    paused = not paused
                if e.key == pygame.K_r:
                    left.reset(LEFT_CFG["start_n"])
                    right.reset(RIGHT_CFG["start_n"])
                    t = 0.0

        # ---- update ----
        if not paused:
            left.step(DT)
            right.step(DT)
            t += DT

        # ---- render ----
        screen.fill(BG)
        draw_panel(screen, left_rect,  left,  LEFT_CFG["name"],   font)
        draw_panel(screen, right_rect, right, RIGHT_CFG["name"], font)

        # headers
        hdr_left  = font.render(LEFT_CFG["name"], True, TEXT)
        hdr_right = font.render(RIGHT_CFG["name"], True, TEXT)
        screen.blit(hdr_left,  (left_rect.left,  left_rect.top - 140))
        screen.blit(hdr_right, (right_rect.left, right_rect.top - 140))

        # footer
        foot = font.render(f"t = {int(t)} s",
                           True, (220,220,220))
        # screen.blit(foot, (MARGIN, H - 40))

        # pygame.display.set_caption("Population Dynamics: Global vs Per-Agent Births")
        pygame.display.flip()
        clock.tick(FPS)

if __name__ == "__main__":
    main()
