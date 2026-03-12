import random
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food
from core.policies.mother import mother_policy_propose, apply_mother_intents
from core.policies.threat import threat_policy_propose, apply_threat_intents
from core.sim.movement import resolve_and_apply_moves
from collections import defaultdict, deque


class World:
    def __init__(self, grid_w, grid_h, mother_starts=None, child_start=None, 
                 food_positions=None, threat_starts=None, seed=42, day_step=None):
        # Fixed seed for reproducibility
        random.seed(seed)
        self.t = 0
        self.grid_w = grid_w
        self.grid_h = grid_h
        # self.time_of_day = day_step
        self.tick = 0
        self.day_step = day_step

        # Initialize mother agents
        self.mothers = []
        self.children = []
        self.threats = []

        self.PATROL_TIMEOUT = (self.grid_w * self.grid_h) // 2

        # 8-directions + stay
        self.RANDOM_MOVES = [(0,0), (1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        # Create mother agents
        for i, (mx, my) in enumerate(mother_starts):
            self.mothers.append(MotherAgent(mx, my, grid_w, grid_h, energy=100.0, mother_id=f"M{i}"))

        # Create child agents
        for i, (cx, cy) in enumerate(child_start):
            self.children.append(ChildAgent(cx, cy, grid_w, grid_h, energy=10.0, child_id=f"C{i}"))

        # Link mothers and children (now with like with index, should be adjust later)
        for m, c in zip(self.mothers, self.children):
            m.set_child(c)
            c.set_mother(m)
        
        # Initialize food
        self.foods = []
        if food_positions:
            for (fx, fy) in food_positions:
                self.foods.append(Food(fx, fy))

        # Create Threat
        for i, (cx, cy) in enumerate(threat_starts):
            self.threats.append(ThreatAgent(cx, cy, grid_w, grid_h, energy=50))

        # --- history for plotting ---
        self.history_len = 300
        self.tick_history = deque(maxlen=self.history_len)
        self.mother_history = defaultdict(lambda: {
            "energy": deque(maxlen=self.history_len),
            "fatigue": deque(maxlen=self.history_len),
            "bonding": deque(maxlen=self.history_len),
            "fear_threat": deque(maxlen=self.history_len),
            "stress": deque(maxlen=self.history_len),
            "closeness_child": deque(maxlen=self.history_len),
            "OT": deque(maxlen=self.history_len),
            "CORT": deque(maxlen=self.history_len),
        })


        
    def step(self, dt: float = 0.1):

        # Check enough cell for spawn new entities
        has_empty, empty_cell, occupied = (self.check_free_cell)
        if not has_empty:
            return
        
        # Internal dynamics before decision 
        for c in self.children:
            # c.print_state()
            c.update(self)
            # print(c.id, c.energy, c.alive)
            pass 

        for m in self.mothers:
            # m.update_psych_neuro(self)
            # print(m.id, m.energy, m.alive)

            # m.print_state()
            pass

        for t in self.threats:
            # t.print_state()
            t.energy -= 5
            if not t.is_alive():
                self.threats.remove(t)
            pass
        

        # Decision making
        m_prop, m_int = mother_policy_propose(self)
        t_prop, t_int = threat_policy_propose(self)

        all_agents = self.mothers + self.threats
        all_prop = {**m_prop, **t_prop}

        # Apply action to the environment
        resolve_and_apply_moves(all_agents, all_prop, self.grid_w, self.grid_h)
        apply_mother_intents(self, m_int)
        apply_threat_intents(self, t_int)

        # cleanup dead agent
        self.cleanup_dead_agents()
        self.cleanup_pick_entities()

        # Spawn Food within time steps
        spawn_interval = max(1, self.day_step // 10)
        if self.tick % spawn_interval == 0 and self.tick != 0:
            self.spawn_random_food(n=1, empty_cell=empty_cell, occupied=occupied)

        self.record_mother_states()
        self.t += dt
        self.tick += 1
        

    @property
    def time_of_day(self):
        return self.tick % self.day_step

    @property
    def is_day(self):
        return self.time_of_day < self.day_step // 2
    
    @property
    def day_count(self):
        return self.tick // self.day_step + 1
    
    def spawn_random_food(self, n=1, empty_cell=0, occupied=None):
        n = min(n, empty_cell)
        for _ in range(n):
            while True:
                x = random.randint(0, self.grid_w - 1)
                y = random.randint(0, self.grid_h - 1)

                if (x, y) not in occupied:
                    self.foods.append(Food(x, y))
                    occupied.add((x,y))
                    break

    def cleanup_dead_agents(self):
        dead_children = [c for c in self.children if not c.is_alive()]
        dead_mothers = [m for m in self.mothers if not m.is_alive()]

        for c in dead_children:
            if c.mother is not None:
                c.mother.child = None

        for m in dead_mothers:
            if m.child is not None:
                m.child.mother = None

        self.children = [c for c in self.children if c.is_alive()]
        self.mothers = [m for m in self.mothers if m.is_alive()]

    def cleanup_pick_entities(self):
        # picked_food = [f for f in self.foods if f.collected]
        self.foods = [f for f in self.foods if not f.collected]
        # print(picked_food)


    @property
    def has_living_agents(self):
        return len(self.mothers) > 0 or len(self.children) > 0
    
    @property
    def check_free_cell(self):
        # avoid spawning on agents
        occupied = {(m.x, m.y) for m in self.mothers}
        occupied |= {(c.x, c.y) for c in self.children}
        occupied |= {(f.x, f.y) for f in self.foods}

        total_grid = self.grid_w * self.grid_h
        empty_grid = total_grid - len(occupied)

        if empty_grid <= 0:
            print('[ERROR]: Grid full')
            return False, None, None
        return True, empty_grid, occupied
        

    def record_mother_states(self):
        self.tick_history.append(self.tick)

        for m in self.mothers:
            h = self.mother_history[m.id]
            h["energy"].append(m.energy)
            h["fatigue"].append(m.fatigue)
            h["bonding"].append(m.bonding)
            h["fear_threat"].append(m.fear_threat)
            h["stress"].append(m.stress)
            h["closeness_child"].append(m.closeness_child)
            h["OT"].append(m.OT)
            h["CORT"].append(m.CORT)