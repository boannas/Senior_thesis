import random
from core.agents import ChildAgent, MotherAgent, ThreatAgent
from core.entities import Food
from core.policies.mother import mother_policy_propose, apply_mother_intents, update_plasticity, update_plasticity_hebbian
from core.policies.threat import threat_policy_propose, apply_threat_intents
from core.sim.movement import resolve_and_apply_moves
from collections import defaultdict, deque

import random
import numpy as np

random.seed(42)
np.random.seed(42)

class World:
    def __init__(self, grid_w, grid_h, mother_starts=None, child_start=None, 
                 food_positions=None, threat_starts=None, seed=42, day_step=None, plasticity_rule=None,
                 food_spawn_interval=None, food_spawn_n=1):
        # plasticity_rule: None (off), "outcome" (deficit-gated), or "hebbian" (co-activity)
        self.plasticity_rule = plasticity_rule
        # Food spawning: if None, use default in step() (day_step//10, n=1)
        self.food_spawn_interval = food_spawn_interval
        self.food_spawn_n = food_spawn_n
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

            "mot_forage": deque(maxlen=self.history_len),
            "mot_care": deque(maxlen=self.history_len),
            "mot_self": deque(maxlen=self.history_len),
            "mot_protect": deque(maxlen=self.history_len),
            # "mot_selected": deque(maxlen=self.history_len),


            "sel_forage": deque(maxlen=self.history_len),
            "sel_care": deque(maxlen=self.history_len),
            "sel_self": deque(maxlen=self.history_len),
            "sel_protect": deque(maxlen=self.history_len),
        })


        self.child_history = defaultdict(lambda: {
            "hunger": deque(maxlen=self.history_len),
            "warmth": deque(maxlen=self.history_len),
            "injury": deque(maxlen=self.history_len),
            # "carried": deque(maxlen=self.history_len),
            # "threat_dist": deque(maxlen=self.history_len),
        })
        
    def step(self, dt: float = 0.1):

        # Check enough cell for spawn new entities
        has_empty, empty_cell, occupied = (self.check_free_cell)
        if not has_empty:
            return
        
        # Internal dynamics before decision 
        for c in self.children:
            if not c.is_alive():
                continue
            c.update(self)

        for m in self.mothers:
            if not m.is_alive():
                continue
            m.update_psych_neuro(self)

        for t in self.threats:
            # t.print_state()
            # t.energy -= 5
            # if not t.is_alive():
            #     self.threats.remove(t)
            pass
        

        # Motivation select (policies skip dead agents)
        m_prop, m_int = mother_policy_propose(self)
        t_prop, t_int = threat_policy_propose(self)

        all_agents = [m for m in self.mothers if m.is_alive()] + [t for t in self.threats if t.is_alive()]
        all_prop = {**m_prop, **t_prop}

        # Apply action to the environment
        resolve_and_apply_moves(all_agents, all_prop, self.grid_w, self.grid_h)
        apply_mother_intents(self, m_int)

        # Plasticity: set to "outcome" (deficit-based) or "hebbian" (co-activity); None = off.
        self._plasticity_this_tick = False
        plasticity_rule = self.plasticity_rule
        if plasticity_rule == "outcome":
            for m in self.mothers:
                if m.is_alive():
                    update_plasticity(m, m_int.get("intended_actions", {}).get(m), self)
            self._plasticity_this_tick = True
        elif plasticity_rule == "hebbian":
            for m in self.mothers:
                if m.is_alive():
                    update_plasticity_hebbian(m, m_int.get("intended_actions", {}).get(m), self)
            self._plasticity_this_tick = True
        # To enable: set world.plasticity_rule = "outcome" or "hebbian" (e.g. in main after creating World).
        apply_threat_intents(self, t_int)

        # cleanup dead agent
        self.cleanup_dead_agents()
        self.cleanup_pick_entities()

        # Spawn Food within time steps
        spawn_interval = self.food_spawn_interval if self.food_spawn_interval is not None else max(1, self.day_step // 10)
        spawn_n = getattr(self, "food_spawn_n", 1)
        if self.tick % spawn_interval == 0 and self.tick != 0:
            self.spawn_random_food(n=spawn_n, empty_cell=empty_cell, occupied=occupied)

        self.record_child_states()
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
        """Unlink dead agents from their child/mother; do NOT remove from list so indices (M0,M1,M2) stay fixed."""
        dead_children = [c for c in self.children if not c.is_alive()]
        dead_mothers = [m for m in self.mothers if not m.is_alive()]

        for c in dead_children:
            if c.mother is not None:
                c.mother.child = None

        for m in dead_mothers:
            if m.child is not None:
                m.child.mother = None

        # Keep lists fixed (no removal) so M0,M1,M2 and C0,C1,C2 identities stay by index
        # self.children = [c for c in self.children if c.is_alive()]
        # self.mothers = [m for m in self.mothers if m.is_alive()]

    def cleanup_pick_entities(self):
        self.foods = [f for f in self.foods if not f.collected]

    @property
    def has_living_agents(self):
        return any(m.is_alive() for m in self.mothers) or any(c.is_alive() for c in self.children)
    
    @property
    def check_free_cell(self):
        # avoid spawning on agents (only alive count as occupied)
        occupied = {(m.x, m.y) for m in self.mothers if m.is_alive()}
        occupied |= {(c.x, c.y) for c in self.children if c.is_alive()}
        occupied |= {(f.x, f.y) for f in self.foods}

        total_grid = self.grid_w * self.grid_h
        empty_grid = total_grid - len(occupied)

        if empty_grid <= 0:
            print('[ERROR]: Grid full')
            return False, None, None
        return True, empty_grid, occupied
        
    def record_mother_states(self):
        self.tick_history.append(self.tick)

        mot_to_idx = {
            "Forage": 0,
            "Care": 1,
            "Self": 2,
            "Protect": 3,
            }
        
        for m in self.mothers:
            if not m.is_alive():
                continue
            h = self.mother_history[m.id]
            h["energy"].append(m.energy)
            h["fatigue"].append(m.fatigue)
            h["bonding"].append(m.bonding)
            h["fear_threat"].append(m.fear_threat)
            h["stress"].append(m.stress)
            h["closeness_child"].append(m.closeness_child)
            h["OT"].append(m.OT)
            h["CORT"].append(m.CORT)

            h["mot_forage"].append(m.motivations["Forage"])
            h["mot_care"].append(m.motivations["Care"])
            h["mot_self"].append(m.motivations["Self"])
            h["mot_protect"].append(m.motivations["Protect"])

            selected = max(m.motivations, key=m.motivations.get)

            h["sel_forage"].append(1 if selected == "Forage" else 0)
            h["sel_care"].append(1 if selected == "Care" else 0)
            h["sel_self"].append(1 if selected == "Self" else 0)
            h["sel_protect"].append(1 if selected == "Protect" else 0)

            # Record u and w fixed vs plastic for comparison plots
            if hasattr(m, "u_fixed") and hasattr(m, "u_plastic"):
                for cat in m.u_fixed:
                    for key in m.u_fixed[cat]:
                        k_f, k_p = f"u_fixed_{cat}_{key}", f"u_plastic_{cat}_{key}"
                        if k_f not in h:
                            h[k_f] = deque(maxlen=self.history_len)
                            h[k_p] = deque(maxlen=self.history_len)
                        h[k_f].append(m.u_fixed[cat][key])
                        h[k_p].append(m.u_plastic[cat][key])
            if hasattr(m, "w_fixed") and hasattr(m, "w_plastic"):
                for cat in m.w_fixed:
                    for key in m.w_fixed[cat]:
                        k_f, k_p = f"w_fixed_{cat}_{key}", f"w_plastic_{cat}_{key}"
                        if k_f not in h:
                            h[k_f] = deque(maxlen=self.history_len)
                            h[k_p] = deque(maxlen=self.history_len)
                        h[k_f].append(m.w_fixed[cat][key])
                        h[k_p].append(m.w_plastic[cat][key])

    def record_child_states(self):
        for c in self.children:
            if not c.is_alive():
                continue
            h = self.child_history[c.id]

            h["hunger"].append(c.hunger)
            h["warmth"].append(c.warmth)
            h["injury"].append(c.injury)
            # h["carried"].append(1 if c.is_carried else 0)

            # if c.nearest_threat_dist == float("inf"):
            #     h["threat_dist"].append(None)
            # else:
            #     h["threat_dist"].append(c.nearest_threat_dist)