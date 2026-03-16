import math
import copy
from core.entities import Food
from core.policies.deficit import IDEAL_VALUE, deficit_low
import random
import numpy as np

random.seed(42)
np.random.seed(42)
rng = np.random.default_rng(42)





# self.w = {
#     "child_need": {
#         "hunger": 0.4,
#         "warmth": 0.2,
#         "injury": 0.4,
#     },
#     "ot": {
#         "closeness_gain": 0.05,
#         "decay": 0.02,
#     },
#     "bonding": {
#         "ot_gain": 0.03,
#         "child_need_decay": 0.02,
#         "child_absent_decay": 0.01,
#     },
#     "cort": {
#         "threat_gain": 0.3,
#         "child_need_gain": 0.4,
#         "energy_deficit_gain": 0.3,
#         "decay": 0.05,
#     },
#     "stress": {
#         "cort_gain": 0.4,
#         "fear_gain": 0.3,
#         "child_need_gain": 0.3,
#         "decay": 0.05,
#     },
#     "fear": {
#         "threat_gain": 0.3,
#         "decay": 0.05,
#     }
# }



MOVED_cost = {
    "Energy": -1.0,
    "Fatigue": 2.0
}

ACTED_cost = {
    "Energy": -0.1,
    "Fatigue": 0.1
}

RECOVERY_rate = {
    "Energy": -0.1, # agent's hunger increase
    "Fatigue": -0.1 # but, fatigue reduce
}

class Agent:
    """Base class for agents (mother and child)"""
    def __init__(self, x, y, grid_w, grid_h, energy, agent_type="mother"):
        # Initialize agent position and type
        self.x, self.y = x, y 
        self.grid_w = grid_w 
        self.grid_h = grid_h 
        self.agent_type = agent_type  # "mother" or "child"

        # --- Survival & Resources ---
        self.age = 0.0              # agent age
        self.energy = energy        
        self.alive = True

    def is_alive(self):
        if not self.alive:
            return False
        if self.energy < 0.0:
            return False
        return True
    
    def distance_to(self, other_x, other_y, metric="octile"):
        dx = abs(self.x - other_x)
        dy = abs(self.y - other_y)

        if metric == "manhattan":
            return dx + dy
        elif metric == "chebyshev":  # max metric
            return max(dx, dy)
        elif metric == "euclidean":
            return math.hypot(dx, dy)
        elif metric == "octile":
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
        else:
            raise ValueError("Unknown metric")

    def scan_perception(self, entities, perception_range):
        """Scan for entities within perception range"""
        food_perceived = []
        mother = []
        child = []
        threat = []
        agents_perceived = []
        
        for e in entities:
            if e is self:   # Skip self
                continue
            
            # compute distance & in range of perception 
            dist = self.distance_to(e.x, e.y, metric="octile")
            if dist > perception_range:
                continue

            # Food
            if isinstance(e, Food):
                if getattr(e, "collected", False):
                    continue
                food_perceived.append((e, dist))
                continue

            # Agents (other mothers, child, threats)
            if isinstance(e, Agent):

                if isinstance(e, MotherAgent):
                    mother.append((e, dist))
                elif isinstance(e, ChildAgent) and e.is_alive():
                    child.append((e, dist))
                elif isinstance(e, ThreatAgent):
                    threat.append((e, dist))
                else:
                    pass
                continue

        agents_perceived = [mother, child, threat]        
        return food_perceived, agents_perceived
    
    def step_towards(self, target_x, target_y):
        """
        Step toward to the target 
        """
        if not self.is_alive():
            return (0, 0)

        dx = 0 if target_x == self.x else (1 if target_x > self.x else -1)
        dy = 0 if target_y == self.y else (1 if target_y > self.y else -1)
        return (dx, dy)

class MotherAgent(Agent):
    """Mother agent - can move and interact with child, food, threats"""
    def __init__(self, x, y, grid_w, grid_h, energy, mother_id=None):
        super().__init__(x, y, grid_w, grid_h, energy, agent_type="mother")
        self.id = mother_id
        self.child = None  # Reference to carried child agent
        self.holding_food = False
        self.holding_child = False
        self.alive = True
        self.food_inventory = 0

        # Physiological states
        self.energy = energy
        self.fatigue = 0.0

        # Psychological states
        self.bonding = 100.00
        self.fear_threat = 0.0
        self.stress = 0.0
        self.closeness_child = 0.0

        # Hormone 
        self.OT = 0.0
        self.CORT = 0.0
        
        # Motivation
        self.motivations = {
            "Forage":   0.0,
            "Care":     0.0,
            "Self":     0.0, 
            "Protect":  0.0
        }

        self.selected_motivation = "Self"

        # --- Fixed gene (inherited, never changes in lifetime) ---
        self.w_fixed = {
            "child_need": {"hunger": rng.uniform(0,1), "warmth": rng.uniform(0,1), "injury": rng.uniform(0,1)},
            "ot": {"closeness_gain": rng.uniform(0,1), "decay": rng.uniform(0,1)},
            "bonding": {"ot_gain": rng.uniform(0,1), "child_need_decay": rng.uniform(0,1), "child_absent_decay": rng.uniform(0,1)},
            "cort": {"threat_gain": rng.uniform(0,1), "child_need_gain": rng.uniform(0,1), "energy_deficit_gain": rng.uniform(0,1), "decay": rng.uniform(0,1)},
            "stress": {"cort_gain": rng.uniform(0,1), "fear_gain": rng.uniform(0,1), "child_need_gain": rng.uniform(0,1), "decay": rng.uniform(0,1)},
            "fear": {"threat_gain": rng.uniform(0,1), "decay": rng.uniform(0,1)},
        }
        self.u_fixed = {
            "forage": {"child_hunger": rng.uniform(0,1), "energy_deficit": rng.uniform(0,1), "low_fear": rng.uniform(0,1)},
            "care": {"child_warmth": rng.uniform(0,1), "closeness_deficit": rng.uniform(0,1), "bonding": rng.uniform(0,1)},
            "self": {"fatigue": rng.uniform(0,1), "fear": rng.uniform(0,1), "stress": rng.uniform(0,1)},
            "protect": {"child_injury": rng.uniform(0,1), "fear": rng.uniform(0,1), "closeness_deficit": rng.uniform(0,1), "bonding": rng.uniform(0,1)},
        }
        self.w_plastic = copy.deepcopy(self.w_fixed)
        self.u_plastic = copy.deepcopy(self.u_fixed)
        self.eta = 0.02

    @property
    def w(self):
        return {cat: {k: (self.w_fixed[cat][k] + self.w_plastic[cat][k]) / 2.0 for k in self.w_fixed[cat]} for cat in self.w_fixed}

    @property
    def u(self):
        return {cat: {k: (self.u_fixed[cat][k] + self.u_plastic[cat][k]) / 2.0 for k in self.u_fixed[cat]} for cat in self.u_fixed}

    def is_alive(self):
        if not self.alive:
            return False
        if self.energy < 0.0:
            return False
        return True
    
    def set_child(self, child_agent):
        """Set the child agent being carried by mother"""
        self.child = child_agent

    def picking_food(self, picking: bool):
        self.holding_food = picking

    def picking_child(self, picking: bool):
        self.holding_child = picking

    def update_physiology(self, moved=False, acted=False):
        if not self.is_alive():
            self.alive = False
            return
        
        # cost 
        if moved:
            self.energy += MOVED_cost["Energy"]
            self.fatigue += MOVED_cost["Fatigue"]

        elif acted:
            self.energy += ACTED_cost["Energy"]
            self.fatigue += ACTED_cost["Fatigue"]
        
        # natural recovery
        else :
            self.energy = max(0.0, self.energy + RECOVERY_rate["Energy"])   # Always hungry
            self.fatigue = max(0.0, self.fatigue + RECOVERY_rate["Fatigue"]) 

        # clamp 
        self.energy = max(0.0, min(100.0, self.energy))
        self.fatigue = max(0.0, min(100.0, self.fatigue))

        # DEAD -_-'
        if self.energy <= 0.0:
            self.alive = False

    def update_psych_neuro(self, world):
        if not self.is_alive():
            self.alive = False
            return

        # Child Signal
        child_need = 0.0
        energy_def = deficit_low(self.energy, IDEAL_VALUE['M_energy'])

        fear_increase = 0.0
        fear_decay = self.w["fear"]["decay"] * self.fear_threat

        ot_increase = 0.0
        ot_decay = self.w["ot"]["decay"]* self.OT

        cort_increase = 0.0
        cort_decay = self.w["cort"]["decay"] * self.CORT

        closeness_child = 0.0
        stress_decay = self.w["stress"]["decay"] * self.stress


        if self.child is not None and self.child.is_alive():


            dchild = self.distance_to(self.child.x, self.child.y, metric='octile')
            norm_dist = min(1.0, dchild / max(world.grid_w, world.grid_h))
            closeness_child = 100.0 * (1.0 - norm_dist)

            hunger_sig = self.child.hunger / 100.0
            warmth_sig = abs(self.child.warmth - 50.0) / 50.0
            injury_sig = self.child.injury / 100.0


            child_need_gain_sum = weight_sum(
                self.w["child_need"],
                ["hunger", "warmth", "injury"]
            )
            child_need = 100 * (
                self.w["child_need"]["hunger"] * hunger_sig +
                self.w["child_need"]["warmth"] * warmth_sig +
                self.w["child_need"]["injury"] * injury_sig
            ) / child_need_gain_sum

            bonding_decay = self.w["bonding"]["child_need_decay"] * child_need

            # OT
            if closeness_child >= 90:  # distance to child <= xx % 
                ot_increase = self.w["ot"]["closeness_gain"] * closeness_child

            # Threat proximity (reference with child)
            threat_near = 0.0
            threat_radius = 10.0

            for t in world.threats:
                d = self.child.distance_to(t.x, t.y, metric='octile')
                if d <= threat_radius: 
                    threat_near = max(threat_near, (threat_radius-d)/threat_radius)
            
            fear_increase = 100.0 * (self.w["fear"]["threat_gain"] * threat_near)

            # CORT 
            cort_gain_sum = weight_sum(
                self.w["cort"],
                ["threat_gain", "child_need_gain", "energy_deficit_gain"]
            )
            cort_increase = 100.0 * (
                self.w["cort"]["threat_gain"] * threat_near +
                self.w["cort"]["child_need_gain"] * (child_need/100.0) + 
                self.w["cort"]["energy_deficit_gain"] * (energy_def/100.0)
                ) / cort_gain_sum

            

        # Child Dead T_T
        else:
            closeness_child = 0.0
            bonding_decay = self.w["bonding"]["child_absent_decay"] * self.bonding

            cort_gain_sum = weight_sum(
                self.w["cort"],
                ["threat_gain", "child_need_gain", "energy_deficit_gain"]
            )
            cort_increase = 100.0 * (
                self.w["cort"]["energy_deficit_gain"] * (energy_def/100.0)
            ) / cort_gain_sum

        # Compute the psych states
        stress_gain_sum = weight_sum(
            self.w["stress"],
            ["cort_gain", "fear_gain", 'child_need_gain']
        )
        stress_increase = (
            self.w["stress"]["cort_gain"] * self.CORT +
            self.w["stress"]["fear_gain"] * self.fear_threat + 
            self.w["stress"]["child_need_gain"] * child_need
        ) / stress_gain_sum


        self.closeness_child = max(0.0, min(100.0, closeness_child))

        self.OT = max(0.0, min(100.0, self.OT + ot_increase - ot_decay))
        
        self.bonding = max(0.0, min(100.0, 
            self.bonding + self.w["bonding"]["ot_gain"] * self.OT - bonding_decay
            ))
        
        self.fear_threat = max(0.0, min(100.0, 
            self.fear_threat + fear_increase - fear_decay
            ))
        
        self.CORT = max(0.0, min(100.0, 
            self.CORT + cort_increase - cort_decay
            ))
        
        self.stress = max(0.0, min(100,
            self.stress + stress_increase - stress_decay
            ))


        # self.print_state()

    def print_state(self):

        print(f"\n[MOTHER {self.id}]")
        print("Energy:", self.energy)
        print("Fatigue:", self.fatigue)

        print("---Psychological---")
        print("Bonding:", self.bonding)
        print("Fear:", self.fear_threat)
        print("Stress:", self.stress)
        print("Closeness Child:", self.closeness_child)

        # print("---Hormones---")
        # print("Oxytocin:", self.OT)
        # print("Cortisol:", self.CORT)

        # print("---Resources---")
        # print("Holding Food:", self.holding_food)
        # print("Holding Child:", self.holding_child)

        if self.child:
            print("Child ID:", self.child.id, self.child.alive)

        print("Alive:", self.alive)

class ChildAgent(Agent):
    """Child agent - can be carried by mother"""
    def __init__(self, x, y, grid_w, grid_h, energy, child_id=None):
        super().__init__(x, y, grid_w, grid_h, energy, agent_type="child")
        self.id = child_id
        self.mother = None  # Reference to mother agent
        self.is_carried = False

        # Internal states
        self.hunger = 0.0       # 0:full, 100:starving
        self.warmth = 50.0       # 0:freezing, 100:overheat 
        self.injury = 0.0       # 0:none, 100:lethat
        self.alive = True

        self.threat_recovery_radius = 5.0
        self.injury_recovery_rate = 0.1
        self.warmth_recovery_rate = 0.1

        self.threat_close = False
        self.nearest_threat_dist = float("inf")

    
    def is_alive(self): # Overide is alive
        if not self.alive:
            return False
        if self.hunger >= 100:
            return False
        if self.warmth <= 0 or self.warmth >= 100:
            return False
        if self.injury >= 100:
            return False

        return True

    def set_mother(self, mother_agent):
        """Set the mother agent carrying this child"""
        self.mother = mother_agent

    def set_carried(self, carried: bool):
        """Set whether the child is being carried by mother"""
        self.is_carried = carried

    def update(self, world):
        if not self.is_alive():
            self.alive = False
            return
        
        self.threat_close = False
        self.nearest_threat_dist = float("inf")

        # Hunger increase over time
        self.hunger = min(100.0, self.hunger + 1.0)

        if self.is_carried:
            self.warmth = min(50.0, self.warmth + self.warmth_recovery_rate) 
        else:
            # Warmth increase over time (North Polar ?)
            self.warmth = max(0.0, self.warmth - 0.5)

        # Injury: recover over time
        for t in world.threats:
            d = self.distance_to(t.x, t.y, metric="octile")
            self.nearest_threat_dist = min(self.nearest_threat_dist, d)
            if d <= self.threat_recovery_radius:
                self.threat_close = True
            
        # Recovery only when threats are not close
        if not self.threat_close:
            self.injury = max(0.0, self.injury - self.injury_recovery_rate)

        # Check Death after update states
        if not self.is_alive():
            self.alive = False

        # self.print_state()

    def print_state(self):

        print(f"\n[CHILD {self.id}]")
        print("Hunger:", self.hunger)
        print("Warmth:", self.warmth)
        print("Injury:", self.injury)
        print("Carried:", self.is_carried)
        # print(self.get_health())

        if self.mother:
            print("Mother ID:", self.mother.id)

        print("Alive:", self.alive)

class ThreatAgent(Agent):
    """Threat agent - can move and pose danger to mother and child"""
    def __init__(self, x, y, grid_w, grid_h, energy, threat_id=None):
        super().__init__(x, y, grid_w, grid_h, energy, agent_type="threat")
        self.id = threat_id
        self.patrol_goal = None
        self.patrol_timer = 0
        self.perception_range = 2
        self.energy = 100.0

        self.mode = "patrol"
        self.flee_timer = 0
        self.last_seen_mothers = []
        self.last_seen_memory_timer = 0  # ticks since last saw mother; clear memory after timeout

    def print_state(self):
        print(f"\n[THREAT {self.id}]")
        print("Position:", (self.x, self.y))
        print("Energy:", self.energy)
        print("Patrol goal:", self.patrol_goal)
        print("timer:", self.patrol_timer)
        print("Alive:", self.alive)




def weight_sum(group, keys):
    return sum(group[k] for k in keys)