import math
from core.entities import Food
from core.policies.deficit import IDEAL_VALUE, deficit_low

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
    """Mother agent - can move and interact with child, food, threats, and nest"""
    def __init__(self, x, y, grid_w, grid_h, energy, mother_id=None):
        super().__init__(x, y, grid_w, grid_h, energy, agent_type="mother")
        self.id = mother_id
        self.child = None  # Reference to carried child agent
        self.holding_food = False
        self.holding_child = False
        self.alive = True

        # Physiological states
        self.energy = energy
        self.fatigue = 0.0
        # self.injury = 0.0       # may not use

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
            self.energy -= 1.0
            self.fatigue += 2.0

        # elif acted:
        #     self.energy -= 1.0
        #     self.fatigue += 1.0
        
        else :
            # natural recovery
            self.fatigue = max(0.0, self.fatigue - 0.1)
            self.energy = max(0.0, self.energy - 0.1)

        # clamp 
        self.energy = max(0.0, self.energy)
        self.fatigue = min(100.0, self.fatigue)

        if self.energy <= 0.0:
            self.alive = False

    def update_psych_neuro(self, world):
        if not self.is_alive():
            self.alive = False
            return


        # Child Signal
        child_need = 0.0
        energy_def = (deficit_low(self.energy, IDEAL_VALUE['M_energy']))


        # Adjustable weights
        w_1 = [0.33, 0.33, 0.33]        # Weight for child need
        w_2 = [0.1, 0.01]   # OT rise
        w_3 = [0.01, 0.02]  # Bonding
        w_4 = [0.1, 0.1, 0.05, 0.01]    # CORT

        if self.child is not None and self.child.is_alive():
            # higher when child hunger high or warmth far from ideal or injury high
            child_need = (
                w_1[0] * (self.child.hunger ) +
                w_1[1] * (abs(self.child.warmth - 50.0) ) +
                w_1[2] * (self.child.injury )
            ) 

            # print(child_need)

            # closeness 100 when same cell, 0 when far
            dchild = self.distance_to(self.child.x, self.child.y, metric='octile')
            norm_dist = dchild / max(world.grid_w, world.grid_h)
            # print(child_need)
            self.closeness_child = max(0.0, 100.0 * (1.0 - norm_dist))

            # OT
            # OT rises when close & doing care
            if self.closeness_child >= 90:  # distance to child <= xx % 
                ot_increase = w_2[0] * self.closeness_child
                self.OT = max(0.0, min(100.0, self.OT + ot_increase))

            else :
                ot_decay = w_2[1] * self.OT 
                self.OT = max(0.0, min(100.0, self.OT - ot_decay))


            # Bonding 
            # OT strengthens bond; unmet need reduce it slightly
            self.bonding = max(0.0, min(100.0, self.bonding + w_3[0] * self.OT - w_3[1] * child_need))
            # print(self.OT , child_need)



            # Threat proximity
            threat_near = 0.0
            threat_radius = 10.0
            
            for t in world.threats:
                d = self.child.distance_to(t.x, t.y, metric='octile')
                # d = self.distance_to(t.x, t.y, metric='octile')
                if d <= threat_radius: 
                    threat_near = max(threat_near, (threat_radius-d)/threat_radius)
                # print(threat_near)
            
            # CORT 
            cort_increase = (
                w_4[0] * threat_near +
                w_4[1] * child_need + 
                w_4[2] * energy_def
                )
            
            # # Fear 
            self.fear_threat = max(0.0, min(100.0, self.fear_threat + threat_near))

        # Child Dead T_T
        else:
            child_dead_decay = 0.5
            self.OT = self.OT - child_dead_decay
            self.closeness_child = 0.0
            self.bonding = self.bonding - child_dead_decay

            cort_increase = (w_4[2] * energy_def)
        cort_reduce = w_4[3] * self.CORT
        self.CORT = max(0.0, min(100.0, self.CORT + cort_increase - cort_reduce))
        # print(energy_def, cort_increase)


        # self.stress = max(0.0, min(100.0, self.stress + (child_need + energy_def) + threat_near))


        # self.print_state()

    def print_state(self):

        print(f"\n[MOTHER {self.id}]")
        # print("Energy:", self.energy)
        # print("Fatigue:", self.fatigue)

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
            print("Child ID:", self.child.id)

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
        self.health = 100.0
        self.alive = True

    
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

    def update(self):
        if not self.is_alive():
            self.alive = False
            return
        
        # Hunger increase over time
        self.hunger = min(100.0, self.hunger + 1.0)

        if self.is_carried:
            self.warmth = min(50.0, self.warmth + 0.1) 
        else:
            # Warmth increase over time (North Polar ?)
            self.warmth = max(0.0, self.warmth - 0.5)

        # Injury : recover over time
        # self.injury = max(0.0, self.injury - 0.1)

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

    # def update(self):
    #     pass

    def print_state(self):

        print(f"\n[THREAT {self.id}]")
        print("Position:", (self.x, self.y))
        print("Energy:", self.energy)
        print("Patrol goal:", self.patrol_goal)
        print("timer:", self.patrol_timer)
        print("Alive:", self.alive)




