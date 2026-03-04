# core/policies/threat.py
import random
from core.sim.movement import in_bounds
PATROL_TIMEOUT = 100

def threat_policy_propose(world):
    perception_range = 2
    proposals = {}

    threat_receive = world.mothers + world.children
    target_child = {}  # threat -> child
    prev_pos = {t: (t.x, t.y) for t in world.threats}

    for threat in world.threats:
        _, agents_perceived = threat.scan_perception(threat_receive, perception_range)

        mothers_seen = agents_perceived[0] if len(agents_perceived) > 0 else []
        children_seen = agents_perceived[1] if len(agents_perceived) > 1 else []

        # 1) chase closest visible child (if not carried)
        if children_seen:
            children_seen.sort(key=lambda t: t[1])
            child, _dist = children_seen[0]

            if child is not None and child.is_alive() and (not child.is_carried):
                dx, dy = threat.step_towards(child.x, child.y)
                nx, ny = threat.x + dx, threat.y + dy

                # keep in bounds
                if not in_bounds(nx, ny, world.grid_w, world.grid_h):
                    nx, ny = threat.x, threat.y

                proposals[threat] = (nx, ny)
                target_child[threat] = child
                continue

        # 2 Patrol
        if (threat.patrol_goal is None) or ((threat.x, threat.y) == threat.patrol_goal) or (threat.patrol_timer <= 0):
            threat.patrol_goal = pick_patrol_goal(world, threat)
            threat.patrol_timer = PATROL_TIMEOUT

        threat.patrol_timer -= 1
        gx, gy = threat.patrol_goal
        dx, dy = threat.step_towards(gx, gy)
        nx, ny = threat.x + dx, threat.y + dy

        print(threat.patrol_goal)
        if not in_bounds(nx, ny, world.grid_w, world.grid_h):
            nx, ny = threat.x, threat.y

        proposals[threat] = (nx, ny)

    intents = {
        "prev_pos": prev_pos,
        "target_child": target_child,
    }
    return proposals, intents


def apply_threat_intents(world, intents):
    target_child = intents.get("target_child", {})

    for threat in world.threats:
        child = target_child.get(threat)
        if child is None:
            continue

        if child.is_alive() and (not child.is_carried) and (threat.x, threat.y) == (child.x, child.y):
            child.energy -= random.randint(5, 10)
            print(child.energy)

def pick_patrol_goal(world, threat, max_tries=30):
    for _ in range(max_tries):
        gx = random.randrange(world.grid_w)
        gy = random.randrange(world.grid_h)
        if (gx, gy) != (threat.x, threat.y):
            return (gx, gy)

    return (threat.x, threat.y)

