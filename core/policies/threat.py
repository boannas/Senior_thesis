# core/policies/threat.py
import random
from func.path_finding import astar
from core.sim.movement import in_bounds, best_step

def threat_policy_propose(world):
    
    proposals = {}

    occupied_now = {(t.x, t.y) for t in world.threats}
    threat_receive = world.mothers + world.children

    target_child = {}
    prev_pos = {t: (t.x, t.y) for t in world.threats}

    for threat in world.threats:
        perception_range = threat.perception_range
        _, agents_perceived = threat.scan_perception(threat_receive, perception_range)

        mothers_seen = agents_perceived[0] if len(agents_perceived) > 0 else []
        children_seen = agents_perceived[1] if len(agents_perceived) > 1 else []

        goal = None
        is_fleeing = False
        nx, ny = threat.x, threat.y  # default: stay

        # ----- Select motivation -----

        # PRIORITY 1: Flee from visible mothers
        if mothers_seen:
            is_fleeing = True
            nx, ny = flee_step(threat, mothers_seen, world, occupied_now)

        # PRIORITY 2: Chase closest visible child 
        if not is_fleeing and children_seen:
            children_seen.sort(key=lambda x: x[1])
            child, _dist = children_seen[0]

            if child is not None and child.is_alive() and (not child.is_carried):
                goal = (child.x, child.y)
                target_child[threat] = child

        # PRIORITY 3: Patrol if no visible target
        if not is_fleeing and goal is None:
            if (
                threat.patrol_goal is None
                or (threat.x, threat.y) == threat.patrol_goal
                or threat.patrol_timer <= 0
            ):
                threat.patrol_goal = pick_patrol_goal(world, threat)
                threat.patrol_timer = world.PATROL_TIMEOUT

            goal = threat.patrol_goal
            threat.patrol_timer -= 1

        # ----- A* only for chase / patrol (not fleeing) -----
        if not is_fleeing and goal is not None:
            blocked = {(t.x, t.y) for t in world.threats if t is not threat}
            blocked |= {(m.x, m.y) for m in world.mothers if m.is_alive()}

            path = astar(
                (threat.x, threat.y),
                goal,
                world.grid_w,
                world.grid_h,
                blocked,
                moves_8=True
            )

            if path is None:
                nx, ny = best_step(
                    threat, goal, occupied_now,
                    world.grid_w, world.grid_h, world.RANDOM_MOVES
                )
            elif len(path) == 1:
                nx, ny = threat.x, threat.y
            else:
                nx, ny = path[1]
                if (nx, ny) in occupied_now and (nx, ny) != (threat.x, threat.y):
                    nx, ny = best_step(
                        threat, goal, occupied_now,
                        world.grid_w, world.grid_h, world.RANDOM_MOVES
                    )

        proposals[threat] = (nx, ny)

    intents = {
        "prev_pos": prev_pos,
        "target_child": target_child,
    }
    return proposals, intents


def flee_step(threat, mothers_seen, world, occupied_now):
    """
    Greedy 1-step repulsion away from all visible mothers.
    No A* — picks whichever neighbor tile maximizes distance from mothers.
    """
    candidates = []

    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            nx = threat.x + dx
            ny = threat.y + dy

            if not (0 <= nx < world.grid_w and 0 <= ny < world.grid_h):
                continue

            if (nx, ny) in occupied_now and (nx, ny) != (threat.x, threat.y):
                continue

            score = sum(
                abs(nx - m.x) + abs(ny - m.y)
                for m, _ in mothers_seen
            )
            candidates.append((score, nx, ny))

    if not candidates:
        return threat.x, threat.y

    candidates.sort(reverse=True)
    best_score, nx, ny = candidates[0]

    # If no neighbor is better than current pos, stay
    current_score = sum(
        abs(threat.x - m.x) + abs(threat.y - m.y)
        for m, _ in mothers_seen
    )
    if best_score <= current_score:
        return threat.x, threat.y

    return nx, ny


def apply_threat_intents(world, intents):
    target_child = intents["target_child"]

    for threat in world.threats:
        child = target_child.get(threat)

        if child is None:
            continue

        if (
            child.is_alive()
            and (not child.is_carried)
            and (threat.x, threat.y) == (child.x, child.y)
        ):
            child.injury += float(random.randint(5, 10))


def pick_patrol_goal(world, threat, max_tries=30):
    for _ in range(max_tries):
        gx = random.randrange(world.grid_w)
        gy = random.randrange(world.grid_h)
        if (gx, gy) != (threat.x, threat.y):
            return (gx, gy)
    return (threat.x, threat.y)
