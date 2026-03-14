# =========================
# core/policies/threat.py
# =========================

import random
from func.path_finding import astar
from core.sim.movement import best_step
import numpy as np

random.seed(42)
np.random.seed(42)


FLEE_HOLD_TICKS = 3
FLEE_EXIT_DIST = 3.0   # must be safely away before leaving flee mode


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
        nx, ny = threat.x, threat.y  # default: stay

        nearest_mother_dist = min((dist for _, dist in mothers_seen), default=float("inf"))

        # ----------------------------
        # 1) MODE TRANSITIONS
        # ----------------------------

        # Mother seen -> immediately enter flee
        if mothers_seen:
            threat.mode = "flee"
            threat.flee_timer = FLEE_HOLD_TICKS
            threat.last_seen_mothers = [(m.x, m.y) for m, _ in mothers_seen]

            # important: clear old patrol goal so threat won't go right back into danger
            threat.patrol_goal = None

        # If currently fleeing and no mother visible, keep fleeing for a few ticks
        elif threat.mode == "flee":
            if threat.flee_timer > 0:
                threat.flee_timer -= 1
            else:
                # only leave flee if safely far enough from last known mothers
                if is_safe_from_last_seen_mothers(threat, FLEE_EXIT_DIST):
                    if has_valid_visible_child(children_seen):
                        threat.mode = "chase"
                    else:
                        threat.mode = "patrol"
                        threat.patrol_goal = pick_safe_patrol_goal(
                            world, threat, threat.last_seen_mothers
                        )
                        threat.patrol_timer = world.PATROL_TIMEOUT

        # If not fleeing, visible child -> chase
        elif has_valid_visible_child(children_seen):
            threat.mode = "chase"

        # Otherwise patrol
        else:
            threat.mode = "patrol"

        # ----------------------------
        # 2) EXECUTE CURRENT MODE
        # ----------------------------

        if threat.mode == "flee":
            # use visible mothers if available, else use remembered mother positions
            flee_sources = mothers_seen_to_xy(mothers_seen)
            if not flee_sources:
                flee_sources = threat.last_seen_mothers

            nx, ny = flee_step_from_positions(threat, flee_sources, world, occupied_now)

        elif threat.mode == "chase":
            children_seen.sort(key=lambda x: x[1])
            child, _dist = children_seen[0]

            if child is not None and child.is_alive() and (not child.is_carried):
                goal = (child.x, child.y)
                target_child[threat] = child
            else:
                threat.mode = "patrol"

        if threat.mode == "patrol":
            if (
                threat.patrol_goal is None
                or (threat.x, threat.y) == threat.patrol_goal
                or threat.patrol_timer <= 0
            ):
                # if there is remembered danger, bias patrol away from it
                if threat.last_seen_mothers:
                    threat.patrol_goal = pick_safe_patrol_goal(
                        world, threat, threat.last_seen_mothers
                    )
                else:
                    threat.patrol_goal = pick_patrol_goal(world, threat)

                threat.patrol_timer = world.PATROL_TIMEOUT

            goal = threat.patrol_goal
            threat.patrol_timer -= 1

        # ----------------------------
        # 3) PATH PLANNING FOR CHASE/PATROL
        # ----------------------------
        if threat.mode in ("chase", "patrol") and goal is not None:
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


def has_valid_visible_child(children_seen):
    if not children_seen:
        return False
    child, _ = min(children_seen, key=lambda x: x[1])
    return child is not None and child.is_alive() and (not child.is_carried)


def mothers_seen_to_xy(mothers_seen):
    return [(m.x, m.y) for m, _ in mothers_seen]


def is_safe_from_last_seen_mothers(threat, safe_dist):
    """
    If no remembered mothers, consider safe.
    Uses Manhattan distance.
    """
    if not threat.last_seen_mothers:
        return True

    nearest = min(
        abs(threat.x - mx) + abs(threat.y - my)
        for mx, my in threat.last_seen_mothers
    )
    return nearest > safe_dist


def flee_step_from_positions(threat, mother_positions, world, occupied_now):
    """
    Greedy 1-step repulsion away from all given mother positions.
    Chooses neighbor maximizing total Manhattan distance from mothers.
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
                abs(nx - mx) + abs(ny - my)
                for mx, my in mother_positions
            )
            candidates.append((score, nx, ny))

    if not candidates:
        return threat.x, threat.y

    candidates.sort(reverse=True)
    best_score, nx, ny = candidates[0]

    current_score = sum(
        abs(threat.x - mx) + abs(threat.y - my)
        for mx, my in mother_positions
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


def pick_safe_patrol_goal(world, threat, mother_positions, max_tries=40):
    """
    Pick a patrol goal biased away from last seen mothers.
    """
    if not mother_positions:
        return pick_patrol_goal(world, threat, max_tries=max_tries)

    best_goal = (threat.x, threat.y)
    best_score = -1

    for _ in range(max_tries):
        gx = random.randrange(world.grid_w)
        gy = random.randrange(world.grid_h)

        if (gx, gy) == (threat.x, threat.y):
            continue

        score = sum(
            abs(gx - mx) + abs(gy - my)
            for mx, my in mother_positions
        )

        if score > best_score:
            best_score = score
            best_goal = (gx, gy)

    return best_goal