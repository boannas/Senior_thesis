# core/sim/movement.py

def in_bounds(x, y, grid_w, grid_h):
    return 0 <= x < grid_w and 0 <= y < grid_h


def best_step(agent, goal_xy, occupied_now, grid_w, grid_h, random_moves):
    gx, gy = goal_xy

    candidates = [(agent.x, agent.y)]
    for dx, dy in random_moves:
        nx, ny = agent.x + dx, agent.y + dy
        if 0 <= nx < grid_w and 0 <= ny < grid_h:
            candidates.append((nx, ny))

    def score(cell):
        nx, ny = cell

        # occupied penalty
        if (nx, ny) in occupied_now and (nx, ny) != (agent.x, agent.y):
            return 10**9

        # distance-to-goal
        s = abs(nx - gx) + abs(ny - gy)

        # (optional) spacing logic - your current code adds 0, so it does nothing
        # keeping structure here in case you later add a penalty
        for (ox, oy) in occupied_now:
            if (ox, oy) == (agent.x, agent.y):
                continue
            if abs(nx - ox) + abs(ny - oy) == 1:
                s += 0

        # discourage staying
        if (nx, ny) == (agent.x, agent.y):
            s += 10**9

        return s

    return min(candidates, key=score)


def resolve_and_apply_moves(agents, proposals, grid_w, grid_h):
    current = {a: (a.x, a.y) for a in agents}

    # def prio(a):
    #     v = getattr(a, "id", None)
    #     if v is None:
    #         v = getattr(a, "mother_id", None)
    #     if v is None:
    #         v = getattr(a, "child_id", None)
    #     if v is None:
    #         return id(a)
    #     return v

    def prio(a):
        """
        Return a comparable priority key for deterministic tie-breaking.
        Always returns a tuple of ints/strings so comparisons never mix int vs str.
        Lower key = higher priority.
        """
        # Prefer explicit IDs (mother_id/child_id/id)
        v = getattr(a, "id", None)
        if v is None:
            v = getattr(a, "mother_id", None)
        if v is None:
            v = getattr(a, "child_id", None)

        if v is None:
            # no explicit id -> use object id (int)
            return (3, id(a))  # type rank 3 = fallback
        else:
            # normalize to string so it's always comparable
            return (0, str(v))  # type rank 0 = explicit id

    dest_map = {}
    for a, dest in proposals.items():
        dest_map.setdefault(dest, []).append(a)

    blocked = set()

    # 1) destination conflicts
    for dest, claimers in dest_map.items():
        if len(claimers) > 1:
            winner = min(claimers, key=prio)
            for a in claimers:
                if a is not winner:
                    blocked.add(a)

    # 2) swap conflicts
    for a, dest_a in proposals.items():
        if a in blocked:
            continue
        for b, dest_b in proposals.items():
            if b is a or b in blocked:
                continue
            if dest_a == current[b] and dest_b == current[a]:
                winner = min([a, b], key=prio)
                loser = b if winner is a else a
                blocked.add(loser)

    # 3) apply moves
    occupied = set(current.values())
    for a, (nx, ny) in proposals.items():
        if a in blocked:
            continue

        nx = max(0, min(grid_w - 1, nx))
        ny = max(0, min(grid_h - 1, ny))

        if (nx, ny) in occupied and (nx, ny) != current[a]:
            continue

        occupied.discard(current[a])
        a.x, a.y = nx, ny
        occupied.add((nx, ny))