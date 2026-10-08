"""Geometry-based soccer policy using only the public observation."""
import heapq
import math
import random


SQ = math.sqrt(0.5)
VECTORS = {"STAY": (0, 0), "UP": (0, 1), "UP_RIGHT": (SQ, SQ),
           "RIGHT": (1, 0), "DOWN_RIGHT": (SQ, -SQ), "DOWN": (0, -1),
           "DOWN_LEFT": (-SQ, -SQ), "LEFT": (-1, 0), "UP_LEFT": (-SQ, SQ)}
KICK_DIRECTIONS = [name for name in VECTORS if name != "STAY"]

# Ball physics of the official simulator. Values the observation provides
# (field size, player radius/speed, goal width, ball radius) are always read
# from it; these are fallbacks and the parts the observation does not expose.
BALL_RADIUS = 1.5          # fallback when field has no "ball_radius"
BALL_SPEED = 8.0           # distance the ball travels per tick
DISTANCE_PER_POWER = 32.0  # kick distance per unit of kick power
SUBSTEP = 0.675            # maximum ball substep length used for collisions
DEFAULT_POWER = 3          # assumed opponent kick power before any is seen
EPS = 1e-6


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def hits(point, rectangle, radius):
    x, y, w, h = rectangle
    dx = point[0] - min(max(point[0], x), x + w)
    dy = point[1] - min(max(point[1], y), y + h)
    return dx * dx + dy * dy < radius * radius


def crosses(a, b, rectangle, padding):
    """Segment versus expanded rectangle, used by the route planner."""
    x, y, w, h = rectangle
    low, high = 0.0, 1.0
    for origin, delta, left, right in (
        (a[0], b[0] - a[0], x - padding, x + w + padding),
        (a[1], b[1] - a[1], y - padding, y + h + padding),
    ):
        if abs(delta) < 1e-10:
            if not left < origin < right:
                return False
        else:
            t1, t2 = (left - origin) / delta, (right - origin) / delta
            low, high = max(low, min(t1, t2)), min(high, max(t1, t2))
            if low >= high:
                return False
    return high > 0 and low < 1


def ticks(t):
    """Whole ticks needed to reach a trajectory point timed at t."""
    return math.ceil(t - 1e-9)


class Policy:
    def __init__(self):
        self.layout = None
        self.corners = []
        self.edges = []
        self.random = random.Random(0)
        self.max_power = DEFAULT_POWER
        self.reset_caches()

    def reset_caches(self):
        # Shortest-path results only depend on positions and the obstacle
        # layout, so they are reused within a tick and dropped afterwards.
        self.reach_cache = {}
        self.route_cache = {}
        self.shot_cache = {}

    # ------------------------------------------------------------------
    # Observation helpers

    def kick_options(self, observation):
        """Legal kick directions and powers, tolerating a missing kick entry."""
        kick = observation.get("action_space", {}).get("kick") or {}
        directions = [d for d in kick.get("direction", KICK_DIRECTIONS) if d in VECTORS and d != "STAY"]
        powers = [p for p in kick.get("power", []) if isinstance(p, (int, float)) and p > 0]
        if powers:
            self.max_power = max(powers)
        return directions or list(KICK_DIRECTIONS), powers

    def move_options(self, observation):
        moves = observation.get("action_space", {}).get("move") or list(VECTORS)
        return [m for m in moves if m in VECTORS] or ["STAY"]

    # ------------------------------------------------------------------
    # Geometry

    def valid(self, point):
        x, y = point
        r = self.radius
        return (r <= x <= self.width - r and r <= y <= self.height - r
                and not any(hits(point, o, r) for o in self.obstacles))

    def clear(self, a, b):
        # A player's collision shape is a circle. Expanding obstacle rectangles
        # into squares incorrectly traps legal positions near their corners.
        r = self.radius
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx + dy*dy
        for o in self.obstacles:
            if not crosses(a, b, o, r):
                continue
            x, y, w, h = o
            if (crosses(a, b, (x-r, y, w+2*r, h), 0)
                    or crosses(a, b, (x, y-r, w, h+2*r), 0)):
                return False
            for cx, cy in ((x, y), (x+w, y), (x, y+h), (x+w, y+h)):
                t = min(1, max(0, ((cx-a[0])*dx + (cy-a[1])*dy)/(length2 or 1)))
                if (a[0]+t*dx-cx)**2 + (a[1]+t*dy-cy)**2 < r*r - 1e-9:
                    return False
        return True

    def prepare(self, state):
        field = state["field"]
        self.width, self.height = field["width"], field["height"]
        self.radius, self.speed = field["player_radius"], field["player_speed"]
        self.goal_width = field["goal_width"]
        self.ball_radius = field.get("ball_radius", BALL_RADIUS)
        # Distance between player and ball centres at which the ball is touched.
        self.control = self.radius + self.ball_radius
        self.obstacles = [(o["x"], o["y"], o["width"], o["height"])
                          for o in state.get("obstacles", [])]
        # Obstacles grown by the ball radius: a cheap box test that rules out
        # most substeps before the exact circle-versus-rectangle check.
        b = self.ball_radius
        self.ball_boxes = [(x, y, w, h, x-b, y-b, x+w+b, y+h+b)
                           for x, y, w, h in self.obstacles]
        layout = (self.width, self.height, self.radius, tuple(self.obstacles))
        if layout == self.layout:
            return
        self.layout = layout
        p = self.radius + 0.5
        self.corners = [point for x, y, w, h in self.obstacles
                        for point in ((x-p, y-p), (x-p, y+h+p),
                                      (x+w+p, y-p), (x+w+p, y+h+p))
                        if self.valid(point)]
        self.edges = [[] for _ in self.corners]
        for i, a in enumerate(self.corners):
            for j in range(i):
                b = self.corners[j]
                if self.clear(a, b):
                    length = distance(a, b)
                    self.edges[i].append((j, length))
                    self.edges[j].append((i, length))

    def reach(self, start):
        """Dijkstra from start over obstacle corners: (costs, first waypoint)."""
        cached = self.reach_cache.get(start)
        if cached is not None:
            return cached
        costs = [float("inf")] * len(self.corners)
        first = list(self.corners)
        queue = []
        for i, p in enumerate(self.corners):
            if self.clear(start, p):
                costs[i] = distance(start, p)
                heapq.heappush(queue, (costs[i], i))
        while queue:
            cost, i = heapq.heappop(queue)
            if cost != costs[i]:
                continue
            for j, length in self.edges[i]:
                if cost + length < costs[j]:
                    costs[j], first[j] = cost + length, first[i]
                    heapq.heappush(queue, (costs[j], j))
        order = sorted((c, i) for i, c in enumerate(costs) if math.isfinite(c))
        result = (costs, first, order)
        self.reach_cache[start] = result
        return result

    def route(self, start, target):
        """Shortest obstacle-avoiding length and the next waypoint to head for."""
        key = (start, target)
        cached = self.route_cache.get(key)
        if cached is not None:
            return cached
        if self.clear(start, target):
            result = (distance(start, target), target)
        else:
            _, first, order = self.reach(start)
            best, waypoint = float("inf"), target
            for cost, i in order:
                if cost >= best:
                    break
                p = self.corners[i]
                total = cost + distance(p, target)
                if total < best and self.clear(p, target):
                    best, waypoint = total, first[i]
            result = ((best, waypoint) if math.isfinite(best)
                      else (distance(start, target) + 50, target))
        self.route_cache[key] = result
        return result

    def reach_map(self, start):
        """Shortest distances to obstacle corners, shared across shot checks."""
        return self.reach(start)[0]

    def reach_distance(self, start, target, costs, control=None):
        # Reaching the edge of the interception circle is sufficient; the ball
        # may itself lie closer to an obstacle than a legal player center can.
        control = self.control if control is None else control
        def remaining(origin):
            length = distance(origin, target)
            fraction = max(0, length-control) / (length or 1)
            end = (origin[0] + fraction*(target[0]-origin[0]),
                   origin[1] + fraction*(target[1]-origin[1]))
            return max(0, length-control) if self.valid(end) and self.clear(origin, end) else float("inf")
        best = remaining(start)
        if math.isfinite(best):
            return best
        return min((cost + remaining(p) for cost, p in zip(costs, self.corners)
                    if math.isfinite(cost)), default=float("inf"))

    def moves(self, start, allowed):
        result = []
        for name in allowed:
            vx, vy = VECTORS[name]
            p = (start[0] + vx * self.speed, start[1] + vy * self.speed)
            if self.valid(p):
                result.append((name, p))
        return result or [("STAY", start)]

    def follow(self, start, target, choices):
        # Evaluate the remaining route after each legal discrete movement.
        # A corner waypoint can lie between grid-sized steps; simply choosing
        # the closest move to that waypoint can select STAY forever.
        return min(choices, key=lambda item: (self.route(item[1], target)[0],
                                             distance(item[1], target)))[0]

    # ------------------------------------------------------------------
    # Ball physics

    def trajectory(self, start, velocity, remaining):
        """Match the official ball substeps, wall rebounds and obstacle rebounds."""
        x, y = start
        vx, vy = velocity
        b = self.ball_radius
        result = []
        elapsed = 0.0
        while remaining > 1e-8:
            travel = min(BALL_SPEED, remaining)
            count = max(1, math.ceil(travel / SUBSTEP))
            step = travel / count
            speed = math.hypot(vx, vy)
            if speed < 1e-9:
                break
            for _ in range(count):
                nx, ny = x + vx / speed * step, y + vy / speed * step
                elapsed += step / BALL_SPEED
                goal = 0
                if (self.width-self.goal_width)/2 <= nx <= (self.width+self.goal_width)/2:
                    goal = 1 if ny + b >= self.height else -1 if ny - b <= 0 else 0
                if goal:
                    result.append(((nx, ny), elapsed, goal))
                    return result
                if nx < b or nx > self.width - b:
                    vx = -vx
                    nx = min(max(nx, b), self.width - b)
                if ny < b or ny > self.height - b:
                    vy = -vy
                    ny = min(max(ny, b), self.height - b)
                for ox, oy, w, h, x0, y0, x1, y1 in self.ball_boxes:
                    if nx <= x0 or nx >= x1 or ny <= y0 or ny >= y1:
                        continue
                    if not hits((nx, ny), (ox, oy, w, h), b):
                        continue
                    crossed_x = x <= ox - b or x >= ox + w + b
                    crossed_y = y <= oy - b or y >= oy + h + b
                    if crossed_x:
                        vx = -vx
                    if crossed_y:
                        vy = -vy
                    if not crossed_x and not crossed_y:
                        vx, vy = -vx, -vy
                    nx, ny = x + vx / speed * 0.05, y + vy / speed * 0.05
                    break
                x, y = nx, ny
                remaining -= step
                result.append(((x, y), elapsed, 0))
        return result

    def shot(self, position, direction, power):
        # The same shot is evaluated by several decisions in one tick (goal
        # shot, release kick, mixed shot), so it is simulated only once.
        # Callers must treat the returned list as read-only.
        key = (position, direction, power)
        cached = self.shot_cache.get(key)
        if cached is not None:
            return cached
        vx, vy = VECTORS[direction]
        offset = self.control + 0.05
        origin = (position[0] + vx * offset, position[1] + vy * offset)
        path = self.trajectory(origin, (vx*BALL_SPEED, vy*BALL_SPEED),
                               DISTANCE_PER_POWER*power)
        self.shot_cache[key] = path
        return path

    def truncate(self, path, power):
        """Prefix of a full-power path that a weaker kick of `power` covers.

        Uses travelled distance (elapsed time * ball speed) rather than a
        point count, so non-integer powers such as 1.5 work too."""
        limit = DISTANCE_PER_POWER * power / BALL_SPEED + EPS
        return [point for point in path if point[1] <= limit]

    # ------------------------------------------------------------------
    # Decision making

    def choose_action(self, observation):
        state = observation["state"]
        self.reset_caches()
        self.prepare(state)
        own, other = observation["player_id"], observation["opponent_id"]
        if state.get("iteration") == 0:
            seed = state.get("seed") or 0
            self.random.seed(seed * 1009 + (17 if own == "player_1" else 31))
        me = tuple(state["players"][own][k] for k in ("x", "y"))
        enemy = tuple(state["players"][other][k] for k in ("x", "y"))
        ball = state["ball"]
        sign = 1 if observation["attack_direction"] == "UP" else -1
        choices = self.moves(me, self.move_options(observation))
        self.kick_options(observation)  # remembers the max power when visible
        if ball.get("possession") == own:
            return self.attack(observation, me, enemy, choices, sign)
        if ball.get("status") == "moving":
            velocity = tuple(ball["velocity"][k] for k in ("x", "y"))
            path = self.trajectory((ball["x"], ball["y"]), velocity,
                                   ball["remaining_kick_distance"])
            # Select the first intercept we can reach, including obstacle detours.
            first = [(q, t) for q, t, goal in path if t <= 1 + 1e-8 and not goal]
            intercepts = [(t, move) for move, p in choices for q, t in first
                          if distance(p, q) <= self.control]
            if intercepts:
                return {"move": min(intercepts)[1]}
            targets = [point for point in path[2::3] + path[-1:] if not point[2]]
            target = (ball["x"], ball["y"])
            costs = self.reach_map(me)
            for p, t, goal in targets:
                target = p
                if t <= 1 + 1e-8:
                    continue
                length = self.reach_distance(me, p, costs)
                if length <= self.speed * ticks(t):
                    break
            return {"move": self.follow(me, target, choices)}
        if ball.get("possession") == other:
            return self.defend(observation, me, enemy, choices, sign)
        target = (ball["x"], ball["y"])
        return {"move": self.follow(me, target, choices)}

    def defend(self, observation, me, enemy, choices, sign):
        """Close down the ball only when the scoring lanes remain covered."""
        # The opponent's kick options are taken from our own action space when
        # present; otherwise all eight directions at the strongest power seen.
        directions, _ = self.kick_options(observation)
        threats = []
        for _, p in self.moves(enemy, self.move_options(observation)):
            for direction in directions:
                path = self.shot(p, direction, self.max_power)
                if (path and path[-1][2] == -sign
                        and not any(distance(q, p) <= self.control for q, t, goal in path if t <= 1 and not goal)):
                    threats.append([(q, max(0, ticks(t)-1))
                                    for q, t, goal in path if not goal])
        target = (enemy[0] + (-3 if enemy[0] > self.width/2 else 3),
                  min(self.height-self.radius, max(self.radius, enemy[1] - sign*3)))
        if not threats:
            return {"move": self.follow(me, target, choices)}
        def value(item):
            move, p = item
            risks = [max(0, min(distance(p, q)-self.control-self.speed*turns for q, turns in path))
                     for path in threats if path]
            return 10*max(risks, default=0) + 3*sum(risks)/max(1, len(risks)) + 0.05*self.route(p, target)[0]
        return {"move": min(choices, key=value)[0]}

    def mixed_shot(self, observation, me, enemy, choices, sign):
        """Mix shot lanes when no single next defender move covers all of them."""
        replies = self.moves(enemy, self.move_options(observation))
        actions, matrix = [], []
        directions, powers = self.kick_options(observation)
        if not powers:
            return None
        power = max(powers)
        for move, p in choices:
            for direction in directions:
                path = self.shot(p, direction, power)
                if not path or path[-1][2] != sign or path[-1][1] > 6:
                    continue
                payoffs = []
                for _, ep in replies:
                    origin, defender = p, ep
                    if distance(p, ep) < 2*self.radius:
                        # Common simultaneous-contact resolution. For awkward
                        # wall/obstacle contacts, conservatively mark it unsafe.
                        length = distance(me, enemy) or 1
                        ux, uy = (me[0]-enemy[0])/length, (me[1]-enemy[1])/length
                        mx, my = (p[0]+ep[0])/2, (p[1]+ep[1])/2
                        origin = (mx+ux*self.radius, my+uy*self.radius)
                        defender = (mx-ux*self.radius, my-uy*self.radius)
                        if not self.valid(origin) or not self.valid(defender):
                            payoffs.append(0)
                            continue
                    actual = path if origin == p else self.shot(origin, direction, power)
                    safe = (actual and actual[-1][2] == sign
                            and not any(distance(q, origin) <= self.control for q, _, goal in actual if not goal)
                            and all(distance(q, defender) > self.control + self.speed*max(0, ticks(t)-1)
                                    for q, t, goal in actual if not goal))
                    payoffs.append(int(bool(safe)))
                if any(payoffs):
                    actions.append({"move": move, "kick": {"direction": direction, "power": power}})
                    matrix.append(payoffs)
        if not matrix:
            return None
        # Fictitious play estimates a mixed strategy. Check its actual minimum
        # column payoff before using it, rather than trusting convergence.
        rows = [0] * len(matrix)
        columns = [1] * len(replies)
        for _ in range(96):
            i = max(range(len(matrix)), key=lambda r: sum(v*c for v, c in zip(matrix[r], columns)))
            rows[i] += 1
            j = min(range(len(replies)), key=lambda c: sum(rows[r]*matrix[r][c] for r in range(len(matrix))))
            columns[j] += 1
        lower_bound = min(sum(rows[r]*matrix[r][c] for r in range(len(matrix))) / 96
                          for c in range(len(replies)))
        if lower_bound < 0.25:
            return None
        return self.random.choices(actions, weights=rows, k=1)[0]

    def attack(self, observation, me, enemy, choices, sign):
        ball = observation["state"]["ball"]
        directions, powers = self.kick_options(observation)
        steps = ball.get("possession_steps", 0)
        best_goal = None
        best_release = None
        for move, p in choices if powers else ():
            for direction in directions:
                path = self.shot(p, direction, max(powers))
                if not path:
                    continue
                # The defender can change direction every turn. Use its entire
                # reachable radius, rather than assuming one particular policy.
                margin = min((distance(q, enemy) - self.speed * ticks(t) - self.control
                              for q, t, goal in path if not goal), default=100.0)
                # Only the launch tick has a stationary kicker. Later rebounds
                # can be chased and deliberately recovered.
                self_hit = any(distance(q, p) <= self.control for q, t, goal in path
                               if t <= 1+1e-9 and not goal)
                endpoint, time, goal = path[-1]
                if self_hit:
                    continue
                if goal == sign:
                    value = margin - 0.1*time
                    if best_goal is None or value > best_goal[0]:
                        best_goal = (value, move, direction, max(powers))
                for power in powers if steps >= 3 else ():
                    short = self.truncate(path, power)
                    if not short or short[-1][2]:
                        continue
                    endpoint, time, _ = short[-1]
                    # Predict the first recoverable point, including rebounds.
                    recover_time = max(ticks(time),
                                       1 + max(0, self.route(p, endpoint)[0]-5)/self.speed)
                    endpoint_time = time
                    safety = float("inf")
                    for q, t, _ in short:
                        tick = ticks(t)
                        safety = min(safety, distance(q, enemy)-self.control-self.speed*tick)
                        if tick > 1 and distance(p, q) <= self.control+self.speed*(tick-1) and self.clear(p, q):
                            endpoint, recover_time, endpoint_time = q, tick, t
                            break
                    enemy_time = max(ticks(endpoint_time),
                                     max(0, self.route(enemy, endpoint)[0]-5)/self.speed)
                    lead = self.speed*(enemy_time-recover_time)
                    progress = sign*(endpoint[1]-p[1])
                    value = progress + 0.5*min(24, lead) - recover_time - 4*max(0, -safety)
                    if best_release is None or value > best_release[0]:
                        best_release = (value, move, direction, power)
        if best_goal is not None and best_goal[0] > -3:
            _, move, direction, power = best_goal
            return {"move": move, "kick": {"direction": direction, "power": power}}
        if powers and steps >= 3 and distance(me, enemy) < 40:
            mixed = self.mixed_shot(observation, me, enemy, choices, sign)
            if mixed:
                return mixed

        # Carry to a clear shooting lane. Routes escape obstacle corners rather
        # than repeatedly requesting a blocked forward move.
        goal_y = self.height - 16 if sign > 0 else 16
        targets = [(self.width/2 + offset, goal_y) for offset in (-12, 0, 12)]
        goal_distance = self.height - me[1] if sign > 0 else me[1]
        if goal_distance > 40 and 0 < sign * (enemy[1] - me[1]) < 45 and abs(enemy[0] - me[0]) < 24:
            targets = [(x, min(self.height-8, max(8, me[1] + sign*28)))
                       for x in (max(8, enemy[0]-24), min(self.width-8, enemy[0]+24))]
        routes = [(self.route(me, target), target) for target in targets]
        (_, waypoint), carry_target = min(routes, key=lambda item: item[0][0])
        def carry_value(item):
            move, p = item
            separation = distance(p, enemy)
            danger = max(0, 15 - separation)
            return -self.route(p, carry_target)[0] - danger
        move, p = max(choices, key=carry_value)
        urgent = steps >= 8 or (steps >= 3 and distance(p, enemy) < 11)
        if urgent:
            release = best_goal if best_goal is not None and best_goal[0] > -5 else best_release
            if release:
                _, move, direction, power = release
                return {"move": move, "kick": {"direction": direction, "power": power}}
        return {"move": move}
