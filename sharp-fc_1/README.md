# Sharp FC

Python 3.11+. No third-party dependencies, downloads, or model weights.

Run from this folder: `python -m team_bot.bot`.

## Protocol

Input and output follow the official newline-delimited JSON protocol.
For each `observation` message the bot reads its player ID, attack direction,
legal actions, player positions, ball state, and obstacle geometry. It replies
with exactly one line: a `move` and, only while in possession, an optional
`kick` with direction and power. Every reply is flushed immediately.

- Blank lines are ignored and get no reply, so replies stay one-to-one with
  observations.
- Other message types are skipped. The bot exits on `match_end`.
- If anything goes wrong while choosing an action, the error is written to
  stderr and the bot replies `{"move":"STAY"}` instead of crashing.

## Strategy

The policy uses the visible field geometry to navigate obstacles, predict ball
rebounds and interceptions, and select shots. It handles both attacking sides.

- **Movement:** shortest paths around obstacles using the actual circular
  player collision shape.
- **Shooting:** shot checks consider every move the defender could make.
  When no single lane is safe but several lanes together cannot all be
  covered, the bot randomizes between them (a mixed strategy from fictitious
  play), seeded per match.
- **Release kicks:** when a shot is not available and possession runs long, it
  kicks forward, weighing forward progress, its own recovery time, and
  interception risk.
- **Defense:** it covers possible scoring paths before closing down the
  opponent.
- **Loose or moving ball:** it goes to the earliest reachable interception
  point, including rebounds.

## Robustness

- Kick powers may be integers or decimals (for example `1.5`).
- A missing `kick` entry in the action space is handled. While defending, the
  bot then assumes all eight directions at the strongest power it has seen
  (3 until it sees one).
- `obstacles`, `seed`, `iteration`, and `possession_steps` are optional.

## Physics constants

Field size, player radius, player speed, and goal width are always read from
the observation. Ball radius is read from `field.ball_radius` when present,
otherwise 1.5. The following match the official simulator and are defined at
the top of `team_bot/policy.py`:

| Constant | Value | Meaning |
|---|---|---|
| `BALL_SPEED` | 8 | ball travel per tick |
| `DISTANCE_PER_POWER` | 32 | kick distance per unit of power |
| `SUBSTEP` | 0.675 | maximum collision substep |

## Performance

Shortest-path searches and simulated shots are cached within each tick. A
typical tick takes about 15 ms, and the slowest observed attacking tick with
15 obstacles took about 60 ms on a standard machine.

## Rules compliance

The bot uses no external APIs and does not modify the simulator or game
state. The environment and organizer model are not part of this submission.
