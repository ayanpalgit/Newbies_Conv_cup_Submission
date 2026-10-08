# Sharp FC

Python 3.11+. No third-party dependencies, downloads, or model weights.

## Run the agent

Extract the ZIP into its own folder. `submission.json` contains the launch
command. Run from that folder: `python -m team_bot.bot`.
The process waits for observations from the official runner; it does not open
a game window by itself.

Input and output follow the official newline-delimited JSON protocol.
For each observation, the bot reads its player ID, attack direction, legal
actions, player positions, ball state, and obstacle geometry. It returns one
`move` and, only while in possession, an optional `kick` with direction and
power. It flushes each response and exits on `match_end`.

## How judges run a match

1. Check each team's ZIP with the official archive checker, then extract the
   accepted submissions into separate folders in the offline evaluation environment.
2. Start one process per team using its own `submission.json` command and folder.
3. At every iteration, send both bots the current state with their respective
   player IDs and attack directions. Neither bot receives the other's current action.
4. Read one JSON action from each bot. The official engine applies both actions
   simultaneously, handles physics and possession, and records goals.
5. Save scores, action errors, and match logs. Repeat with different seeds and
   both player assignments; apply the organizers' announced ranking and tie-break rules.

The bot handles either player side and continues accepting observations until
`match_end`, including extra iterations supplied by the organizers.

## Run against another team's submission

Use the official repository's `participants/` folder. Extract this ZIP into
`my_team/` and the other team's ZIP into `other_team/`, with each
`submission.json` directly inside its folder.

Save the following as `config/head_to_head.json`. Replace the opponent's `name`
and `command` with the exact values from **their** `submission.json`; the example
assumes they use the starter-kit launch command. `working_directory` is a match
runner setting and belongs in this match configuration, not in either submission.

```json
{
  "game_config": "game.json",
  "seed": 101,
  "action_timeout_seconds": 2.0,
  "log_directory": "logs/head_to_head",
  "player_1": {
    "name": "Sharp FC",
    "command": ["python", "-m", "team_bot.bot"],
    "working_directory": "my_team"
  },
  "player_2": {
    "name": "Other team",
    "command": ["python", "-m", "team_bot.bot"],
    "working_directory": "other_team"
  }
}
```

From `participants/`, run:

```powershell
python run_match.py --config config/head_to_head.json
```

The command prints the score and action-error counts and saves a JSONL replay
under `logs/head_to_head/`. Swap the complete `player_1` and `player_2` objects
to reverse sides, and use `--seed 102` for another layout. The official environment
and runner are supplied separately by the organizers; the ZIP contains the agent.


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
