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

## Playing strategy

The policy uses the visible field geometry to navigate obstacles, predict ball
rebounds and interceptions, and select shots. It handles both attacking sides.
Routes use the actual circular player collision shape. Shot checks consider
possible defender movements, and can vary shot choices when several lanes are available.
Defense covers possible scoring paths before closing down the opponent.
Release kicks balance forward progress, recovery time, and interception risk.
The bot uses no external APIs and does not modify the simulator or game state.
The environment and organizer model are not part of this submission.
