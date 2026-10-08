"""Offline JSON-lines entry point. No external packages or model files."""
import json
import sys

from .policy import Policy


def main():
    policy = Policy()
    for line in sys.stdin:
        try:
            message = json.loads(line)
            if message.get("type") == "match_end":
                break
            if message.get("type") != "observation":
                continue
            action = policy.choose_action(message["observation"])
        except Exception as error:
            print(str(error), file=sys.stderr, flush=True)
            action = {"move": "STAY"}
        print(json.dumps(action, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
