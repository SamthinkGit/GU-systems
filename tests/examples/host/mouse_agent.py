import sys

from action_space.experimental.mouse.agent import MouseAgent

if __name__ == "__main__":
    # Example Usage:
    # ./mouse_agent.py "Firefox"
    # ./mouse_agent.py "The Sam Account on Netflix"

    MouseAgent.find(sys.argv[1])
