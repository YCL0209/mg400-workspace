"""``python -m viz`` entry — runs the inspection ws server.

Convention matches ``python -m robot_core.scripts.workbench`` style.
"""

from .server import main

if __name__ == "__main__":
    main()
