"""Makes Fossen's Python Vehicle Simulator importable: its source is outside Python's path."""

import os
import sys

sys.path.insert(0, os.environ['FOSSEN_SIMULATOR'])
