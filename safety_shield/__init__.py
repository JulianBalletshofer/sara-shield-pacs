# Re-export the extension module at package import (optional but convenient)
try:
    from .safety_shield_py import *  # noqa
except Exception:
    print("from .safety_shield_py import * failed.")

# Or, if you prefer explicit:
# from . import safety_shield_py
# __all__ = ["safety_shield_py"]
