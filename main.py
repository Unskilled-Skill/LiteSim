# main.py
import os
import sys

# Crash prevention
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE" 

# PyVista settings
try:
    import pyvista as pv
    pv.global_theme.allow_empty_mesh = True
except ImportError:
    print("Install pyvista!")
    sys.exit(1)

# Start App
from gui import ControlPanel

if __name__ == "__main__":
    app = ControlPanel()
    app.mainloop()