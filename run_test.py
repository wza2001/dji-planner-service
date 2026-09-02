import sys
sys.path.insert(0, ".")
try:
    import app.main
except ModuleNotFoundError as e:
    print(f"caught module error: {e}")
except Exception as e:
    print(f"caught other error: {e}")
