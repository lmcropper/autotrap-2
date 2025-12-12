import platform, sys, os
from pylablib.devices import Thorlabs
import pylablib
print("Python:", sys.executable, sys.version.replace('\\n', ' '))
print("Arch:", platform.architecture())
print("pylablib:", getattr(pylablib, "__version__", "unknown"))
print("PATH contains Thorlabs DLL dir?", any("Thorlabs" in p for p in os.environ.get("PATH","").split(os.pathsep)))

try:
    raw = Thorlabs.list_kinesis_devices()
    print("Thorlabs.list_kinesis_devices() ->", raw)
except Exception as e:
    print("Thorlabs.list_kinesis_devices() raised:", repr(e))

# Optional: list COM ports (if devices expose serial)
try:
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    print("serial.comports() ->", [ (p.device, p.description) for p in ports ])
except Exception as e:
    print("serial.tools.list_ports error:", repr(e))