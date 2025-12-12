import os
import cv2
import time

# --- DLL CONFIGURATION START ---
# The SDK needs to know exactly where the DLLs are located.
# We take the path you provided and strip the filename to get the directory.
target_dll_path = r"C:\Program Files\Thorlabs\Scientific Imaging\Scientific Camera Support\Scientific Camera Interfaces\SDK\Native Toolkit\dlls\Native_64_lib\thorlabs_tsi_camera_sdk.dll"
dll_dir = os.path.dirname(target_dll_path)

# 1. Add to the System Path (Required for dependencies)
os.environ['PATH'] = dll_dir + os.pathsep + os.environ['PATH']

# 2. Add to Python's DLL search path (Required for Python 3.8+)
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(dll_dir)
# --- DLL CONFIGURATION END ---

# Import the SDK *after* setting the path
from thorlabs_tsi_sdk.tl_camera import TLCameraSDK

# 1. Initialize the Camera SDK
with TLCameraSDK() as sdk:
    available_cameras = sdk.discover_available_cameras()
    if len(available_cameras) == 0:
        print("No cameras found!")
        exit()
    
    # Connect to the first camera found
    with sdk.open_camera(available_cameras[0]) as camera:
        
        # 2. Configure Settings
        camera.exposure_time_us = 5000  # Set exposure to 5ms
        camera.frames_per_trigger_zero_for_unlimited = 0  # Continuous mode
        camera.image_poll_timeout_ms = 1000  # Wait up to 1 second for a frame
        camera.arm(2)  # Prepare the camera buffer (2 frames)
        camera.issue_software_trigger()
        
        print("Starting feedback loop. Press 'q' to stop.")
        
        while True:
            # 3. Get the Frame
            frame = camera.get_pending_frame_or_null()
            
            if frame is not None:
                image_data = frame.image_buffer
                
                # 4. Processing (OpenCV)
                display_image = cv2.normalize(image_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                
                # Simple Logic: Find the brightest spot
                (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(display_image)
                
                # Visualization
                cv2.circle(display_image, maxLoc, 10, (255, 0, 0), 2)
                cv2.imshow('Optical Feedback View', display_image)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
