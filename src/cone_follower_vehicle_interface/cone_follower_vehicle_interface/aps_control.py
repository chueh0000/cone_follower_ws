import time
import sys
import threading
import tty
import termios

from doipclient import DoIPClient
from doipclient.connectors import DoIPClientUDSConnector
from common import get_uds_client
from client_config import DOIP_SERVER_IP, DoIP_LOGICAL_ADDRESS
from udsoncan.client import Client

# Import both your Write and Read classes
from FoxPi_write import FoxPiWriteDID 
from FoxPi_read import FoxPiReadDID 

# Globals for control state
current_speed = 0
speed_changed = False
stop_triggered = False
SPEED_LIMIT = 7

# --- IMPORTANT ECU CONFIGURATION ---
PARK_SHIFT_VALUE = 2
DRIVE_SHIFT_VALUE = 5

def get_key():
    """Reads a single keypress from standard input (Unix/Linux/Mac)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # If the first character is an escape sequence (\x1b)
        if ch == '\x1b': 
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            if ch2 == '[':
                if ch3 == 'A':
                    return 'UP'
                elif ch3 == 'B':
                    return 'DOWN'
        # Enter key might be \r or \n depending on terminal
        elif ch == '\r' or ch == '\n':
            return 'ENTER'
        # Ctrl+C
        elif ch == '\x03':
            return 'CTRL_C'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_listener():
    """Runs in a background thread to listen for arrow keys."""
    global current_speed, speed_changed, stop_triggered
    
    while not stop_triggered:
        key = get_key()
        
        if key == 'UP':
            if current_speed < SPEED_LIMIT:
                current_speed += 1
                speed_changed = True
        elif key == 'DOWN':
            if current_speed > 0:
                current_speed -= 1
                speed_changed = True
        elif key == 'ENTER' or key == 'CTRL_C':
            stop_triggered = True
            break

def read_vehicle_data(FoxPi_Read):
    """Helper function to keep the continuous reading logic clean."""
    try:
        driving_ctrl_status = FoxPi_Read.FoxPi_Driving_Ctrl()
        time.sleep(0.4)
        motion_status = FoxPi_Read.FoxPi_Motion_Status()
        time.sleep(0.4)
        motor_status = FoxPi_Read.FoxPi_Motor_Status()
        
        # Displaying APSSpeedCMD instead of TargetSpd for this specific mode
        # Using a carriage return (\r) and end="" to overwrite the same line in terminal
        sys.stdout.write(f"\r\033[96m[Vehicle Status]\033[0m APS TargetSpd: {driving_ctrl_status.get('APSSpeedCMD', current_speed)} kph | "
                         f"VehicleSpeed: {motion_status.get('VehicleSpeed', 'N/A')} | "
                         f"TqSource: {motor_status.get('TqSource', 'N/A')}          ")
        sys.stdout.flush()
    except Exception as e:
        print(f"\n\033[91mError reading data: {e}\033[0m")

def main():
    global current_speed, speed_changed, stop_triggered

    print("Connecting to DoIP server...")
    doip_client = DoIPClient(DOIP_SERVER_IP, DoIP_LOGICAL_ADDRESS, protocol_version=3)
    uds_connection = DoIPClientUDSConnector(doip_client)
    
    assert uds_connection.is_open, "Failed to open UDS connection!"
    
    with Client(uds_connection, request_timeout=4, config=get_uds_client()) as client:
        FoxPi_Write = FoxPiWriteDID(client)
        FoxPi_Read = FoxPiReadDID(client)
        
        print("\n--- Step 1: Executing Reset Sequence ---")
        FoxPi_Write.FoxPi_Reset_Sequence()
        time.sleep(1)
        
        print("\n--- Step 2: Putting Vehicle in Drive via APS ---")
        aps_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, DRIVE_SHIFT_VALUE, 0]
        FoxPi_Write.FoxPi_Driving_Ctrl(aps_values)
        time.sleep(1)
        
        print("\n--- Step 3: Starting APS Speed Control ---")
        print("\033[93m*** CONTROLS ***\033[0m")
        print("\033[93m[UP ARROW]   : Increase speed (Max 7 km/h)\033[0m")
        print("\033[93m[DOWN ARROW] : Decrease speed\033[0m")
        print("\033[93m[ENTER]      : Slowly stop, Park, and Exit\033[0m\n")

        # Start the raw terminal keyboard listener in a separate thread
        listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
        listener_thread.start()
        
        # Write initial state once
        FoxPi_Write.FoxPi_Driving_Ctrl(aps_values)
        
        # Main Control Loop
        while not stop_triggered:
            if speed_changed:
                aps_values[13] = current_speed
                FoxPi_Write.FoxPi_Driving_Ctrl(aps_values)
                # Print on a new line so it doesn't get eaten by the status overwrite
                print(f"\n\033[92m>>> SPEED UPDATED: {current_speed} km/h <<<\033[0m")
                speed_changed = False
            
            read_vehicle_data(FoxPi_Read)
            time.sleep(0.1)
            
        print("\n\n--- Step 4: Stop Triggered. Decelerating... ---")
        while current_speed > 0:
            current_speed -= 1
            aps_values[13] = current_speed
            FoxPi_Write.FoxPi_Driving_Ctrl(aps_values)
            print(f"\033[93mDecelerating... Target Speed: {current_speed} km/h\033[0m")
            
            read_vehicle_data(FoxPi_Read)
            time.sleep(0.01)
                
        print("\n\n--- Step 5: Putting Vehicle in Park ---")
        time.sleep(2)
        aps_values[12] = PARK_SHIFT_VALUE 
        FoxPi_Write.FoxPi_Driving_Ctrl(aps_values)
        time.sleep(1)
            
        print("\n\n--- Step 6: Disabling Control ---")
        disable_APS_values = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        FoxPi_Write.FoxPi_Driving_Ctrl(disable_APS_values)
        time.sleep(0.5)
        FoxPi_Write.FoxPi_Ctrl_Enable_Switch([0])
        print("\033[92mControl safely disabled. Exiting program.\033[0m")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\033[91mProgram interrupted manually. Exiting...\033[0m")