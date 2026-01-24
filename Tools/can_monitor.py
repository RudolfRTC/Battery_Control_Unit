#!/usr/bin/env python3
"""
BCU CAN Monitor - Simple script to monitor CAN messages from BCU
Uses PCAN-USB interface at 500 kbps
"""

import can
import sys
import time
from datetime import datetime

# BCU CAN message IDs
CAN_IDS = {
    0x100: "BCU_Status",
    0x101: "BCU_Current_1",
    0x102: "BCU_Current_2",
    0x103: "BCU_Current_3",
    0x104: "BCU_Voltage",
    0x105: "BCU_Temperature",
    0x106: "BCU_Outputs",
    0x107: "BCU_Inputs",
    0x108: "BCU_Faults",
    0x110: "BCU_BTT_Diag_1",
    0x111: "BCU_BTT_Diag_2",
    0x112: "BCU_BTT_Diag_3",
    0x113: "BCU_BTT_Current_1",
    0x114: "BCU_BTT_Current_2",
    0x115: "BCU_BTT_Current_3",
    0x120: "BCU_RUL_Status",
    0x121: "BCU_Timing",
    0x7E8: "UDS_Response",
}

def main():
    print("=" * 60)
    print("BCU CAN Monitor - PCAN-USB @ 500 kbps")
    print("=" * 60)
    print("Press Ctrl+C to exit\n")
    
    try:
        # Try to connect to PCAN-USB
        bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        print(f"[OK] Connected to PCAN-USB")
    except Exception as e:
        print(f"[ERROR] Failed to connect to PCAN-USB: {e}")
        print("\nTroubleshooting:")
        print("  1. Ensure PCAN-USB is connected")
        print("  2. Ensure PCAN drivers are installed")
        print("  3. Try: pip install python-can[pcan]")
        return 1
    
    msg_count = 0
    start_time = time.time()
    last_ids = {}
    
    try:
        print("\nWaiting for CAN messages...\n")
        print(f"{'Time':12} {'ID':6} {'Name':20} {'DLC':4} {'Data':24}")
        print("-" * 70)
        
        while True:
            msg = bus.recv(timeout=1.0)
            
            if msg is not None:
                msg_count += 1
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                msg_name = CAN_IDS.get(msg.arbitration_id, "Unknown")
                data_hex = " ".join(f"{b:02X}" for b in msg.data)
                
                print(f"{timestamp:12} {msg.arbitration_id:04X}  {msg_name:20} {msg.dlc:4} {data_hex}")
                
                last_ids[msg.arbitration_id] = time.time()
            else:
                # Timeout - print status
                elapsed = time.time() - start_time
                print(f"\r[{elapsed:.1f}s] Waiting... {msg_count} messages received", end="", flush=True)
                
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        bus.shutdown()
        
    elapsed = time.time() - start_time
    print(f"\nReceived {msg_count} messages in {elapsed:.1f} seconds")
    if msg_count > 0:
        print(f"Average rate: {msg_count/elapsed:.1f} msg/s")
        print(f"\nUnique IDs seen: {sorted(last_ids.keys())}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
