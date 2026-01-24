#!/usr/bin/env python3
"""
BCU Output Test - Cycles through all 20 outputs, 5 seconds each
Uses PCAN-USB at 500 kbps
"""

import can
import time
import sys

# CAN IDs
CAN_ID_CMD_OUTPUT_SET = 0x200

def set_output(bus, channel, state):
    """Set output channel ON or OFF"""
    data = [channel, state, 0, 0, 0, 0, 0, 0]
    msg = can.Message(arbitration_id=CAN_ID_CMD_OUTPUT_SET, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        print(f"  [ERROR] Failed to send: {e}")
        return False

def main():
    print("=" * 60)
    print("BCU Output Test - 20 channels, 5 seconds each")
    print("=" * 60)
    
    try:
        bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        print("[OK] Connected to PCAN-USB @ 500 kbps\n")
    except Exception as e:
        print(f"[ERROR] Failed to connect: {e}")
        return 1
    
    try:
        for channel in range(20):
            # Turn ON
            print(f"Channel {channel:2d}: ON  ", end="", flush=True)
            if set_output(bus, channel, 1):
                # Wait 5 seconds
                for i in range(5, 0, -1):
                    print(f"\rChannel {channel:2d}: ON  [{i}s remaining]  ", end="", flush=True)
                    time.sleep(1)
                
                # Turn OFF
                set_output(bus, channel, 0)
                print(f"\rChannel {channel:2d}: OFF                    ")
            else:
                print("FAILED")
            
            # Small delay before next channel
            time.sleep(0.2)
        
        print("\n" + "=" * 60)
        print("Test complete - all 20 channels cycled")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted - turning all outputs OFF...")
        for ch in range(20):
            set_output(bus, ch, 0)
        print("All outputs OFF")
    finally:
        bus.shutdown()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
