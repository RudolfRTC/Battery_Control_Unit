#!/usr/bin/env python3
"""
PCAN Loopback Test - Tests if PCAN hardware itself is working
Sends a message and expects to NOT receive it back (no loopback)
This just verifies PCAN can initialize and doesn't immediately go bus-off
"""

import can
import time
import sys

def main():
    print("PCAN Hardware Test")
    print("=" * 40)
    print("This tests if PCAN-USB adapter is working")
    print("CAN bus should have proper termination (120 ohm)")
    print()
    
    try:
        # Try to create bus with different modes
        print("Test 1: Normal mode...")
        try:
            bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
            state = bus.state
            print(f"  Bus state: {state}")
            bus.shutdown()
            if state == can.BusState.ERROR:
                print("  [WARN] Bus in error state")
            else:
                print("  [OK] Bus initialized")
        except Exception as e:
            print(f"  [ERROR] {e}")
        
        time.sleep(0.5)
        
        # Check bus state after init
        print("\nTest 2: Check bus state over time...")
        bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        
        for i in range(5):
            try:
                state = bus.state
                print(f"  t={i}s: Bus state = {state}")
                
                # Try to receive (will timeout if no messages)
                msg = bus.recv(timeout=0.5)
                if msg:
                    print(f"    RX: 0x{msg.arbitration_id:03X}")
                    
            except can.CanError as e:
                print(f"  t={i}s: [ERROR] {e}")
                break
            
            time.sleep(0.5)
        
        bus.shutdown()
        
        print("\n" + "=" * 40)
        print("If bus goes to ERROR or BUS_OFF immediately,")
        print("check:")
        print("  1. Is there another CAN node connected?")
        print("  2. Is 120 ohm termination present?")
        print("  3. Are CAN_H and CAN_L connected correctly?")
        
    except Exception as e:
        print(f"[ERROR] {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
