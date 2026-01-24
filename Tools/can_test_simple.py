#!/usr/bin/env python3
"""
Simple CAN test - just try to receive for 10 seconds with bus reset
"""

import can
import time
import sys

def main():
    print("Simple CAN Test")
    print("=" * 40)
    
    # Try multiple times with fresh bus connection
    for attempt in range(3):
        print(f"\nAttempt {attempt + 1}/3...")
        
        try:
            bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=500000)
            print("[OK] Connected to PCAN-USB")
            
            # Try to receive messages
            msg_count = 0
            start = time.time()
            
            while (time.time() - start) < 5.0:
                try:
                    msg = bus.recv(timeout=0.5)
                    if msg:
                        msg_count += 1
                        print(f"  RX: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")
                except can.CanError as e:
                    print(f"  [ERROR] {e}")
                    break
            
            bus.shutdown()
            
            if msg_count > 0:
                print(f"\n[OK] Received {msg_count} messages")
                return 0
            else:
                print("  No messages received")
                
        except Exception as e:
            print(f"[ERROR] {e}")
        
        time.sleep(1)
    
    print("\n[FAIL] Could not communicate with BCU")
    print("\nCheck:")
    print("  1. CAN_H and CAN_L connections")
    print("  2. 120 ohm termination resistor")
    print("  3. BCU power supply")
    print("  4. PCAN cable connection")
    
    return 1

if __name__ == "__main__":
    sys.exit(main())
