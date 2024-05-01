#!/usr/bin/env python3


import subprocess
import json
import hashlib
import string
import re
import sys

def hex_to_num(hex_str):
    # Check if the hex string is valid
    if not all(c in string.hexdigits for c in hex_str):
        raise ValueError("Invalid hex string")

    # Convert the hex string to bytes
    hex_bytes = bytes.fromhex(hex_str)

    # Compute the SHA-256 hash of the hex bytes
    hash = hashlib.sha256(hex_bytes).hexdigest()

    # Truncate the hash to 6 characters
    hash = hash[:6]

    # Return the hash as an ASCII string
    return hash

def main():
    try:
        # Check if a CPU ID was provided as an argument
        if len(sys.argv) != 2:
            print("Usage: python3 script.py <cpu_id>")
            sys.exit(1)

        # Obtain CPU ID from command line argument
        cpu_id = sys.argv[1]

        if cpu_id:
            # Calculate the serial number
            serial_number = hex_to_num(cpu_id)
            print(f"CPU ID: 0x{cpu_id}")
            print(f"Serial Number: {serial_number}")
        else:
            print("Failed to obtain CPU ID.")
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
