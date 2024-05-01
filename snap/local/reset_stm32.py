#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import sh
import time
import sys
import argparse
from pyftdi.ftdi import Ftdi

# CBUS0 - BOOT0
# CBUS1 - RST

class Stm32Connection:
    def __init__(self):
        # print(self.ftdi.list_devices())
        # print(self.ftdi.show_devices())
        self.device = "ftdi://ftdi:ft-x:/1"
        # self.device = "ftdi://ftdi:ft-x:DK0AM0V0/1"
        self.ftdi = Ftdi()

    def reset(self):
        self.ftdi.open_from_url(url=self.device)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set CBUS0 and CBUS1 to output
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b10)  # set CBUS0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b00)  # set CBUS0 to 1 and RST to 0
        time.sleep(0.1)
        # self.ftdi.set_cbus_direction(0b11,0b00) # set CBUS0 and CBUS1 to input
        self.ftdi.close()

def main():
    parser = argparse.ArgumentParser(
        description="Resetting STM32 microcontroller in ROSbot XL"
    )

    print("resetting STM32...")
    stm32_connection = Stm32Connection()
    stm32_connection.reset()
    print("done")
    
if __name__ == "__main__":
    main()
