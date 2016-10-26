#!/bin/bash
echo "Sending CMD_VEL \"speed: -30.0 current: 1.0\" message..."
cansend can0 50C#03.A3.80.3F
cansend can0 50C#05.B5.00.00.F0.C1
echo "CMD_VEL \"speed: -30.0 current: 1.0\" message sent on can0."
