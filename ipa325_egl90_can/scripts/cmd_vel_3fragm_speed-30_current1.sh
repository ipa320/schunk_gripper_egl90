#!/bin/bash
echo "Sending CMD_VEL \"speed: -30.0 current: 1.0\" as 3 fragments message..."
cansend can0  50C#09.84.B5.00.00.F0.C1.00
cansend can0  50C#03.85.00.80.3F.00.00
cansend can0  50C#00.86.00.00.00
echo "CMD_VEL \"speed: -30.0 current: 1.0\" as 3 fragments message sent on can0."
