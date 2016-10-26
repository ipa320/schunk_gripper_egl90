#!/bin/bash
echo "Sending CMD_VEL \"speed: -30.0\" message..."
cansend can0 50C#05.B5.00.00.F0.C1
echo "CMD_VEL \"speed: -30.0\" message sent on can0."
