#!/bin/bash
echo "Sending CMD_ACK message..."
cansend can0 50C#01.8B
echo "CMD_ACK message sent on can0."
