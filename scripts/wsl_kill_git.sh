#!/bin/bash
pkill -f 'git clone' || true
pkill -f 'git-remote-https' || true
sleep 1
echo killed
ls /home/inspect/inspect_ws/src 2>/dev/null || echo no-ws-src
ls /mnt/c/Users/Administrator/Documents/trae_projects/inspect_ws/src | head
