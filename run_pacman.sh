#!/bin/bash
echo "Iniciando robôs fantasmas..."
rosrun stage_pid stage_pacman.py 1 & rosrun stage_pid stage_pacman.py 2 & rosrun stage_pid stage_pacman.py 3 & rosrun stage_pid stage_pacman.py 4
