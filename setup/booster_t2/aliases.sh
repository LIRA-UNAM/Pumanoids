# --- Show important commands on interactive shell startup ---
echo ""
echo "=== Booster Robot - Important Commands ==="
echo "  start_left    - Start the robot on the LEFT side of the field | Change team id and player id with id:=x and team:=x"
echo "  start_right   - Start the robot on the RIGHT side of the field | Change team id and player id with id:=x and team:=x"
echo "  stop          - Stop the running robot process"
echo "  brain_log     - Tail the brain log file (live)"
echo "  cb            - Build the beijing demo"
echo "  ip_gc         - Set the GameController IP (usage: ip_gc ip:=x.x.x.x)"
echo "  goal_keeper   - Change team id with team:=x"
echo "==========================================="
echo ""
alias start_left='cd ~/Pumanoids/beijing && source beijing_ws/install/setup.bash && sudo ./scripts/start.sh pos:=left disable_com:=false tree:=game'
alias stop='cd ~/Pumanoids/beijing && sudo ./scripts/stop.sh'
alias brain_log='cd  ~/Pumanoids/beijing && tail -F brain.log'
alias cb='cd ~/Pumanoids && make demo'
alias start_right='cd ~/Pumanoids/beijing && source beijing_ws/install/setup.bash && sudo ./scripts/start.sh pos:=right disable_com:=false tree:=game'
alias ip_gc='cd ~/Pumanoids/beijing && ./scripts/set_gc_ip.sh'
alias goal_keeper='cd ~/Pumanoids/beijing && source beijing_ws/install/setup.bash && sudo ./scripts/start.sh pos:=left disable_com:=false tree:=game role:=goal_keeper'
