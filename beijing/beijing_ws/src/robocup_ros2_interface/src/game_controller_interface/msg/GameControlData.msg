# RoboCupGameControlData v20, published after validating the UDP packet.

char[4] header
uint8 version
uint8 packet_number
uint8 players_per_team
uint8 competition_type
bool stopped
uint8 game_phase
uint8 state
uint8 set_play
bool first_half
uint8 kicking_team
int16 secs_remaining
int16 secondary_time
TeamInfo[2] teams
