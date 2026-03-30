#!/bin/bash

# UNDER DEVELOPMENT. PLEASE MANAGE THE SETUP MANUALLY UNTIL THIS FILE IS COMPLETED.
# FOR MANUAL INSTRUCTIONS, PLEASE REFER TO ./Documentation/Guides/Installation.md

# Save the user's terminal screen.
printf '\e[?1049h'

# Get the terminal emulator window size.
get_term_size() {
    shopt -s checkwinsize; (:;:)
}
# Trap the window resize signal (handle window resize events).
# See: 'man trap' and 'trap -l'
trap 'get_term_size' WINCH

# Clear the screen and move cursor to (0,0).
# This mimics the 'clear' command.
printf '\e[2J\e[H'

# --- UI CONTENT START ---

DURATION=10
echo "Loading... Please wait."

echo "Installing dependencies..."
echo "Don't worry, these are just test prints. No actual installation is happening (yet)."
echo "Please wait 5 seconds for the process to complete."

# Simple countdown loop
for i in {5..1}; do
    printf "\rTime remaining: %2d seconds " "$i"
    sleep 1
done

echo -e "\n\nTask complete! Press any key to exit."
read -n 1

# --- UI CONTENT END ---

# Restore the user's terminal screen.
printf '\e[?1049l'