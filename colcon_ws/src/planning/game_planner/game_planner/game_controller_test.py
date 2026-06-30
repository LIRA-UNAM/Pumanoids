#!/usr/bin/env python3

import socket

from gamestate import (
    GameState,
    ReturnData,
    GAMECONTROLLER_RETURN_STRUCT_VERSION,
)

HOST = "0.0.0.0"
PORT = 3838

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((HOST, PORT))

print(f"Listening on {HOST}:{PORT}")

while True:

    data, addr = sock.recvfrom(2048)

    if len(data) < GameState.sizeof():
        print("Packet too small.")
        continue

    try:
        game = GameState.parse(data)

    except Exception as e:
        print(e)
        continue

    print("\n==============================")
    print("Game Controller Packet")
    print("==============================")

    print("Header:", game.header.decode())
    print("Version:", game.version)
    print("Packet:", game.packetNumber)

    print("Players/team:", game.playersPerTeam)

    print("Competition:", game.competitionType)

    print("Stopped:", game.stopped)

    print("Game Phase:", game.gamePhase)

    print("State:", game.state)

    print("Set Play:", game.setPlay)

    print("First Half:", game.firstHalf)

    print("Kicking Team:", game.kickingTeam)

    print("Seconds Remaining:", game.secsRemaining)

    print("Secondary Time:", game.secondaryTime)

    print()

    response = ReturnData.build(
        dict(
            header=b"RGrt",
            version=GAMECONTROLLER_RETURN_STRUCT_VERSION,
            playerNum=1,
            teamNum=0,
            fallen=0,
            pose=[0.0, 0.0, 0.0],
            ballAge=-1.0,
            ball=[0.0, 0.0],
        )
    )

    sock.sendto(response, (addr[0], 3939))
