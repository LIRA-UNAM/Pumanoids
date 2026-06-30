import socket
from gamestate import GAME_CONTROLLER_RESPONSE_VERSION, GameState, ReturnData 


host = "0.0.0.0"  
port = 3838       


server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((host, port))
print(f"Escuchando en {host}:{port}...")

while True:
    data, addr = server_socket.recvfrom(1024)  
    print(f"\nRecibiendo desde {addr}")


    if len(data) < GameState.sizeof():
        print("⚠️ Paquete demasiado pequeño, ignorando...")
        continue


    try:
        gamestate = GameState.parse(data)
    except Exception as e:
        print(f"⚠️ Error al desempaquetar los datos: {e}")
        continue


    print(f"""
    📡 **Datos del GameController**
    --------------------------------
    🏷️  Encabezado: {gamestate.header.decode("utf-8")}
    🔢 Versión del protocolo: {gamestate.version}
    📦 Número de paquete: {gamestate.packet_number}
    👥 Jugadores por equipo: {gamestate.players_per_team}
    🏆 Tipo de juego: {gamestate.game_type}
    ⏳ Estado del juego: {gamestate.game_state}
    🏁 Primera mitad: {'Sí' if gamestate.first_half else 'No'}
    ⚽ Equipo que saca: {gamestate.kick_of_team}
    🔄 Estado secundario: {gamestate.secondary_state}
    ℹ️  Info secundaria: {gamestate.secondary_state_info.decode("utf-8")}
    📍 Último equipo en tocar balón fuera: {gamestate.drop_in_team}
    ⏱️ Tiempo desde último saque de banda: {gamestate.drop_in_time} seg
    ⌛ Segundos restantes: {gamestate.seconds_remaining} seg
    ⏳ Tiempo secundario: {gamestate.secondary_seconds_remaining} seg
       Tiempo penalización: {gamestate.teams[1].players[0].secs_till_unpenalized}
       Penalización :{gamestate.teams[1].players[0].penalty}

    """)
    return_message = ReturnData.build(dict(
    version=GAME_CONTROLLER_RESPONSE_VERSION,
    team=0,
    player=1,
    message= 2
    )) 
    #Send message
    server_socket.sendto(return_message, (addr[0], 3939))

 
