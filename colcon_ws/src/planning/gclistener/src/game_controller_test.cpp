#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <RoboCupGameControlData.h>

#include <cstring>
#include <iostream>

constexpr int LISTEN_PORT = 3838;
constexpr int RESPONSE_PORT = 3939;
constexpr int MESSAGE_PORT = 3839;

int main()
{
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock<0)
  {
    perror("Cant create socket");
    return 1;
  }
  RoboCupGameControlData data;

  sockaddr_in localAddr{};
  localAddr.sin_family = AF_INET;
  localAddr.sin_port = htons(LISTEN_PORT);
  localAddr.sin_addr.s_addr = INADDR_ANY;

  if(bind(sock,(sockaddr*)&localAddr, sizeof(localAddr))<0)
  {
    perror("Cant bind the socket");
    close(sock);
    return 1;
  }

  std::cout << "Listening on port: " << LISTEN_PORT << std::endl;
  while (true)
  {
    char buffer[2048];

    sockaddr_in senderAddr{};
    socklen_t senderLen = sizeof(senderAddr);

    ssize_t bytes = recvfrom
    (
      sock,
      buffer,
      sizeof(buffer),
      0,
      (sockaddr*)&senderAddr,
      &senderLen
    );
    if (bytes < sizeof(RoboCupGameControlData))
    {
      perror("No info recived or trash recived");
      return 1;
    }

    std::cout << "recived" << bytes << "from" 
    << inet_ntoa(senderAddr.sin_addr)
    << ":" << ntohs(senderAddr.sin_port) << std::endl;

    std::memcpy(&data, buffer, sizeof(RoboCupGameControlData));
    std::cout << "Estado: " << (int)data.state << '\n';
  }
}

