#include <iostream>
#include <cstring>

#include <errno.h>
#include <string.h>
#include <unistd.h>

//socket
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>

// pulseaudio
#include <pulse/error.h> 
#include <pulse/simple.h>

#define APP_NAME "erasers_play"
#define STREAM_NAME "play"
#define DATA_SIZE 1024

int main(int argc, char **argv) {

  // settings for socket communication
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0){
    std::cout << "Error socket:" << std::strerror(errno);
    exit(1);
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(struct sockaddr_in));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(1234);
  // addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  addr.sin_addr.s_addr = inet_addr("192.168.195.27");

  int connected;
  while (1) {
    int connected = connect(sockfd, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
    std::cout << "waiting for connected ..."<< std::endl;
    sleep(1);
    if (connected == 0)
      break;
  }
  std::cout << "socket connected" << std::endl;

  // settings for pulseaudio
  int pa_errno, pa_result, read_bytes;

  pa_sample_spec ss;
  ss.format = PA_SAMPLE_S16LE;
  ss.rate = 48000;
  ss.channels = 1;

  pa_simple *pa = pa_simple_new(NULL, APP_NAME, PA_STREAM_PLAYBACK, NULL, STREAM_NAME, &ss, NULL, NULL, &pa_errno);
  if (pa == NULL) {
    fprintf(stderr, "ERROR: Failed to connect pulseaudio server: %s\n", pa_strerror(pa_errno));
    return 1;
  }
  
  // char r_str[12]; //受信データ格納用
  char data[DATA_SIZE];  
  while (1){
    recv(sockfd, data, DATA_SIZE, 0);
    pa_result = pa_simple_write(pa, data, DATA_SIZE, &pa_errno);
    if (pa_result < 0) {
      fprintf(stderr, "ERROR: Failed to write data to pulseaudio: %s\n", pa_strerror(pa_errno));
      return 1;
    }
  }

  pa_simple_free(pa);
  close(sockfd);

  return 0;

}
