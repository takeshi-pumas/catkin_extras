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

#define APP_NAME "erasers_record"
#define STREAM_NAME "rec"
#define DATA_SIZE 1024

int main(int argc, char **argv) {

  // settings for socket connection
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
  addr.sin_addr.s_addr = inet_addr("192.168.0.192");

  if(bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0){
    std::cout << "Error bind:" << std::strerror(errno);
    exit(1);
  }

  if(listen(sockfd,SOMAXCONN) < 0){
    std::cout << "Error listen:" << std::strerror(errno);
    close(sockfd);
    exit(1);
  }

  struct sockaddr_in get_addr;
  socklen_t len = sizeof(struct sockaddr_in);
  int connect = accept(sockfd, (struct sockaddr *)&get_addr, &len);

  if(connect < 0){
    std::cout << "Error accept:" << std::strerror(errno);
    exit(1);
  }

  // setting for pulseaudio
  int pa_errno, pa_result, written_bytes;

  pa_sample_spec ss;
  ss.format = PA_SAMPLE_S16LE;
  ss.rate = 48000;
  ss.channels = 1;

  pa_simple *pa = pa_simple_new(NULL, APP_NAME, PA_STREAM_RECORD, NULL, STREAM_NAME, &ss, NULL, NULL, &pa_errno);

  if (pa == NULL) {
    fprintf(stderr, "ERROR: Failed to connect pulseaudio server: %s\n", pa_strerror(pa_errno));
    return 1;
  }

  // char str[] = "hello world";
  char data[DATA_SIZE];
  while(1){
    pa_result = pa_simple_read(pa, data, DATA_SIZE, &pa_errno);
    if (pa_result < 0) {
      fprintf(stderr, "ERROR: Failed to read data from pulseaudio: %s\n", pa_strerror(pa_errno));
      return 1;
    }
    written_bytes = write(STDOUT_FILENO, data, DATA_SIZE);
    if (written_bytes < DATA_SIZE) {
      fprintf(stderr, "ERROR: Failed to write data to stdout: %s\n", strerror(errno));
      return 1;
    }

    send(connect, data, DATA_SIZE, 0); //送信

  }
  
  pa_simple_free(pa);
  close(connect);

  return 0;

}
