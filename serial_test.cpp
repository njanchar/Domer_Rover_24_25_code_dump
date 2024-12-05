#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

#include <thread>

using namespace std;

int main(){
    int serial_port = open("/dev/", O_RDWR);

    if(serial_port < 0){
        printf("Error %i \n", errno, strerror(errno));
    }

    struct termios tty;

    if(tcgetattr(serial_port, &tty)!=0){
        printf("Error %i \n", errno, strerror(errno));
    }

  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= ~CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_cflag &= ~ICANON;
  tty.c_cflag &= ~ECHO;
  tty.c_cflag &= ~ECHOE;
  tty.c_cflag &= ~ECHONL;
  tty.c_cflag &= ~ISIG;
  tty.c_cflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 1;

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);    // need both ftns to work
    
  if(tcsetattr(serial_port, TCSANOW, &tty)!=0){
    printf("Error %i \n", errno, strerror(errno));
  }

  while(true){
    unsigned char msg[] = {'D', 'R'};
    write(serial_port, msg, sizeof(msg));
    this_thread::sleep_for(chrono::seconds(1));
  }


}
