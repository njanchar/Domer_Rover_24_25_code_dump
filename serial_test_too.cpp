#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <thread>

using namespace std;

int main(){
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    close(serial_port);
    serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);

    struct termios tty;

    if(tcgetattr(serial_port, &tty)!=0){
       // printf("Error %i \n", errno, strerror(errno));
    	printf("error");
	}
	
	if(flock(serial_port, LOCK_EX | LOCK_NB) == -1){
	printf("serial port locked");
	return -1;
	}
	
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;
  tty.c_cc[VMIN] = 20;

  cfsetispeed(&tty, B9600);
  cfsetospeed(&tty, B9600);

  if(tcsetattr(serial_port, TCSANOW, &tty)!=0){
   // printf("Error %i \n", errno, strerror(errno));
	printf("error");
	close(serial_port);
	}
	
   usleep(1000*1000);
   int read_nums[2];
   uint8_t temp_buf[4];
   float temp_num;
   uint8_t temp_buf2[4];
   float temp_num2;
	uint8_t read_buf[256];
	uint8_t write_buf[4];
   float send_it = 3.45;
   memcpy(&(write_buf),&send_it,sizeof(write_buf));
   float temp_num3;
   float temp_num4;
   float temp_num5;
   uint8_t temp_buf5[4];
   uint8_t temp_buf3[4];
   uint8_t temp_buf4[4];
	memset(&read_buf, '\0', sizeof(read_buf));
	printf("About to send\n");
	write(serial_port, write_buf, sizeof(write_buf));
	printf("Sent: 1\n");
	this_thread::sleep_for(chrono::seconds(1));
	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
	
	if(num_bytes < 0){
	printf("error readding bytes");
	return 1;
	}
	// memcpy(&(read_nums[0]), &(read_buf[0]), sizeof(read_nums));
   for(int i=0;i<20;i++){
      if(i<4){temp_buf[i] = read_buf[i];}
      else if(i<8){temp_buf2[i-4] = read_buf[i];}
      else if(i<12){temp_buf3[i-8] = read_buf[i];}
      else if(i<16){temp_buf4[i-12] = read_buf[i];}
      else if(i<20){temp_buf5[i-16] = read_buf[i];}
   }
  // temp_buf4[0] = '\0';
  // temp_buf4[1] = '\0';
  // temp_buf4[2] = '\0';
  // temp_buf4[3] = '\0';
   memcpy(&temp_num,&(temp_buf),sizeof(temp_num));
   memcpy(&temp_num2,&(temp_buf2),sizeof(temp_num2));
   memcpy(&temp_num3,&(temp_buf3),sizeof(temp_num3));
   memcpy(&temp_num4,&(temp_buf4),sizeof(temp_num4));
   memcpy(&temp_num5,&(temp_buf5),sizeof(temp_num5));
	printf("Read %i bytes.\n", num_bytes);
	// printf("Recieved message: %i, %i\n", (int)(read_buf[0]), (int)(read_buf[1]));
   printf("Read two nums: %f, %f , %f , %f, %f\n", temp_num, temp_num2, temp_num3, temp_num4, temp_num5);

	close(serial_port);
	
	return 0;


}

