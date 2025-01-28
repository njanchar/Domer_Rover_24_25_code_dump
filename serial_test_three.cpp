// DOMER ROVER SERIAL TEST THREE
/* Test Goals:
 Send + Recieve repeated, differing messages
 Only parrot back if read command was requested*/

 //Include files
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

#include <bits/stdc++.h>

using namespace std;

int main(){
    // SETUP --------------------------------------------------------------------------
    // get control of port
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
  tty.c_cc[VMIN] = 0;

  cfsetispeed(&tty, B9600); // set baud rate
  cfsetospeed(&tty, B9600);

  if(tcsetattr(serial_port, TCSANOW, &tty)!=0){
   // printf("Error %i \n", errno, strerror(errno));
	printf("error");
	close(serial_port);
	}
	
   usleep(1000*1000);
   // end setup -----------------------------------------------------------------------------

    float nums[7];
    uint8_t write_this[sizeof(nums)];
    uint8_t read_buf[256];
    memset(&read_buf, '\0', sizeof(read_buf));

while(1){
   for(int i=0;i<7;i++){
      nums[i] = (float)(rand());
   }
    int iNum;
    
    if (rand() % 2 == 1)
    {
        nums[0] = 10.5;// read request = 10
    }
    else
    {
        nums[0] = -10.5; // write request = 100
    }

        for(int i=0;i<sizeof(nums)/sizeof(nums[0]);i++)
    {
        memcpy(&(write_this[i*sizeof(float)]), &(nums[i]), sizeof(nums[i]));
    }
       // memcpy(&(write_this[0]), &iNum, sizeof(iNum));
   
	printf("About to send\n");
	write(serial_port, write_this,sizeof(write_this));
	printf("Sent: 7\n");
	this_thread::sleep_for(chrono::seconds(1));
	int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
	
	if(num_bytes < 0){
	printf("error readding bytes");
	return 1;
	}
	// memcpy(&(read_nums[0]), &(read_buf[0]), sizeof(read_nums));
    
    uint8_t read_in[sizeof(write_this)];
    float nums_in[7];
    int iNumRec;
    memcpy(&(read_in[0]), &(read_buf[0]), sizeof(read_buf[0]));

   for(int i=0;i<7;i++){
        memcpy(&(nums_in[i]), &(read_buf[i*4]), sizeof(float));
   }

   memcpy(&iNumRec, &(read_buf[0]), sizeof(iNumRec));
  // temp_buf4[0] = '\0';
  // temp_buf4[1] = '\0';
  // temp_buf4[2] = '\0';
  // temp_buf4[3] = '\0';
    
	printf("Read %i bytes.\n", num_bytes);
	// printf("Recieved message: %i, %i\n", (int)(read_buf[0]), (int)(read_buf[1]));
   printf("Recieved: %f, %f, %f , %f , %f, %f, %f\n", nums_in[0], nums_in[1], nums_in[2], nums_in[3], nums_in[4], nums_in[5], nums_in[6]);
}
	close(serial_port);
	
	return 0;
}




