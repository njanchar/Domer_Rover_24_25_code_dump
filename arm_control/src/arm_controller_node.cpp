#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/ps_controller.hpp"

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

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode() : Node("arm_control_node")
    {
        // SERIAL SETUP -----------------------------------------------------------
        serial_port = open("/dev/ttyACM0", O_RDWR);
        close(serial_port);
        serial_port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
        // Setup 
        struct termios tty;

        if(tcgetattr(serial_port, &tty)!=0)
        {
            // printf("Error %i \n", errno, strerror(errno));
    	    printf("error");
	    }
        // Check if serial port is occupied
	    if(flock(serial_port, LOCK_EX | LOCK_NB) == -1)
        {
	        printf("serial port locked");
	        //return -1;
	    }
        // Configure Serial Port
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
        // Set Baud Rate to 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if(tcsetattr(serial_port, TCSANOW, &tty)!=0){
            // printf("Error %i \n", errno, strerror(errno));
	        printf("error");
	        close(serial_port);
	    }
    
        usleep(1000*1000); // Makes it work, idk why
        // END SERIAL SETUP -----------------------------------------------------------------------

        // Create subscriber
        subscriber_ = this->create_subscription<custom_msgs::msg::PsController>("topic_cmds", 10, std::bind(&ArmControlNode::controllerCallback,this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Arm Control Node Started");
    }

private:
    uint8_t write_buf[21]; // Write buffer
    rclcpp::Subscription<custom_msgs::msg::PsController>::SharedPtr subscriber_;
    float positions[5] = {0.0,0.0,90.0,90.0,0.0}; // positions in meters for the sake of converting and copying to the buffer
    float dt = 0.05; // could be implemented for increasing commands
    int serial_port;

    void controllerCallback(const custom_msgs::msg::PsController::SharedPtr msg)   // subscriber callback
    {
        for(int i=0;i<sizeof(write_buf);i++){write_buf[i] = '\0';} // set write buffer to zero
        write_buf[0] = 'w'; // indicator char
        if(msg->dpad_u){positions[0] += 0.01;} // slightly increment each position value because the callback should be firing often
        if(msg->dpad_d){positions[0] -= 0.01;}
        if(msg->rpad_u){positions[1] -= 0.01;}
        if(msg->rpad_d){positions[1] -= 0.01;}

	if(msg->lstick_y>0){positions[2] += 0.1;}
	if(msg->lstick_y<0){positions[2] -= 0.1;}
	if(msg->rstick_y>0){positions[3] += 0.01;}
	if(msg->rstick_y<0){positions[3] -= 0.01;}
	    
	// Make sure no unwanted negative values
	if(positions[0] < 0){positions[0] = 0.0;}
	if(positions[2] < 0){positions[2] = 0.0;}
	if(positions[2] > 180){positions[2] = 180.0;}
	if(positions[3] < 0){positions[3] = 0.0;}
	if(positions[3] > 180){positions[3] = 180.0;}
	    
        float s_write = 0.296926 + positions[0]; // temporary variable to add in the unextended length into the command

        memcpy(&(write_buf[1]), &s_write, sizeof(float)); // copy shoulder position, size of float
        memcpy(&(write_buf[5]), &(positions[1]), sizeof(float)); // copy elbow position
	memcpy(&(write_buf[9]), &(positions[2]), sizeof(float));
	memcpy(&(write_buf[13]), &(positions[3]), sizeof(float));
	    
        write(serial_port, write_buf, sizeof(write_buf)); // write buffer
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmControlNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
