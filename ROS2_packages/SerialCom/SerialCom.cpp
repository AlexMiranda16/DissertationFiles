// C library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()


uint8_t HexNibbleToByte(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  else if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  else return 0;
}

int getMessageValue(char *message){
    int value = (HexNibbleToByte(message[0]) << 12) +
                (HexNibbleToByte(message[1]) << 8)  +
                (HexNibbleToByte(message[2]) << 4)  +
                 HexNibbleToByte(message[3]);

    return value;
}

int receiveMessage(int serial_port){

    int n;
    char channel;
    char msg[4];
    int value;

    while(true){
            
        //Reads first letter and checks if it is a valid one
        n = read(serial_port, &channel, 1);
        if ( (channel >= 'g') && (channel <= 'z') && (n == 1) ){
            
            printf("%c",channel);
            //Stores the 4 bytes of the message
            for(int i = 0; i<4; i++){
                read(serial_port, &msg[i], 1);
                printf("%d", msg[i]);
            }
            printf("     ");

            //Converts the message to a value
            value = getMessageValue(msg);
            break;
        }
    }
}

int main(int arc, char **argv){

    //Opening the serial port device (Arduino)
    int serial_port = open("/dev/ttyUNO", O_RDWR);

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }

    struct termios tty;

    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -2;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    //tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
    tty.c_cflag &= ~CSIZE; // Clear all the size bits, then use one of the statements below
    //tty.c_cflag |= CS5; // 5 bits per byte
    //tty.c_cflag |= CS6; // 6 bits per byte
    //tty.c_cflag |= CS7; // 7 bits per byte
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)


    tty.c_lflag &= ~ICANON; // Disable Canonical input mode
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP


    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    tty.c_iflag &= ~(INPCK);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes


    // Output flags - Turn off output processing
    //
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    //
    // config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
    //                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
    tty.c_oflag = 0;

//    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
//    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)

    tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 5;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return -3;
    }

    //Each message from arduino has a maximum of 5 bytes, 1 for the channel, 4 for the value
    char channel;   //Channel for the arduino communication
    char msg[4];    //Message to send/receive to/from arduino
    int value = 0;  //Real value of the message received
    
    char char_buf;
    bool read_complete = false; //Flag to check if the read is complete
    int n = 0;

    while(true){
        //read(serial_port, &char_buf, 1);

        while(!read_complete){
            
            //Reads first letter and checks if it is a valid one
            n = read(serial_port, &channel, 1);
            if ( (channel >= 'g') && (channel <= 'z') && (n == 1) ){
                
                //printf("%c",channel);
                //Stores the 4 bytes of the message
                for(int i = 0; i<4; i++){
                    read(serial_port, &msg[i], 1);
                    //printf("%d", msg[i]);
                }
                //printf("     ");

                //Converts the message to a value and ends the read loop
                value = getMessageValue(msg);
                read_complete = true;
            }
        }
        read_complete = false;

        

        //For testing
        printf("Channel: %c // Value: %d\n", channel, value);
        
        write(serial_port,"R", 1);
        write(serial_port, "0", 1);
        write(serial_port, "0", 1);
        write(serial_port, "5", 1);
        write(serial_port, "F", 1);
        
    }

    
/*

    char read_buf [256];
    read_buf[0] = 'T';

    char msg[5] = {'V', '0', '0', 'F', 'A'};
    //byte test = 8;

    //write(serial_port, msg, sizeof(msg));
    //write(serial_port,"1", 1);
    //write(serial_port, "0", 1);
    //write(serial_port, "0", 1);
    //write(serial_port, "F", 1);
    //write(serial_port, "A", 1);

    char test_buf;
   // int n = read(serial_port, &test_buf, sizeof(test_buf));
    printf("%c\n", test_buf);

    // Read bytes. The behaviour of read() (e.g. does it block?,
    // how long does it block for?) depends on the configuration
    // settings above, specifically VMIN and VTIME
    //int n = read(serial_port, &read_buf, sizeof(read_buf));
    //printf("%d\n", n);

    printf("Message is: %c\n", read_buf[0]);
    printf("Message is: %c\n", read_buf[1]);
    printf("Message is: %c\n", read_buf[2]);
    printf("Message is: %c\n", read_buf[3]);
    printf("Message is: %c\n", read_buf[4]);
    printf("Message is: %c\n", read_buf[5]);
    printf("Message is: %c\n", read_buf[6]);
    printf("Message is: %c\n", read_buf[7]);
    printf("Message is: %c\n", read_buf[8]);
    printf("Message is: %c\n", read_buf[9]);
    printf("Message is: %c\n", read_buf[10]);
    printf("Message is: %c\n", read_buf[11]);
    printf("Message is: %c\n", read_buf[12]);
    printf("Message is: %c\n", read_buf[13]);
    printf("Message is: %c\n", read_buf[14]);
    printf("Message is: %c\n", read_buf[15]);
    printf("Message is: %c\n", read_buf[16]);
    printf("Message is: %c\n", read_buf[17]);
    printf("Message is: %c\n", read_buf[18]);
    printf("Message is: %c\n", read_buf[19]);
    printf("Message is: %c\n", read_buf[20]);
    printf("Message is: %c\n", read_buf[21]);


    // n is the number of bytes read. n may be 0 if no bytes were received, and can also be negative to signal an error.
*/
    close(serial_port);

    return 0;
}
