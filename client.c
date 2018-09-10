/* 
 *  Adapted socket client code (by Borja Sotomayor) to handle roomba move sequence commands.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <errno.h> /* Error number definitions */
#include <fcntl.h> /* File control definitions */
#include <pthread.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <ctype.h>
#define MAXBUFF 100
#define SAFE 0x83
#define FULL 0x84

char data;
int create_fd;
char velocity_highbyte = 0x00;
//char velocity_lowbyte = 0xC8;  // default velocity = 200 mm/sec
char velocity_lowbyte = 0xFF;
int n = 0;

void stopRobot();
void turnRight();
void turnLeft();
void waitAngle(char angle_highbyte, char angle_lowbyte);
void turn(char velocity_highbyte, char velocity_lowbyte, char radius_highbyte, char radius_lowbyte);
void drive(char* dist);
void setDefaultSpeed(char highbyte, char lowbyte);
void setMode(char mode);
void startRobot();
void* sendCmd(void *cmd);

int main(int argc, char *argv[])
{
    pthread_t roombaThread;
    /* A socket is just a file descriptor, i.e., an int */
    int clientSocket;
    //int create_fd;
    char velocity_highbyte = 0x00;
    char velocity_lowbyte = 0xC8;  // default velocity = 200 mm/sec

    /* The addrinfo struct is used both as a parameter to getaddrinfo (to provide "hints" on
       what type of address we're using), and as a way to return the addresses associated
       to a given hostname and service. For example, "www.google.com" (with service "http")
       might resolve to multiple IP addresses, and one port (80). So, addrinfo is
       actually a linked list:

           struct addrinfo {
               int              ai_flags;
               int              ai_family;
               int              ai_socktype;
               int              ai_protocol;
               size_t           ai_addrlen;
               struct sockaddr *ai_addr;
               char            *ai_canonname;
               struct addrinfo *ai_next;
           };

       Note how it contains information used in other socket functions (family, socket type, etc.)
       and the sockaddr for the address.

       We need to declare three addrinfo's: */
    struct addrinfo hints, // Used to provide hints to getaddrinfo()
                    *res,  // Used to return the list of addrinfo's
                    *p;    // Used to iterate over this list


    /* Host and port */
    char *host, *port;

    /* Buffer to receive bytes from the socket */
    char buffer[100 + 1]; // +1 for '\0'

    /* Number of bytes received */
    int nbytes;

    /* Used by getopt */
    int opt;

    /* Use getopt to fetch the host and port */
    while ((opt = getopt(argc, argv, "h:p:")) != -1)
        switch (opt)
        {
            case 'h':
                host = strdup(optarg);
                break;
            case 'p':
                port = strdup(optarg);
                break;
            default:
                printf("Unknown option\n"); exit(1);
        }

    if(host == NULL || port == NULL)
    {
        printf("USAGE: client -h HOST -p PORT\n");
        exit(1);
    }

    /* We start by creating the "hints" addrinfo that is used to aid
       getaddrinfo in returning addresses that we'll actually be able
       to use */

    /* We want to leave all unused fields of hints to zero*/
    memset(&hints, 0, sizeof(hints));

    /* We leave the family unspecified. Based on the hostname (or IP address), 
       getaddrinfo should be able to determine what family we want to use.
       However, we can set this to AF_INET or AF_INET6 to force a specific version */
    hints.ai_family = AF_UNSPEC;

    /* We want a reliable, full-duplex socket */
    hints.ai_socktype = SOCK_STREAM;

    /* Call getaddrinfo with the host and port specified in the command line */
    if (getaddrinfo(host, port, &hints, &res) != 0)
    {
        perror("getaddrinfo() failed");
        exit(-1);
    }

    /* Iterate through the list */
    for(p = res;p != NULL; p = p->ai_next) 
    {
        /* The list could potentially include multiple entries (e.g., if a
           hostname resolves to multiple IP addresses). Here we just pick
           the first address we can connect to, although we could do
           additional filtering (e.g., we may prefer IPv6 addresses to IPv4
           addresses */

        /* Try to open a socket */
        if ((clientSocket = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) 
        {
            perror("Could not open socket");
            continue;
        }

        /* Try to connect.
           Note: Like many other socket functions, this function expects a sockaddr and
                 its length, both of which are conveniently provided in addrinfo */
        if (connect(clientSocket, p->ai_addr, p->ai_addrlen) == -1) 
        {
            close(clientSocket);
            perror("Could not connect to socket");
            continue;
        }

        /* If we make it this far, we've connected succesfully. Don't check any more entries */
        break;
    }
    
    /* We don't need the linked list anymore. Free it. */
    freeaddrinfo(res);

    /* Establishing roomba connection */
    struct termios options;
    int ret;
    create_fd = open("/dev/ttyUSB0", O_RDWR);
    if (create_fd == -1) {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
    }

    /* Get the current options for the port */
    ret = tcgetattr(create_fd, &options);
    //printf("return value is %d: input speed is %d\r\n", ret, options.c_ispeed);
   
    /* Set the baud rates to 57600 */
    ret = cfsetispeed(&options, B57600);
    //printf("cfsetispeed returns %d \r\n", ret);
    ret = cfsetospeed(&options, B57600);
    //printf("cfsetospeed returns %d \r\n", ret);

    /* Enable the receiver and set local mode*/
    options.c_cflag |= (CLOCAL | CREAD);

    /* Set the new options for the port */
    ret = tcsetattr(create_fd, TCSANOW, &options);
    //printf("tcsetattr returns %d\r\n", ret);
    ret = tcgetattr(create_fd, &options);
    startRobot(); sleep(1);
    setMode(FULL); sleep(2);

    /* Read from the socket */
    while (1) {
        nbytes = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);
        if (nbytes == 0)
        {
            perror("Server closed the connection");
            close(clientSocket);
            exit(-1);
        }
        else if (nbytes == -1)
        {
            perror("Socket recv() failed");
            close(clientSocket);
            exit(-1);
        }
        else
        {
            buffer[nbytes] = '\0';
            printf("Received move sequence: %s\n", buffer);
            pthread_create(&roombaThread, NULL, sendCmd, (void *) buffer);
            pthread_join(roombaThread, NULL);
        }
    }

    close(clientSocket);
    close(create_fd);
    return 0;
}

void* sendCmd(void* cmd) {
    char* moveSeq = (char*) cmd;
    executeSeq(moveSeq);
}

void startRobot() {
  data = 0x80;  // 128 = start command
  n = write(create_fd, &data, 1);
  if (n < 0) {
    fprintf(stderr, "write() failed with error %d\r\n", errno);
  }
}

void setMode(char mode) {
  data = mode;
  n = write(create_fd, &data, 1);
  if (n < 0) {
    fprintf(stderr, "write() failed with error %d\r\n", errno);
  }
}

// Change default speed (starts as 200 mm/s)
void setDefaultSpeed(char highbyte, char lowbyte) {
  velocity_highbyte = highbyte;
  velocity_lowbyte = lowbyte;
}
// Drive forever using default speed
void drive(char* dist) {
  char* rad = "0x96";
  char data5[5];
  data5[0] = 0x89; // 137 = drive
  data5[1] = velocity_highbyte;
  data5[2] = velocity_lowbyte;
  data5[3] = 0xF4;
  data5[4] = 0xF3;
  //data5[3] = dist[0];
  //data5[4] = dist[1];
  n = write(create_fd, &data5, 5);
  if (n < 0){
    fprintf(stderr,"write() failed with error %d\r\n", errno);
  }
}

void waitDistance(char dist_highbyte, char dist_lowbyte){
  char data3[3];
  data3[0] = 0x9C; // 156 = wait distance
  data3[1] = dist_highbyte;
  data3[2] = dist_lowbyte;
  n = write(create_fd, &data3, 3);
  if (n < 0){
    fprintf(stderr,"write() failed with error %d\r\n", errno);
  }
}

void backOneStep(){
  reverse();
  waitDistance(0xFE, 0xCF);
  stopRobot();
}

// Velocity = -200 = 0xFF38
// Radius = 500 = 0x01F4
void reverse(){
  char data5[5];
  data5[0] = 0x89; // 137 = drive
  data5[1] = 0xFF; // 255 = reverse
  data5[2] = 0x38; // 56
  data5[3] = 0xF3; // 1
  data5[4] = 0xF4; // 244
  n = write(create_fd, &data5, 5);
  if (n < 0){
    fprintf(stderr,"write() failed with error %d\r\n", errno);
  }
}

// Turn specified direction with specified speed
void turn(char velocity_highbyte, char velocity_lowbyte, char radius_highbyte,
          char radius_lowbyte) {
  char data5[5];
  data5[0] = 0x89;  // 137 = drive
  data5[1] = velocity_highbyte;
  data5[2] = velocity_lowbyte;
  data5[3] = radius_highbyte;
  data5[4] = radius_lowbyte;
  n = write(create_fd, &data5, 5);
  if (n < 0) {
    fprintf(stderr, "write() failed with error %d\r\n", errno);
  }
}

void waitAngle(char angle_highbyte, char angle_lowbyte) {
  char data3[3];
  data3[0] = 0x9D;  // 157 = wait angle
  data3[1] = angle_highbyte;
  data3[2] = angle_lowbyte;
  n = write(create_fd, &data3, 3);
  if (n < 0) {
    fprintf(stderr, "write() failed with error %d\r\n", errno);
  }
}

// Turn left (CCW) at default speed
void turnLeft() {
  printf("turning left..\n");
  turn(velocity_highbyte, velocity_lowbyte, 0x00, 0x01);
  waitAngle(0, 84);  // 84 degrees (looks like 90 degrees)
  stopRobot();
}
// Turn right (CW) at default speed

void turnRight() {
  turn(velocity_highbyte, velocity_lowbyte, 0xFF, 0xFF);
  waitAngle(0xFF, 0xAE);  //-83 degrees (looks like -90 degrees)
  stopRobot();
}

void stopRobot() {
  char data5[5];
  data5[0] = 0x89;  // 137 = drive
  data5[1] = 0x00;
  data5[2] = 0x00;
  data5[3] = 0x00;
  data5[4] = 0x00;
  n = write(create_fd, &data5, 5);
  if (n < 0) {
    fprintf(stderr, "write() failed with error %d\r\n", errno);
  }
}

void executeSeq(char* cmd) {
   //printf("Move sequence: %s composed of %d chars\n", cmd, strlen(cmd) - 1);
   // egs: w5aw10ds4
   //      ddw3dd555s04dds2 -> "ddwddsdd" & [3, 4, 2]

   // TODO: Parse *cmd to attain sleep values for each move cmd (w & s) -> sleepAmts
   //       then strip actual move cmds into moveCmd and iterate, using 
   //       sleepAmts 

    // moveCmd: wawds
    // sleepAmts: 5, 10, 4
    const int MAX_CMDS = 10;
    const int MAX_DURATION_NUM = 4;

    int moveDurations[MAX_CMDS];
    char durationValue[MAX_DURATION_NUM];
    int durationIndex = 0;
    int durationCnt = 0;
    int moveCnt = 0;
    char moveCmd[32] = "";
 
    int recordingDuration = 0; // Flag to differentiate b/w moveCmd and move duration
    int c;
    for (c = 0; c < strlen(cmd) - 1; c++) {
        if (isdigit(cmd[c])) {
            recordingDuration = 1;
            durationValue[durationIndex++] = cmd[c];
            if ((c+1) == strlen(cmd) - 1) { // end of cmd
                durationValue[durationIndex] = '\0';
                moveDurations[durationCnt++] = atoi(durationValue);  
            }
        }

        else if (isalpha(cmd[c])) { 
            if (recordingDuration == 1) {
                durationValue[durationIndex] = '\0'; 
                moveDurations[durationCnt++] = atoi(durationValue);
                durationIndex = 0;
                recordingDuration = 0;
            }
            append(moveCmd, cmd[c]);
            moveCnt++;
        }
        else {
            moveCmd[moveCnt] = '\0';
            return;
        }
    }

    /*int d;
    for (d = 0; d < durationCnt; d++) {
        printf("%d ", moveDurations[d]);
    }
    int m;
    for (m = 0; m < moveCnt; m++) {
        printf("%c", moveCmd[m]);
    }*/

   char* distance = "0x60";
   durationIndex = 0;

   for (c = 0; c < strlen(cmd) - 1; c++) {
        if (!isalpha(cmd[c])) {
            printf("End of sequence.\n");
            return;
        }

        //printf("Sending %c to roomba\n", cmd[c]);
        switch (cmd[c]) {
                case 'w':
                case 'W':
                drive(distance);
                sleep(moveDurations[durationIndex++]);
                stopRobot();
                break;

                case 's':
                case 'S':
                reverse();
                sleep(moveDurations[durationIndex++]);
                stopRobot();
                break;

                case 'a':
                case 'A':
                turnLeft();
                break;

                case 'd':
                case 'D':
                turnRight();
                break;

                case 'f':
                case 'F':
                turnRight();
                turnRight();
                turnRight();
                turnRight();
                    break;

                default:
                    break;
            }
    }
}

void append(char* string, char c) {
    int len = strlen(string);
    string[len] = c;
    string[len + 1] = '\0';
}
