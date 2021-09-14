#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<time.h>
#include<sys/types.h>
#include<errno.h>
#define BAUD 115200 //115200 for JY61 ,9600 for others

static int ret;
static int fd;


int main(void)
{
    static int ret;
    static int fd;

    char r_buf[1024];
    bzero(r_buf,1024);
    FILE *fp;

    //uart_open(variables)
    int fd_open = fd;
    const char *pathname_open = "/dev/ttyUSB0";
    //uart_set(variables)
    struct termios newtio,oldtio;
    int fd_set = fd; int nSpeed = BAUD; int nBits = 8; char nEvent = 'N'; int nStop = 1;
    int fd_setresult;
    //recv_data(variables)
    int fd_recv = fd; char* recv_buffer = r_buf; int length = 44;
    //parsedata(variables)
    char chr;
    float a[3],w[3],Angle[3],h[3];
    static char chrBuf[100];
    static unsigned char chrCnt=0;
    signed short sData[4];
    unsigned char i;
    //uart_close(variables)
    int fd_close;


    //uart_open(functions)
    fd_open = open(pathname_open, O_RDWR|O_NOCTTY);
    if(-1 == fd_open)
    {
        perror("Can't Open Serial Port"); 
        fd = -1;
    }
    else
    printf("open %s success!\n",pathname_open);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 


    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

    //uart_set(functions)
    if  ( tcgetattr( fd_set,&oldtio)  !=  0) 
    {  
        perror("SetupSerial 1");
        printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd_set,&oldtio)); 
        fd_setresult = -1; 
    }
    bzero( &newtio, sizeof( newtio ) ); 
    newtio.c_cflag  |=  CLOCAL | CREAD;  
    newtio.c_cflag &= ~CSIZE;  
    switch( nBits ) 
    { 
    case 7: 
        newtio.c_cflag |= CS7; 
    break; 
    case 8: 
        newtio.c_cflag |= CS8; 
    break; 
    }
    switch( nEvent ) 
    { 
    case 'o':
    case 'O': 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag |= PARODD; 
        newtio.c_iflag |= (INPCK | ISTRIP); 
    break; 
    case 'e':
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP); 
        newtio.c_cflag |= PARENB; 
        newtio.c_cflag &= ~PARODD; 
    break;
    case 'n':
    case 'N': 
        newtio.c_cflag &= ~PARENB; 
    break;
    default:
    break;
    } 

    switch( nSpeed ) 
    { 
    case 2400: 
        cfsetispeed(&newtio, B2400); 
        cfsetospeed(&newtio, B2400); 
    break; 
    case 4800: 
        cfsetispeed(&newtio, B4800); 
        cfsetospeed(&newtio, B4800); 
    break; 
    case 9600: 
        cfsetispeed(&newtio, B9600); 
        cfsetospeed(&newtio, B9600); 
    break; 
    case 115200: 
        cfsetispeed(&newtio, B115200); 
        cfsetospeed(&newtio, B115200); 
    break; 
    case 460800: 
        cfsetispeed(&newtio, B460800); 
        cfsetospeed(&newtio, B460800); 
    break; 
    default: 
        cfsetispeed(&newtio, B9600); 
        cfsetospeed(&newtio, B9600); 
    break; 
    } 
    if( nStop == 1 ) 
    newtio.c_cflag &=  ~CSTOPB; 
    else if ( nStop == 2 ) 
    newtio.c_cflag |=  CSTOPB; 
    newtio.c_cc[VTIME]  = 0; 
    newtio.c_cc[VMIN] = 0; 
    tcflush(fd_set,TCIFLUSH); 

    if((tcsetattr(fd_set,TCSANOW,&newtio))!=0) 
    { 
        perror("com set error"); 
        fd_setresult = -1; 
    } 
        printf("set done!\n"); 
        fd_setresult = 0; 

    if(fd_setresult == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
    }

	fp = fopen("Record.txt","w");

    while(1)
    {
        //recv_data(function)
        length = read(fd_recv, recv_buffer, length);
        ret = length;

        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) 
        {
            fprintf(fp,"%2X ",r_buf[i]);
            //parsedata(function)

            time_t now;
            chrBuf[chrCnt++]=r_buf[i];
            if (chrCnt<11) 
                // break;
            if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)) 
            {
                printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);
                memcpy(&chrBuf[0],&chrBuf[1],10);
                chrCnt--;
                // break;
            }
            memcpy(&sData[0],&chrBuf[2],8);
            switch(chrBuf[1])
            {
                case 0x51:
                    for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
                    time(&now);
                    printf("\r\nT:%s a:%6.3f %6.3f %6.3f ",asctime(localtime(&now)),a[0],a[1],a[2]);
                    
                    break;
                case 0x52:
                    for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
                    printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);					
                    break;
                case 0x53:
                    for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
                    printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
                    break;
                case 0x54:
                    for (i=0;i<3;i++) h[i] = (float)sData[i];
                    printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
                    
                    break;
            }		
            chrCnt=0;		
        }
        usleep(1000);
    }

    // uart_close(function)
    fd_close = fd;
    assert(fd_close);
    close(fd_close);
    ret = 0;

    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
