#include<stdio.h>      
#include<stdlib.h>     
#include<unistd.h>     
#include<sys/types.h>  
#include<sys/stat.h>   
#include<fcntl.h>      
#include<termios.h>    
#include<errno.h>      
#include<string.h>

#include"uart.h"


#define FALSE  -1
#define TRUE   0



//Open the serial port
//Returns the value of the serial device file description
//Right back to 3, an error is returned to -1
int UART_Open(void)
{
        int fd;
	char *dev0 ="/dev/ttyUSB0";
        char *dev1 ="/dev/ttyUSB1";
        char *dev2 ="/dev/ttyUSB2";
  	fd = open(dev0, O_RDWR);//|O_NOCTTY|O_NDELAY
        //printf("fd:%d\n",fd);
  	if (FALSE == fd)
        {
		perror("Can't Open Serial Port");
                exit(1); 
  		
  		}
        printf("Open Serial Port!\n");
	// Determine whether the serial interface is blocked
  	if(fcntl(fd, F_SETFL, 0) < 0)
        {
		printf("fcntl failed!\n");
                exit(1); 	
  		} 
        else 
        {
       		printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
  		}
	// Test whether the terminal device
  	if(0 == isatty(STDIN_FILENO))
        {
  		printf("standard input is not a terminal device\n");
                exit(1); 
  		}
  	return fd;
}

//Close the serial port
void UART_Close(int fd)
{
	close(fd);
}

//Serial port parameters
int UART_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{ 
    
    int   i; 
 
    int   speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
          		      B38400, B19200, B9600, B4800, B2400, B1200, B300 
			     };
    int   name_arr[] = {
			    38400,  19200,  9600,  4800,  2400,  1200,  300, 38400,  
          		    19200,  9600, 4800, 2400, 1200,  300 
			   };  
	struct termios options; 


	if(tcgetattr( fd,&options)  !=  0)
       {  
	    perror("SetupSerial 1");     
	    return(FALSE);  
    	}

	//Set the baud serial input and output baud rates
	for(i= 0;i < sizeof(speed_arr) / sizeof(int);i++)
	{ 	
		if  (speed == name_arr[i]) 
		{        
      			cfsetispeed(&options, speed_arr[i]);  
      			cfsetospeed(&options, speed_arr[i]);   
		}
    }
	// Modify the control mode to ensure that the program will not take up the serial port
	options.c_cflag |= CLOCAL;
	// Modify the control mode makes it possible to read the input data from the serial port
	options.c_cflag |= CREAD;
	// Set data flow control
	switch(flow_ctrl)
	{
		case 0 :// Do not use flow control
			options.c_cflag &= ~CRTSCTS;
			break;	
    	case 1 :// Use hardware flow control
    		options.c_cflag |= CRTSCTS;
    		break;
    	case 2 :// Use software flow control
    		options.c_cflag |= IXON | IXOFF | IXANY;
    		break;
	}
    // Set the data bits
	options.c_cflag &= ~CSIZE; // Masked by other flag bits
	switch (databits)
	{   
		case 5 :
    			options.c_cflag |= CS5;
    			break;
    		case 6	:
    			options.c_cflag |= CS6;
    			break;
    		case 7	:     
        		options.c_cflag |= CS7; 
        		break;
    		case 8:     
        		options.c_cflag |= CS8;
        		break;   
       		default:    
        		fprintf(stderr,"Unsupported data size\n"); 
        		return (FALSE);
	}
	// Set the parity bit
	switch (parity)
	{   
		case 'n':
    		case 'N': 
        		options.c_cflag &= ~PARENB;  
        		options.c_iflag &= ~INPCK;     
        		break;  
    		case 'o':   
    		case 'O':    
        		options.c_cflag |= (PARODD | PARENB);  
        		options.c_iflag |= INPCK;              
        		break;  
    		case 'e':  
    		case 'E':   
        		options.c_cflag |= PARENB;        
        		options.c_cflag &= ~PARODD;        
        		options.c_iflag |= INPCK;       
        		break;
    		case 's': 
    		case 'S': 
        		options.c_cflag &= ~PARENB;
        		options.c_cflag &= ~CSTOPB;
        		break;  
        	default:   
        		fprintf(stderr,"Unsupported parity\n");    
        		return (FALSE); 
	}  
	// Set the stop bits
	switch (stopbits)
	{   
		case 1:    
			options.c_cflag &= ~CSTOPB;  
        		break;  
    		case 2:    
        		options.c_cflag |= CSTOPB;  
       			break;
    		default:    
         		fprintf(stderr,"Unsupported stop bits\n");  
         		return (FALSE); 
	} 
        //local mode flags
        options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG);
       //Modify the output mode, raw data output
        options.c_oflag &= ~OPOST; 
	// Set the wait time and minimum receive characters
	options.c_cc[VTIME] = 0;    //Reads a character waiting 1 * (1/10) s
	options.c_cc[VMIN] = 4;    //The minimum number of characters to read 1

	// If data overflow occurs, the receive data, but no longer read
	tcflush(fd,TCIFLUSH);

	// Activate the configuration (set termios modified data to the serial port)
	if(tcsetattr(fd,TCSANOW,&options) != 0)
	{ 
		perror("com set error!\n");   
    		return (FALSE);  
	} 
	return (TRUE + 1);  
}

//Serial port initialization
int UART_Init(int fd, int speed,int flow_ctrlint ,int databits,int stopbits,char parity)
{
	
	if (FALSE == UART_Set(fd,speed,flow_ctrlint,databits,stopbits,parity)) 
	{    		
	  printf("Set Port Error\n");		
          exit(1); 
    	} 
	else 
	{
           printf("Set Port OK!\n");	
           return  (TRUE + 1) ;
   	}
}



//To receive serial data
//recv_buf: Receiving serial data stored in the buffer recvbuf
//data_len: length of one frame of data
int UART_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;
    
    struct timeval time;
    
    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);
    
    time.tv_sec = 0;
    time.tv_usec = 0;
    // Use select to achieve serial multiplex communication
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    if(fs_sel)
	{
	    len = read(fd,rcv_buf,data_len);
            if(rcv_buf[0] == 'o' && rcv_buf[3] == 's')
            {	
	      return len;
             }
    	}
	else {
		return FALSE;
	}	
}

//send data
int UART_Send(int fd, char *send_buf,int data_len)
{
    int ret;
    
    ret = write(fd,send_buf,data_len);
    if (data_len == ret )
	{	
	    return ret;
    }
	else 
	{    
	    tcflush(fd,TCOFLUSH);    
	    return FALSE;
        
    }
    
}

int i;
float Hex_To_Decimal(unsigned char *Byte,int num)//十六进制到浮点数
{
	char cByte[4];
	for ( i=0;i<num;i++)
	{
	cByte[i] = Byte[i];
	}

	float pfValue=*(float*)&cByte;
return pfValue;
}

unsigned short count_CRC(unsigned char *addr, int num)
{
	unsigned short CRC = 0xFFFF;
	while (num--)
	{
		CRC ^= *addr++;
		for (i = 0; i < 8; i++)
		{
			if (CRC & 1)
			{
				CRC >>= 1;
				CRC ^= 0xA001;
			}
			else
			{
				CRC >>= 1;
			}
		}
	}
	return CRC;
}

