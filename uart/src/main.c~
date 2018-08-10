
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

unsigned char hexbyte[4]={0};


int main (int argc,char* argv[])
{
  int my_fd;
  
  unsigned char rcv_buf[8];
  unsigned char send_buf[8]={0x01,0x03,0x00,0x80,0xfa,0x00,0x06,0x82};

  
  
  int len,i;
  float Hdecimal=0.0;
   //uart initialization
    my_fd = UART_Open(); 
    UART_Init(my_fd,9600,0,8,1,'N');
    //bzero(( unsigned char*)&temp_buff,1);


  while(1) 
  {
    // UART_Send(my_fd,"ooks",4);
      UART_Send(my_fd,  send_buf,8);
     len=UART_Recv(my_fd, rcv_buf,8);
   //  if(len==4 || rcv_buf[0]== 1 || rcv_buf[3]== 4)	
   //   {
      //   printf("%d\n",len);
      printf("---------%d\n",len);
      printf("1=%x\n",rcv_buf[0]);
      printf("2=%x\n",rcv_buf[1]);
		  printf("3=%x\n",rcv_buf[2]);
		  printf("4=%x\n",rcv_buf[3]); 
      printf("5=%x\n",rcv_buf[4]);
      printf("6=%x\n",rcv_buf[5]);
		  printf("7=%x\n",rcv_buf[6]);
		  printf("8=%x\n",rcv_buf[7]); 
    //  }
      
    //   for(i=0;i<3;i++)
    //  {
    //    hexbyte[i] = rcv_buf[i];
    //  }
    //   Hdecimal=Hex_To_Decimal(hexbyte,sizeof(hexbyte));//十六进制转换为浮点数
    //   printf("FUFU %f\n",Hdecimal);
    }
   UART_Close(my_fd); 
  return 0;
}
