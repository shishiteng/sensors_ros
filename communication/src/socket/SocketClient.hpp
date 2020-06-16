/*************************************************************************
    > File Name: debug_test.cpp
    > Author: Louis.Qiu
    > Mail: louis.qiu@cloudminds.com
    > Created Time: Fri 22 Jun 2018 10:08:03 PM CST
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h> /* memset */
#include <unistd.h> /* close */
#include <signal.h>


class SocketClient
{
public:
    SocketClient(char* ip,int port);

    SocketClient(){}
    void serConnect();
    ~SocketClient();

    void read(char szBuffer[]);
    void write(char* str);
private:
    int port;
    char* ip;
    int clitSock;
};
 



