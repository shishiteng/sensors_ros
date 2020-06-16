/*************************************************************************
    > File Name: debug_test.cpp
    > Author: Louis.Qiu
    > Mail: louis.qiu@cloudminds.com
    > Created Time: Fri 22 Jun 2018 10:38:43 PM CST
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



    SocketClient::SocketClient(char* ip,int port)
    {
        this->ip = ip;
        this->port = port;
        //绑定套接字
    }

    ~SocketClient()
  {
    closesocket(clitSock);
  }

    void SocketClient::serConnect()
    {
        clitSock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
        //向服务器发起请求
        sockaddr_in sockAddr;
        memset(&sockAddr, 0, sizeof(sockAddr));  //每个字节都用0填充
        sockAddr.sin_family = PF_INET;
        sockAddr.sin_addr.s_addr = inet_addr(ip);
        sockAddr.sin_port = htons(port);
        connect(clitSock, (SOCKADDR*)&sockAddr, sizeof(SOCKADDR));
    }

    void SocketClient::read(char szBuffer[])
    {
        recv(clitSock, szBuffer, MAXBYTE, NULL);
    }

    void SocketClient::write(char* str)
    {
        send(clitSock, str, strlen(str) + sizeof(char), NULL);
    }


 



