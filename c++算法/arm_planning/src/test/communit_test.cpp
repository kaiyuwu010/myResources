#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>


class UDPServer {
public:
    int num=0;
    UDPServer(const std::string& address, int port) {
        server_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (server_fd < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            exit(EXIT_FAILURE);
        }

        memset(&server_addr, 0, sizeof(server_addr));
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);

        if (bind(server_fd, (const struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Bind failed" << std::endl;
            close(server_fd);
            exit(EXIT_FAILURE);
        }
    }

    ~UDPServer() {
        close(server_fd);
    }

    void receiveMessages() {
        char buffer[1024];
        struct sockaddr_in client_addr;
        socklen_t len = sizeof(client_addr);

        while (true) {
            num++;
            int n = recvfrom(server_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&client_addr, &len);
            if (n < 0) {
                std::cerr << "Receive failed" << std::endl;
                break;
            }
            buffer[n] = '\0';
            std::cout << "!!!!!!!!!!!!!!!Received message: " << num << "个message"<<buffer<<std::endl;
        }
    }

private:
    int server_fd;
    struct sockaddr_in server_addr;
};

int main() {
    UDPServer server("192.168.1.100", 8099);
    server.receiveMessages();
    return 0;
    // 创建UDP套接字
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed." << std::endl;
        return 1;
    }

    // 服务器地址和端口
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(49001); // 设置服务器端口
    server_addr.sin_addr.s_addr = INADDR_ANY; // 接受任意IP地址

    // 绑定套接字到地址和端口
    if (bind(sock, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Binding failed." << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "Server is running and waiting for data..." << std::endl;

    // 客户端地址结构
    sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    // 缓冲区用于存储接收到的数据
    char buffer[1024];
    int i =0;
    while (true) {
        // 接收数据
        int bytes_received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, (sockaddr*)&client_addr, &client_addr_len);
        if (bytes_received < 0) {
            std::cerr << "Receive failed." << std::endl;
            break;
        }

        buffer[bytes_received] = '\0'; // 添加字符串终止符
        // std::cout << "Received: " << buffer << std::endl;

        // 发送响应回客户端
        const char* response = "Hello, UDP Client!";
        int response_len = strlen(response);
        if (sendto(sock, response, response_len, 0, (sockaddr*)&client_addr, client_addr_len) < 0) {
            std::cerr << "Send failed." << std::endl;
            break;
        }
        i++;
        std::cout << "Received: " << i <<"个数据"<< std::endl;
    }

    // 关闭套接字
    close(sock);
    return 0;
}