#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <mutex>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "iot_bridge_interfaces/msg/iot_message.hpp"

// 클라이언트 정보 구조체
struct ClientInfo
{
    int socket;
    std::string id;
    std::string ip_address;
};

class TcpBridgeNode : public rclcpp::Node
{
public:
    TcpBridgeNode() : Node("tcp_bridge_node")
    {
        // 파라미터 선언 및 로드
        this->declare_parameter<int>("port", 9190);
        this->declare_parameter<int>("backlog", 5);

        this->get_parameter("port", port_);
        this->get_parameter("backlog", backlog_);

        // ROS2 퍼블리셔 / 서브스크라이버
        iot_publisher_ = this->create_publisher<iot_bridge_interfaces::msg::IotMessage>("/iot_to_ros", 10);
        iot_subscriber_ = this->create_subscription<iot_bridge_interfaces::msg::IotMessage>(
            "/ros_to_iot", 10, std::bind(&TcpBridgeNode::ros_to_iot_callback, this, std::placeholders::_1));

        // TCP 서버 설정
        setup_server();

        // 클라이언트 연결 스레드
        server_thread_ = std::thread(&TcpBridgeNode::run_server, this);

        RCLCPP_INFO(this->get_logger(), "✅ TCP IoT Bridge Node started on port %d", port_);
    }

    ~TcpBridgeNode()
    {
        is_running_ = false;
        shutdown(server_socket_, SHUT_RDWR);
        close(server_socket_);
        if (server_thread_.joinable())
            server_thread_.join();

        std::lock_guard<std::mutex> lock(client_mutex_);
        for (const auto &client : client_list_)
            close(client.socket);

        RCLCPP_INFO(this->get_logger(), "🧩 TCP IoT Bridge Node shut down.");
    }

private:
    // 서버 소켓 생성
    void setup_server()
    {
        server_socket_ = socket(PF_INET, SOCK_STREAM, 0);
        if (server_socket_ == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to create socket: %s", strerror(errno));
            rclcpp::shutdown();
        }

        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        serv_addr.sin_port = htons(port_);

        int option = 1;
        setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));

        if (bind(server_socket_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to bind socket: %s", strerror(errno));
            rclcpp::shutdown();
        }

        if (listen(server_socket_, backlog_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to listen on socket: %s", strerror(errno));
            rclcpp::shutdown();
        }
    }

    // 새 클라이언트 접속 처리
    void run_server()
    {
        while (is_running_ && rclcpp::ok())
        {
            struct sockaddr_in clnt_addr;
            socklen_t clnt_addr_size = sizeof(clnt_addr);
            int client_socket = accept(server_socket_, (struct sockaddr *)&clnt_addr, &clnt_addr_size);

            if (client_socket < 0)
            {
                if (is_running_)
                    RCLCPP_WARN(this->get_logger(), "⚠️ Accept failed: %s", strerror(errno));
                continue;
            }

            std::string client_ip = inet_ntoa(clnt_addr.sin_addr);
            RCLCPP_INFO(this->get_logger(), "🔌 New client connected from %s", client_ip.c_str());

            std::thread client_thread(&TcpBridgeNode::handle_client, this, client_socket, client_ip);
            client_thread.detach();
        }
    }

	// === [핵심] 클라이언트 처리 (수정된 전체 함수) ===
	void handle_client(int client_socket, std::string client_ip)
	{
	    // 1️⃣ 클라이언트 인증 (ID 등록) - 기존과 동일
	    char buffer[BUFSIZ];
	    int str_len = read(client_socket, buffer, BUFSIZ - 1);
	    if (str_len <= 0)
	    {
	        RCLCPP_WARN(this->get_logger(), "Failed to read ID from client %s or disconnected.", client_ip.c_str());
	        close(client_socket);
	        return;
	    }
	    buffer[str_len] = '\0';
	
	    std::string init_msg(buffer);
	    std::string client_id;
	    // ID:비밀번호 형식 또는 ID만 있는 형식 모두 처리
	    size_t colon_pos = init_msg.find(':');
	    if (colon_pos != std::string::npos) {
	        client_id = init_msg.substr(1, colon_pos - 1);
	    } else {
	        // '[' 와 ']' 사이의 내용을 ID로 간주
	        size_t end_bracket_pos = init_msg.find(']');
	        if (end_bracket_pos != std::string::npos && end_bracket_pos > 1) {
	            client_id = init_msg.substr(1, end_bracket_pos - 1);
	        } else {
	             RCLCPP_WARN(this->get_logger(), "Invalid handshake format from %s: %s", client_ip.c_str(), init_msg.c_str());
	             close(client_socket);
	             return;
	        }
	    }
	
	    ClientInfo new_client{client_socket, client_id, client_ip};
	    {
	        std::lock_guard<std::mutex> lock(client_mutex_);
	        client_list_.push_back(new_client);
	    }
	
	    RCLCPP_INFO(this->get_logger(), "🆔 Client ID '%s' registered (Socket: %d)", client_id.c_str(), client_socket);
	
	    // 이미지 수신을 위한 유틸리티 함수
	    auto recv_all = [&](int sock, std::vector<char> &out_buffer, int expected_size) -> bool {
	        int total_received = 0;
	        out_buffer.resize(expected_size); // 정확한 크기로 버퍼를 미리 할당
	        while (total_received < expected_size)
	        {
	            int bytes = read(sock, &out_buffer[total_received], expected_size - total_received);
	            if (bytes <= 0) {
	                RCLCPP_ERROR(this->get_logger(), "recv failed or connection closed while receiving image data.");
	                return false; // 읽기 실패 또는 연결 종료
	            }
	            total_received += bytes;
	        }
	        return true;
	    };
	
	    // 2️⃣ 메시지 수신 루프
	    std::string partial_buffer; // 누적 버퍼
	
	    while (is_running_ && rclcpp::ok())
	    {
	        memset(buffer, 0, BUFSIZ);
	        str_len = read(client_socket, buffer, BUFSIZ - 1);
	        if (str_len <= 0)
	        {
	            break; // 클라이언트 연결 종료
	        }
	
	        buffer[str_len] = '\0';
	        partial_buffer.append(buffer, str_len);
	
	        // 버퍼에 '\n'가 포함되어 있다면 계속해서 메시지를 처리
	        while (true)
	        {
	            size_t newline_pos = partial_buffer.find('\n');
	            if (newline_pos == std::string::npos)
	                break; // 완성된 메시지(헤더)가 없으면 루프 종료 후 다시 read 대기
	
	            std::string raw_msg = partial_buffer.substr(0, newline_pos);
	            partial_buffer.erase(0, newline_pos + 1);
	
	            size_t start_pos = raw_msg.find('[');
	            size_t end_pos = raw_msg.find(']');
	            if (start_pos == std::string::npos || end_pos == std::string::npos || end_pos < start_pos)
	            {
	                RCLCPP_WARN(this->get_logger(), "⚠️ Invalid message format from %s: %s",
	                            client_id.c_str(), raw_msg.c_str());
	                continue;
	            }
	
	            std::string dest_id = raw_msg.substr(start_pos + 1, end_pos - start_pos - 1);
	            std::string message_content = raw_msg.substr(end_pos + 1);
	
	            // 3️⃣ {IMG} 헤더 감지 - 여기가 핵심 수정 부분입니다.
	            if (message_content.rfind("{IMG}", 0) == 0)
	            {
	                message_content.erase(0, 5); // "{IMG}" 제거
	                std::istringstream iss(message_content);
	                int img_size = 0;
	                iss >> img_size;
	
	                if (img_size <= 0)
	                {
	                    RCLCPP_WARN(this->get_logger(), "Invalid image size (%d) from %s", img_size, client_id.c_str());
	                    continue;
	                }
	
	                // 1. 최종 이미지 데이터를 담을 벡터를 준비합니다.
	                std::vector<char> image_buffer;
	                image_buffer.reserve(img_size);
	
	                // 2. 헤더 처리 후 partial_buffer에 남아있던 데이터(이미지 데이터의 첫 부분)를 옮깁니다.
	                image_buffer.insert(image_buffer.end(), partial_buffer.begin(), partial_buffer.end());
	                
	                // 3. 앞으로 더 받아야 할 데이터 양을 계산합니다.
	                int remaining_bytes = img_size - image_buffer.size();
	
	                // 4. 받아야 할 데이터가 남아있다면, 딱 그만큼만 추가로 수신합니다.
	                if (remaining_bytes > 0)
	                {
	                    std::vector<char> remaining_data_chunk;
	                    if (!recv_all(client_socket, remaining_data_chunk, remaining_bytes))
	                    {
	                        RCLCPP_WARN(this->get_logger(), "Failed to receive full image from %s", client_id.c_str());
	                        goto client_disconnected; // goto를 사용하여 연결 종료 처리로 점프
	                    }
	                    image_buffer.insert(image_buffer.end(), remaining_data_chunk.begin(), remaining_data_chunk.end());
	                }
	
	                // 이제 image_buffer 에는 완전한 이미지 데이터가 들어있습니다.
	                // 모든 이미지 데이터를 소모했으므로, 다음 메시지를 위해 partial_buffer는 비어있어야 합니다.
	                // 위 로직에서 이미 partial_buffer의 내용을 image_buffer로 옮겼고, 남은 데이터는 read로 채웠으므로
	                // partial_buffer는 자연스럽게 다음 메시지를 기다리는 상태가 됩니다.
	                partial_buffer.clear();
	
	                RCLCPP_INFO(this->get_logger(), "🖼️ Image received from %s (%d bytes), relaying to %s",
	                            client_id.c_str(), (int)image_buffer.size(), dest_id.c_str());
	
	                // 대상 클라이언트로 이미지 전송
	                {
	                    std::lock_guard<std::mutex> lock(client_mutex_);
	                    bool relayed = false;
	                    for (const auto &client : client_list_)
	                    {
	                        if (client.id == dest_id)
	                        {
	                            std::string header = "[" + client_id + "]{IMG}" + std::to_string(image_buffer.size()) + "\n";
	                            write(client.socket, header.c_str(), header.size());
	                            write(client.socket, image_buffer.data(), image_buffer.size());
	                            RCLCPP_INFO(this->get_logger(), "📤 Image relayed from %s to %s", client_id.c_str(), dest_id.c_str());
	                            relayed = true;
	                            break;
	                        }
	                    }
	                    if (!relayed) {
	                        RCLCPP_WARN(this->get_logger(), "Destination client '%s' not found for image relay.", dest_id.c_str());
	                    }
	                }
	                continue; // 이미지 처리가 끝났으므로 다음 메시지 처리로 넘어감
	            }
	
	            // 4️⃣ 일반 텍스트 릴레이 - 기존과 동일
	            RCLCPP_INFO(this->get_logger(), "💬 Relay msg [%s -> %s]: %s",
	                        client_id.c_str(), dest_id.c_str(), message_content.c_str());
	
	            {
	                std::lock_guard<std::mutex> lock(client_mutex_);
	                bool relayed = false;
	                for (const auto &client : client_list_)
	                {
	                    if (client.id == dest_id || dest_id == "ALLMSG")
	                    {
	                        std::string forward_msg = "[" + client_id + "]" + message_content + "\n";
	                        write(client.socket, forward_msg.c_str(), forward_msg.size());
	                        relayed = true;
	                        if (dest_id != "ALLMSG") break;
	                    }
	                }
	                 if(relayed) {
	                    RCLCPP_INFO(this->get_logger(), "➡️ Message relayed from %s to %s", client_id.c_str(), dest_id.c_str());
	                 }
	            }
	
	            // 5️⃣ ROS 토픽으로 발행 - 기존과 동일
	            auto msg = iot_bridge_interfaces::msg::IotMessage();
	            msg.sender_id = client_id;
	            msg.destination_id = dest_id;
	            msg.message = message_content;
	            iot_publisher_->publish(msg);
	        }
	    }
	
	// 클라이언트 연결 종료 시 정리 로직
	client_disconnected:
	    RCLCPP_INFO(this->get_logger(), "🔚 Client %s (%s) disconnected.", client_id.c_str(), client_ip.c_str());
	    std::lock_guard<std::mutex> lock(client_mutex_);
	    client_list_.erase(
	        std::remove_if(client_list_.begin(), client_list_.end(),
	                       [client_socket](const ClientInfo &client)
	                       { return client.socket == client_socket; }),
	        client_list_.end());
	    close(client_socket);
	}

    // === [ROS → 클라이언트 전송] ===
    void ros_to_iot_callback(const iot_bridge_interfaces::msg::IotMessage::SharedPtr msg)
    {
        std::string formatted_msg = "[" + msg->sender_id + "]" + msg->message + "\n";

        std::lock_guard<std::mutex> lock(client_mutex_);
        if (msg->destination_id == "ALLMSG")
        {
            for (const auto &client : client_list_)
                write(client.socket, formatted_msg.c_str(), formatted_msg.length());
        }
        else
        {
            for (const auto &client : client_list_)
            {
                if (client.id == msg->destination_id)
                {
                    write(client.socket, formatted_msg.c_str(), formatted_msg.length());
                    break;
                }
            }
        }
    }

    // === 멤버 변수 ===
    int port_;
    int backlog_;
    int server_socket_;
    std::vector<ClientInfo> client_list_;
    std::mutex client_mutex_;
    std::thread server_thread_;
    std::atomic<bool> is_running_{true};

    rclcpp::Publisher<iot_bridge_interfaces::msg::IotMessage>::SharedPtr iot_publisher_;
    rclcpp::Subscription<iot_bridge_interfaces::msg::IotMessage>::SharedPtr iot_subscriber_;
};

// === main ===
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

