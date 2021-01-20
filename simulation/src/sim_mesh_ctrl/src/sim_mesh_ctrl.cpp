
#include "sim_mesh_ctrl/sim_mesh_ctrl.hpp"
#include <thread>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;

#define BUFFER_SIZE 2048

struct DroneLocation
{
    std::string name;
    double x;
    double y;
    double z;
};

class SimMeshCtrlPrivate
{
public:

    int udp_open();
    void handle_msg();
    void calculate_distances();

public:
    SimMeshCtrl *_node;

    // Node parameters
    uint16_t _udp_port;
    std::string _udp_address;

    // UDP Socket data & buffer
    int _sock;
    struct sockaddr_in _in_addr;
    uint8_t _buffer[BUFFER_SIZE];
    size_t _buf_size;

    // Mesh distance data
    std::vector<struct DroneLocation> _drones;
    std::vector<std::pair<std::string, double>> _distances;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
};

int SimMeshCtrlPrivate::udp_open()
{
    _sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (_sock < 0) {
        RCLCPP_ERROR(this->_node->get_logger(), "ERROR opening socket: %d", _sock);
        exit(EXIT_FAILURE);
    }
    int broadcast = 1;
    int ret = setsockopt(_sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    if (ret < 0)
    {
        perror("broadcast set fail");
        RCLCPP_ERROR(this->_node->get_logger(), "ERROR setting Broadcast option: %d", ret);
        close(_sock);
        exit(EXIT_FAILURE);
    }

    _in_addr.sin_family = AF_INET;
    _in_addr.sin_port = htons(_udp_port);
    _in_addr.sin_addr.s_addr = inet_addr(_udp_address.c_str());

    if (bind(_sock, (struct sockaddr *) &_in_addr, sizeof(_in_addr)) < 0) {
        RCLCPP_ERROR(this->_node->get_logger(), "ERROR binding port %u: %d", _udp_port, _sock);
        exit(EXIT_FAILURE);
    }
    RCLCPP_INFO(this->_node->get_logger(), "Bind to %s:%u", _udp_address.c_str(), _udp_port);
    _buf_size = 0;

    return 0;
}

void SimMeshCtrlPrivate::calculate_distances()
{
    std::vector<std::pair<std::string, double>> distances;
    std::string myname = this->_node->get_namespace();

    if(myname.at(0) == '/')
    {
        myname.erase(myname.begin());
    }
    DroneLocation me;
    for (auto it = _drones.begin(); it != _drones.end();)
    {
        if (myname.compare(it->name) == 0)
        {
            me = *it;
            _drones.erase(it);
            break;
        } else {
            ++it;
        }
    }

    auto message = std_msgs::msg::String();
    _distances.clear();
    RCLCPP_INFO(this->_node->get_logger(), "Myself:  %s: x(%f) y(%f) z(%f)", me.name.c_str(), me.x, me.y, me.z);
    RCLCPP_INFO(this->_node->get_logger(), "Distances:");
    for (struct DroneLocation  dr : _drones)
    {
        double dist = std::sqrt((dr.x - me.x)*(dr.x - me.x) + (dr.y - me.y)*(dr.y - me.y) + (dr.z - me.z)*(dr.z - me.z));
        RCLCPP_INFO(this->_node->get_logger(), "         %s: x(%f) y(%f z(%f) -> Dist: %f", dr.name.c_str(), dr.x, dr.y, dr.z,dist);
        _distances.push_back(std::pair<std::string, double>(dr.name, dist));
        if (!message.data.empty())
            message.data += ", ";
        message.data += "{ \"name\": \"" + dr.name + "\", \"dist\": \"" + std::to_string(dist) + "\" }";
    }
    message.data = "{ \"positions\": [ " + message.data + " ] }";

    RCLCPP_INFO(this->_node->get_logger(), "Publishing: '%s'", message.data.c_str());
    _publisher->publish(message);
}

void SimMeshCtrlPrivate::handle_msg()
{
    _buffer[_buf_size] = 0;
    if (json::accept((char*)_buffer)) {
        json j = json::parse((char*)_buffer);
        json j_array = j["positions"];
        if (! j_array.is_null()) {
            int size = j_array.size();
            _drones.clear();
            RCLCPP_INFO(this->_node->get_logger(), "Drones:");
            for (int i=0; i<size; i++)
            {
                json j_drone = j_array[i];
                if (!j_drone["name"].is_null() &&
                    !j_drone["x"].is_null() &&
                    !j_drone["y"].is_null() &&
                    !j_drone["z"].is_null())
                {
                    struct DroneLocation drone;
                    drone.name = std::string(j_drone["name"]);
                    drone.x = std::stod(std::string(j_drone["x"]));
                    drone.y = std::stod(std::string(j_drone["y"]));
                    drone.z = std::stod(std::string(j_drone["z"]));
                    RCLCPP_INFO(this->_node->get_logger(), "       %s: x(%f) y(%f z(%f)", drone.name.c_str(), drone.x, drone.y, drone.z);
                    _drones.push_back(drone);
                }
            }

            calculate_distances();
        }
    } else {
        RCLCPP_WARN(this->_node->get_logger(), "Invalid json string, ignore");
    }
}


SimMeshCtrl::SimMeshCtrl()
: Node("SimMeshCtrl"),
  _impl(std::make_unique<SimMeshCtrlPrivate>())
{
    RCLCPP_INFO(this->get_logger(), "Simulation Mesh Control");
    _impl->_node = this;

    this->declare_parameter<int>("port", 14000);
    this->declare_parameter<std::string>("address", "0.0.0.0");
    this->get_parameter("port", _impl->_udp_port);
    this->get_parameter("address", _impl->_udp_address);

    RCLCPP_INFO(this->get_logger(), "udp_port:     %d",  _impl->_udp_port);
    RCLCPP_INFO(this->get_logger(), "udp_address: '%s'", _impl->_udp_address.c_str());

    _impl->udp_open();
    _impl->_publisher = this->create_publisher<std_msgs::msg::String>("mesh_dist", 10);
}

SimMeshCtrl::~SimMeshCtrl()
{
    RCLCPP_INFO(this->get_logger(), "Destructor");
}

void SimMeshCtrl::udp_poll()
{
    int ret = recv(_impl->_sock, _impl->_buffer, sizeof(_impl->_buffer) - 1, 0);
    if (ret > 0)
    {
        _impl->_buf_size = ret;
        if (memcmp(_impl->_buffer, "end", 3) == 0)
        {
            return;
        }
        _impl->handle_msg();
    }
}

void SimMeshCtrl::get_address(std::string &addr, uint16_t &port)
{
    port = _impl->_udp_port;
    addr = _impl->_udp_address;
    if (addr.compare("0.0.0.0") == 0)
    {
        addr = "127.0.0.1";
    }
}

void udp_worker(std::shared_ptr<SimMeshCtrl> node)
{
    while (rclcpp::ok())
    {
        node->udp_poll();
    }
}

void udp_terminate(std::string address, uint16_t port)
{
    struct sockaddr_in out;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        exit(-1);
    }

    out.sin_family = AF_INET;
    out.sin_port = htons(port);
    out.sin_addr.s_addr = inet_addr(address.c_str());
    char buf[6];
    sprintf(buf, "end");
    sendto(sock, buf, 3, 0, (struct sockaddr *)&out, sizeof(out));
}

int main(int argc, char * argv[])
{
    std::string address;
    uint16_t port;

    rclcpp::init(argc, argv);
    std::shared_ptr<SimMeshCtrl> node = std::make_shared<SimMeshCtrl>();

    node->get_address(address, port);

    std::thread udp_listener(udp_worker, node);
    rclcpp::spin(node);
    // Release udp_listener by sending datagram
    udp_terminate(address, port);
    udp_listener.join();
    rclcpp::shutdown();
    return 0;
}
