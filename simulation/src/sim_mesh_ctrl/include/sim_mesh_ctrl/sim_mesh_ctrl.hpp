#ifndef _SIM_MESH_CTRL_HPP_
#define _SIM_MESH_CTRL_HPP_

#include <sys/types.h>
#include <sys/socket.h>
#include <rclcpp/rclcpp.hpp>

class SimMeshCtrlPrivate;

class SimMeshCtrl : public rclcpp::Node
{
public:
    SimMeshCtrl();
    virtual ~SimMeshCtrl();

    void udp_poll();
    void get_address(std::string &addr, uint16_t &port);
private:
    std::unique_ptr<SimMeshCtrlPrivate> _impl;
};

#endif // _SIM_MESH_CTRL_HPP_
