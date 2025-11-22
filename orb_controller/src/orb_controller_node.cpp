#include <ros/ros.h>
#include <std_msgs/Header.h>
#include "orb_controller/ezpd_bcr_i2c.h"
#include "orb_controller/PowerStatus.h"
#include "orb_controller/PDStatus.h"
#include "orb_controller/SetVoltageCurrent.h"
#include "orb_controller/GetStatus.h"
#include "orb_controller/RequestPDRole.h"

class ORBControllerNode {
public:
    ORBControllerNode() : nh_("~") {
        // Get parameters from ROS parameter server
        int i2c_bus;
        int i2c_addr;
        double publish_rate;
        
        nh_.param("i2c_bus", i2c_bus, 1);
        nh_.param("i2c_address", i2c_addr, 0x08);
        nh_.param("publish_rate", publish_rate, 10.0);
        
        // Initialize I2C interface
        i2c_interface_ = std::make_unique<EZPD_BCR_I2C>(i2c_bus, static_cast<uint8_t>(i2c_addr));
        
        if (!i2c_interface_->initialize()) {
            ROS_ERROR("Failed to initialize I2C interface: %s", i2c_interface_->getLastError().c_str());
            ros::shutdown();
            return;
        }
        
        ROS_INFO("Successfully initialized I2C interface on bus %d, address 0x%02X", i2c_bus, i2c_addr);
        
        // Setup publishers
        power_status_pub_ = nh_.advertise<orb_controller::PowerStatus>("power_status", 10);
        pd_status_pub_ = nh_.advertise<orb_controller::PDStatus>("pd_status", 10);
        
        // Setup services
        set_voltage_current_srv_ = nh_.advertiseService("set_voltage_current", 
            &ORBControllerNode::setVoltageCurrentCallback, this);
        get_status_srv_ = nh_.advertiseService("get_status", 
            &ORBControllerNode::getStatusCallback, this);
        request_pd_role_srv_ = nh_.advertiseService("request_pd_role", 
            &ORBControllerNode::requestPDRoleCallback, this);
        
        // Setup timer for periodic status publishing
        status_timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate), 
            &ORBControllerNode::statusTimerCallback, this);
        
        ROS_INFO("ORB Controller Node initialized successfully");
    }
    
    ~ORBControllerNode() {
        ROS_INFO("Shutting down ORB Controller Node");
    }
    
private:
    void statusTimerCallback(const ros::TimerEvent& event) {
        publishPowerStatus();
        publishPDStatus();
    }
    
    void publishPowerStatus() {
        orb_controller::PowerStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "orb_controller";
        
        // Read voltage and current from the chip
        uint16_t voltage_mv, current_ma;
        uint8_t pd_status;
        
        if (i2c_interface_->getVoltageStatus(voltage_mv)) {
            msg.voltage_mv = voltage_mv;
        } else {
            ROS_WARN_THROTTLE(5, "Failed to read voltage status");
            msg.voltage_mv = 0;
        }
        
        if (i2c_interface_->getCurrentStatus(current_ma)) {
            msg.current_ma = current_ma;
        } else {
            ROS_WARN_THROTTLE(5, "Failed to read current status");
            msg.current_ma = 0;
        }
        
        // Calculate power in milliwatts
        msg.power_mw = (static_cast<uint32_t>(msg.voltage_mv) * msg.current_ma) / 1000;
        
        if (i2c_interface_->getPDStatus(pd_status)) {
            msg.pd_active = (pd_status & 0x01) != 0;  // Bit 0 indicates PD active (example)
            msg.pd_profile = (pd_status >> 4) & 0x07; // Bits 4-6 for profile (example)
        } else {
            ROS_WARN_THROTTLE(5, "Failed to read PD status");
            msg.pd_active = false;
            msg.pd_profile = 0;
        }
        
        power_status_pub_.publish(msg);
    }
    
    void publishPDStatus() {
        orb_controller::PDStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "orb_controller";
        
        uint8_t pd_status, port_status;
        
        if (i2c_interface_->getPDStatus(pd_status)) {
            msg.pd_status_raw = pd_status;
            
            // Parse PD status bits (these are examples - adjust based on actual datasheet)
            msg.pd_contract_active = (pd_status & 0x01) != 0;
            msg.current_profile = (pd_status >> 4) & 0x07;
        } else {
            ROS_WARN_THROTTLE(5, "Failed to read PD status");
            msg.pd_status_raw = 0;
            msg.pd_contract_active = false;
            msg.current_profile = 0;
        }
        
        if (i2c_interface_->getPortStatus(port_status)) {
            msg.port_status_raw = port_status;
            
            // Parse port status bits (example - adjust based on datasheet)
            msg.type_c_connected = (port_status & 0x01) != 0;
        } else {
            ROS_WARN_THROTTLE(5, "Failed to read port status");
            msg.port_status_raw = 0;
            msg.type_c_connected = false;
        }
        
        // Get negotiated values
        uint16_t voltage_mv, current_ma;
        if (i2c_interface_->getVoltageStatus(voltage_mv)) {
            msg.negotiated_voltage_mv = voltage_mv;
        } else {
            msg.negotiated_voltage_mv = 0;
        }
        
        if (i2c_interface_->getCurrentStatus(current_ma)) {
            msg.negotiated_current_ma = current_ma;
        } else {
            msg.negotiated_current_ma = 0;
        }
        
        pd_status_pub_.publish(msg);
    }
    
    bool setVoltageCurrentCallback(orb_controller::SetVoltageCurrent::Request& req,
                                    orb_controller::SetVoltageCurrent::Response& res) {
        ROS_INFO("Setting voltage to %d mV, current to %d mA", req.voltage_mv, req.current_ma);
        
        if (i2c_interface_->setVoltageCurrent(req.voltage_mv, req.current_ma)) {
            res.success = true;
            res.message = "Successfully set voltage and current";
            ROS_INFO("Successfully set voltage and current");
        } else {
            res.success = false;
            res.message = i2c_interface_->getLastError();
            ROS_ERROR("Failed to set voltage and current: %s", res.message.c_str());
        }
        
        return true;
    }
    
    bool getStatusCallback(orb_controller::GetStatus::Request& req,
                          orb_controller::GetStatus::Response& res) {
        uint16_t voltage_mv, current_ma;
        uint8_t pd_status, port_status;
        
        bool success = true;
        
        if (i2c_interface_->getVoltageStatus(voltage_mv)) {
            res.voltage_mv = voltage_mv;
        } else {
            res.voltage_mv = 0;
            success = false;
        }
        
        if (i2c_interface_->getCurrentStatus(current_ma)) {
            res.current_ma = current_ma;
        } else {
            res.current_ma = 0;
            success = false;
        }
        
        res.power_mw = (static_cast<uint32_t>(res.voltage_mv) * res.current_ma) / 1000;
        
        if (i2c_interface_->getPDStatus(pd_status)) {
            res.pd_status_raw = pd_status;
            res.pd_active = (pd_status & 0x01) != 0;
            res.pd_profile = (pd_status >> 4) & 0x07;
        } else {
            res.pd_status_raw = 0;
            res.pd_active = false;
            res.pd_profile = 0;
            success = false;
        }
        
        if (i2c_interface_->getPortStatus(port_status)) {
            res.port_status_raw = port_status;
        } else {
            res.port_status_raw = 0;
            success = false;
        }
        
        res.success = success;
        if (success) {
            res.message = "Status retrieved successfully";
        } else {
            res.message = "Some status values could not be read: " + i2c_interface_->getLastError();
        }
        
        return true;
    }
    
    bool requestPDRoleCallback(orb_controller::RequestPDRole::Request& req,
                               orb_controller::RequestPDRole::Response& res) {
        ROS_INFO("Requesting PD profile %d", req.profile);
        
        if (req.profile < 1 || req.profile > 5) {
            res.success = false;
            res.message = "Invalid profile. Must be 1-5 (1=5V, 2=9V, 3=12V, 4=15V, 5=20V)";
            ROS_ERROR("%s", res.message.c_str());
            return true;
        }
        
        if (i2c_interface_->requestPDRole(req.profile)) {
            res.success = true;
            res.message = "Successfully requested PD profile";
            ROS_INFO("Successfully requested PD profile %d", req.profile);
        } else {
            res.success = false;
            res.message = i2c_interface_->getLastError();
            ROS_ERROR("Failed to request PD profile: %s", res.message.c_str());
        }
        
        return true;
    }
    
private:
    ros::NodeHandle nh_;
    std::unique_ptr<EZPD_BCR_I2C> i2c_interface_;
    
    ros::Publisher power_status_pub_;
    ros::Publisher pd_status_pub_;
    
    ros::ServiceServer set_voltage_current_srv_;
    ros::ServiceServer get_status_srv_;
    ros::ServiceServer request_pd_role_srv_;
    
    ros::Timer status_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "orb_controller_node");
    
    try {
        ORBControllerNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in ORB Controller Node: %s", e.what());
        return 1;
    }
    
    return 0;
}