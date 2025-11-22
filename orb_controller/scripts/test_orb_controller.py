#!/usr/bin/env python3
"""
Test script for ORB Controller ROS node
This script demonstrates how to interact with the ORB controller services and topics
"""

import rospy
from orb_controller.msg import PowerStatus, PDStatus
from orb_controller.srv import GetStatus, SetVoltageCurrent, RequestPDRole
import time
import sys

class ORBControllerTest:
    def __init__(self):
        rospy.init_node('orb_controller_test', anonymous=True)
        
        # Subscribe to status topics
        self.power_status_sub = rospy.Subscriber('/orb/power_status', PowerStatus, self.power_status_callback)
        self.pd_status_sub = rospy.Subscriber('/orb/pd_status', PDStatus, self.pd_status_callback)
        
        # Setup service proxies
        rospy.wait_for_service('/orb_controller/get_status', timeout=5)
        rospy.wait_for_service('/orb_controller/set_voltage_current', timeout=5)
        rospy.wait_for_service('/orb_controller/request_pd_role', timeout=5)
        
        self.get_status_srv = rospy.ServiceProxy('/orb_controller/get_status', GetStatus)
        self.set_voltage_current_srv = rospy.ServiceProxy('/orb_controller/set_voltage_current', SetVoltageCurrent)
        self.request_pd_role_srv = rospy.ServiceProxy('/orb_controller/request_pd_role', RequestPDRole)
        
        self.latest_power_status = None
        self.latest_pd_status = None
        
        rospy.loginfo("ORB Controller Test initialized")
    
    def power_status_callback(self, msg):
        """Callback for power status messages"""
        self.latest_power_status = msg
        rospy.loginfo(f"Power Status - Voltage: {msg.voltage_mv}mV, Current: {msg.current_ma}mA, Power: {msg.power_mw}mW")
    
    def pd_status_callback(self, msg):
        """Callback for PD status messages"""
        self.latest_pd_status = msg
        rospy.loginfo(f"PD Status - Connected: {msg.type_c_connected}, Contract: {msg.pd_contract_active}, Profile: {msg.current_profile}")
    
    def test_get_status(self):
        """Test getting current status"""
        try:
            rospy.loginfo("Testing get_status service...")
            response = self.get_status_srv()
            
            if response.success:
                rospy.loginfo("Get Status successful:")
                rospy.loginfo(f"  Voltage: {response.voltage_mv} mV")
                rospy.loginfo(f"  Current: {response.current_ma} mA")
                rospy.loginfo(f"  Power: {response.power_mw} mW")
                rospy.loginfo(f"  PD Active: {response.pd_active}")
                rospy.loginfo(f"  PD Profile: {response.pd_profile}")
                rospy.loginfo(f"  PD Status Raw: 0x{response.pd_status_raw:02X}")
                rospy.loginfo(f"  Port Status Raw: 0x{response.port_status_raw:02X}")
            else:
                rospy.logwarn(f"Get Status failed: {response.message}")
            
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
    
    def test_request_pd_role(self, profile):
        """Test requesting a specific PD profile"""
        try:
            rospy.loginfo(f"Testing request_pd_role service with profile {profile}...")
            response = self.request_pd_role_srv(profile)
            
            if response.success:
                rospy.loginfo(f"Request PD Role successful: {response.message}")
            else:
                rospy.logwarn(f"Request PD Role failed: {response.message}")
            
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
    
    def test_set_voltage_current(self, voltage_mv, current_ma):
        """Test setting voltage and current"""
        try:
            rospy.loginfo(f"Testing set_voltage_current service: {voltage_mv}mV, {current_ma}mA...")
            response = self.set_voltage_current_srv(voltage_mv, current_ma)
            
            if response.success:
                rospy.loginfo(f"Set Voltage/Current successful: {response.message}")
            else:
                rospy.logwarn(f"Set Voltage/Current failed: {response.message}")
            
            return response
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return None
    
    def run_full_test(self):
        """Run a full test sequence"""
        rospy.loginfo("="*50)
        rospy.loginfo("Starting ORB Controller Full Test")
        rospy.loginfo("="*50)
        
        # Test 1: Get initial status
        rospy.loginfo("\n--- Test 1: Get Initial Status ---")
        self.test_get_status()
        time.sleep(1)
        
        # Test 2: Request different PD profiles
        rospy.loginfo("\n--- Test 2: Request PD Profiles ---")
        profiles = [
            (1, "5V"),
            (2, "9V"),
            (3, "12V"),
            (4, "15V"),
            (5, "20V")
        ]
        
        for profile, voltage in profiles:
            rospy.loginfo(f"\nRequesting Profile {profile} ({voltage})...")
            self.test_request_pd_role(profile)
            time.sleep(2)  # Wait for profile to settle
            self.test_get_status()
            time.sleep(1)
        
        # Test 3: Set specific voltage and current
        rospy.loginfo("\n--- Test 3: Set Specific Voltage and Current ---")
        test_configs = [
            (5000, 2000),   # 5V, 2A
            (9000, 2000),   # 9V, 2A
            (12000, 1500),  # 12V, 1.5A
            (15000, 1000),  # 15V, 1A
            (20000, 1000),  # 20V, 1A
        ]
        
        for voltage_mv, current_ma in test_configs:
            rospy.loginfo(f"\nSetting {voltage_mv}mV, {current_ma}mA...")
            self.test_set_voltage_current(voltage_mv, current_ma)
            time.sleep(2)  # Wait for settings to apply
            self.test_get_status()
            time.sleep(1)
        
        # Test 4: Monitor status for 10 seconds
        rospy.loginfo("\n--- Test 4: Monitor Status for 10 seconds ---")
        rospy.loginfo("Monitoring status messages...")
        time.sleep(10)
        
        rospy.loginfo("\n="*50)
        rospy.loginfo("ORB Controller Test Complete")
        rospy.loginfo("="*50)
    
    def run_monitor_mode(self):
        """Run in monitoring mode - just display status"""
        rospy.loginfo("Running in monitoring mode. Press Ctrl+C to exit.")
        rospy.spin()

def main():
    try:
        tester = ORBControllerTest()
        
        if len(sys.argv) > 1 and sys.argv[1] == "monitor":
            tester.run_monitor_mode()
        else:
            tester.run_full_test()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Test failed: {e}")

if __name__ == '__main__':
    main()