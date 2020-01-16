#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import time
from open_opc_gateway_client import OpenOpcGatewayClient
import sys


class KoneOpcController():

    def __init__(self, read_timeout, connection_timeout, simu=False):
        # The timeout in seconds to wait for a value to change
        self.read_timeout = read_timeout
        self.connection_timeout = connection_timeout

        self.gateway_map = {}
        self.simu = simu

    # Return whether we succesfully connected to the gateway server
    def connect(self, node, gateway_ip, gateway_port, opc_serv_ip, opc_serv_type):
        # If there is no client to that gateway server, we create one
        if gateway_ip not in self.gateway_map:
            self.gateway_map[gateway_ip] = OpenOpcGatewayClient(self.connection_timeout, self.simu)

        return self.gateway_map[gateway_ip].connect(node, gateway_ip, gateway_port, opc_serv_ip, opc_serv_type)

    def close(self, node, gateway_ip):
        # If there is already a gateway to that server
        if gateway_ip in self.gateway_map:
            self.gateway_map[gateway_ip].close(node)
            del self.gateway_map[gateway_ip]

    # Function to read a lift of tags and wait for their values to change to the requested value
    def read_and_wait_for_value(self, node, gateway_ip, tag_list, req_value_list, timeout, opc_group):
        # node.get_logger().info("Waiting for value {} to be {} [{}]".format(tag_list, req_value_list, gateway_ip))

        if len(tag_list) == 0 or len(tag_list) != len(req_value_list):
            node.get_logger().error("Trying to read from incorrect lists {} - {} [{}]".format(len(tag_list), len(req_value_list), gateway_ip))
            return False

        ok = False
        start = time.time()
        while not ok and (time.time() - start) < timeout:
            # Read the value of the tags
            cmd_status, result = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)
            if cmd_status:
                ok = True
                # check that each value are the requested one
                for idx, value in enumerate(result):
                    # if the value is different, we try again
                    if str(value) != str(req_value_list[idx]):
                        node.get_logger().warn("Value {} - {} - {} is not the requested one {} after {} sec [{}]".format(idx, tag_list[idx], value, req_value_list[idx], time.time() - start, gateway_ip))
                        ok = False
                if ok:
                    status = True
                    node.get_logger().info("Got the expected value after {} sec [{}]".format(time.time() - start, gateway_ip))
                else:
                    status = False
                    time.sleep(1)
            else:
                node.get_logger().error("Couldn't get the expected value after {} sec [{}]".format(time.time() - start, gateway_ip))
                status = False
                time.sleep(1)
        return status

    # Check that the lift is in AGV mode and set it to the AGV mode if needed
    def check_and_set_lift_agv_mode(self, node, gateway_ip, opc_group, site, location, group, lift):
        tag_list = ["{}.{}.{}.{}.LiftMode".format(site, location, group, lift)]

        # Read the value of the tag
        status, (current_lift_mode,) = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)

        if status:
            # AGV mode
            if current_lift_mode == 26:
                node.get_logger().info("{} Lift is in AGV mode".format(lift))

            # Normal mode, we change it to AGV mode
            elif current_lift_mode == 0:
                node.get_logger().info("{} Lift in Normal mode, changing to AGV mode".format(lift))

                # Set the mode to AGV
                status = self.set_mode(node, gateway_ip, site, location, group, lift, 2)

                if not status:
                    node.get_logger().error("{} Could not set the lift to AGV mode".format(lift))
            else:
                node.get_logger().error("{} The lift is neither in Normal or AGV mode, the lift is currently in mode {}".format(lift, current_lift_mode))
                status = False
        else:
            node.get_logger().error("{} Could not check the LiftMode".format(lift))
            status = False

        return status, current_lift_mode

    # Handle the sequence to send the lift to the given floor
    def send_lift_to_floor(self, node, gateway_ip, opc_group, site, location, group, lift, floor, door_side):
        node.get_logger().info("{} Trying to send the lift to floor {}".format(lift, floor))

        cmd_base = "{}.{}.{}.{}".format(site, location, group, lift)
        car_call_base = "{}.Commands.CarCall".format(cmd_base)

        # Check that the LiftMode is 26
        tag_list = ["{}.LiftMode".format(cmd_base)]
        status, (current_lift_mode,) = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)

        if status:
            if current_lift_mode == 26:
                node.get_logger().info("{} Lift is in AGV mode".format(lift))
            elif current_lift_mode == 0:
                node.get_logger().error("{} Lift in Normal mode, please call the right service to set to AGV mode before sending the lift to the floor of your choice".format(lift))
                return False
            else:
                node.get_logger().error("{} The lift is neither in Normal or AGV mode, can only change the lift to AGV mode when in Normal mode".format(lift))
                return False
        else:
            node.get_logger().error("{} Could not check the LiftMode".format(lift))
            return False

        # Initiate car call sequence
        tag_list = [("{}.Floor".format(car_call_base), floor), ("{}.Door".format(car_call_base), door_side), ("{}.Send".format(car_call_base), 1)]
        status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Send lift to floor : Could not initiate car call sequence {}".format(lift, status))
            return False

        if lift == 'Lift2':
            node.get_logger().info("{} \nSleeping\n\n".format(lift))
            time.sleep(5)

        # Wait a bit for Ack to turn to 1
        tag_list = ["{}.Ack".format(car_call_base), "{}.LiftMode".format(cmd_base)]
        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, [1, 26], self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Send lift to floor : Could not read the CarCall.Ack tag {} or timed out waiting for the right value".format(lift, status))
            # If the ack failed, we need to set Send back to 0
            # TODO test this
            tag_list = [("{}.Send".format(car_call_base), 0)]
            status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)
            return False

        # Set Send to 0
        tag_list = [("{}.Send".format(car_call_base), 0)]
        status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Send lift to floor : Could not write on the CarCall.Send tag {}".format(lift, status))
            return False

        # Lift should be coming now
        # Wait a bit for Ack to turn to 0
        tag_list = ["{}.Ack".format(car_call_base), "{}.LiftMode".format(cmd_base)]
        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, [0, 26], self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Send lift to floor : Could not read on the CarCall.Ack tag {} or timed out waiting for the right value".format(lift, status))
            return False

        # Checking that the lift arrived, and AGV should be able to move in
        # Check that the LiftMode is 26
        tag_list = ["{}.LiftMode".format(cmd_base)]
        status, (current_lift_mode,) = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)

        if status:
            if current_lift_mode == 26:
                node.get_logger().info("{} Lift is in AGV mode".format(lift))
            elif current_lift_mode == 0:
                node.get_logger().error("{} Called a lift car but the lift mode changed is now in Normal mode".format(lift))
                return False
            else:
                node.get_logger().error("{} Called a lift car but the lift mode changed is now neither in Normal or AGV mode".format(lift))
                return -False
        else:
            node.get_logger().error("{} Could not check the LiftMode".format(lift))
            return False

        # Check the lift's position
        tag_list = ["{}.ActualPosition".format(cmd_base)]

        # Check that the door is open
        if door_side == 1:
            tag_list.append("{}.DoorState.Side1".format(cmd_base))
        elif door_side == 2:
            tag_list.append("{}.DoorState.Side2".format(cmd_base))
        else:
            node.get_logger().error("{} Door side value is incorrect {}, should not be here".format(lift, door_side))
            return False

        # Check the MovingState => Lift shouldn't be moving...
        tag_list.append("{}.MovingState".format(cmd_base))

        # Check the MovingState => Lift shouldn't be moving...
        tag_list.append("{}.LiftMode".format(cmd_base))

        # The requested values for the requested floor, the door state, the moving state and the lift mode
        req_value_list = [floor, 0, 6, 26]

        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, req_value_list, self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Could not read the position of the lift, the door state and the moving state tags {} or timed out waiting for the correct values".format(lift, status))
            return False

        # Normal return, everything went as expected
        return status

    # Set the mode of the lift to the given mode var
    # Mode value :
    # 0: Normal mode
    # 1: AGV mode
    def set_mode(self, node, gateway_ip, opc_group, site, location, group, lift, mode):
        lift_mode = 0

        cmd_base = "{}.{}.{}.{}".format(site, location, group, lift)
        agv_service_base = "{}.Commands.AgvService".format(cmd_base)

        if mode == 1:
            node.get_logger().info("{} Trying to set the Lift to the Normal mode".format(lift))
        elif mode == 2:
            node.get_logger().info("{} Trying to set the Lift to the AGV mode".format(lift))
            lift_mode = 26
        else:
            node.get_logger().error("{} Trying to set the Lift to an incorrect mode {}".format(lift, mode))
            return False

        # Check that the lift is not in another mode than Normal or AGV
        tag_list = ["{}.LiftMode".format(cmd_base)]
        status, (current_lift_mode,) = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Could not check the LiftMode".format(lift))
            return False

        if current_lift_mode != 0 and current_lift_mode != 26:
            node.get_logger().error("{} The lift is neither in Normal or AGV mode, can only change the lift to AGV mode when in Normal mode".format(lift))
            return False

        if lift_mode == current_lift_mode:
            node.get_logger().warn("{} The lift is already in the requested mode".format(lift))
            return True

        # Initiating the set mode sequence
        tag_list = [("{}.State".format(agv_service_base), mode - 1), ("{}.Send".format(agv_service_base), 1)]
        status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Set mode : Could not initiate the set mode sequence {}".format(lift, status))
            return False

        # Wait a bit for Ack to turn to 1
        tag_list = ["{}.Ack".format(agv_service_base)]
        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, [1], self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Set mode : Could not read on the AgvService.Ack tag {} or timed out waiting for the right value".format(lift, status))
            # If the ack failed, we need to set Send back to 0
            # TODO test this
            tag_list = [("{}.Send".format(agv_service_base), 0)]
            status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)
            return False

        # Set Send to 0
        tag_list = [("{}.Send".format(agv_service_base), 0)]
        status = self.gateway_map[gateway_ip].write(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Set mode : Could not write on the AgvService.Send tag {}".format(lift, status))
            return False

        # Wait a bit for Ack to turn to 0
        tag_list = ["{}.Ack".format(agv_service_base)]
        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, [0], self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Set mode : Could not read the AgvService.Ack tag {} or timed out waiting for the right value".format(lift, status))
            return False

        # Wait a bit for LiftMode to turn to 26
        tag_list = ["{}.LiftMode".format(cmd_base)]
        status = self.read_and_wait_for_value(node, gateway_ip, tag_list, [lift_mode], self.read_timeout, opc_group)

        if not status:
            node.get_logger().error("{} Set mode : Could not read the LiftMode tag {} or timed out waiting for the right value".format(lift, status))
            return False

        return True

    # Get lift info from the given lift
    def get_lift_info(self, node, gateway_ip, opc_group, site, location, group, lift):
        cmd_base = "{}.{}.{}.{}".format(site, location, group, lift)

        # Check that the lift is not in another mode than Normal or AGV
        tag_list = ["{}.LiftMode".format(cmd_base),
                    "{}.ActualPosition".format(cmd_base),
                    "{}.NextLanding".format(cmd_base),
                    "{}.MovingState".format(cmd_base),
                    "{}.MovingDirection".format(cmd_base),
                    "{}.DoorState.Side1".format(cmd_base),
                    "{}.DoorState.Side2".format(cmd_base)]
        status, info = self.gateway_map[gateway_ip].read(node, tag_list, opc_group)

        if not status:
            node.get_logger().error("{} Could not check the lift info".format(lift))
            return False, []

        return status, info
