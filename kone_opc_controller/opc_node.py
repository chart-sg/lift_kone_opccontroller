#!/usr/bin/env python

# TODO Could probably reduce this to only import LiftState.msg and LiftRequest.msg
from rmf_msgs.msg import *

import rclpy
from rclpy.node import Node
from rclpy.qos import _qos_profile_default
from rclpy.time import Time
from std_msgs.msg import Int32
from .kone_opc_controller import KoneOpcController
import Pyro4.util
from threading import Thread, Lock
import sys


class OpcNode(Node):

    def __init__(self):
        super().__init__("kone_open_opc_controller")

        self.lift_info_array = []

        # Default timeouts in seconds
        self.connection_timeout = 5
        self.read_timeout = 60

        # Default topics
        self.lift_request_topic = 'lift_request'
        self.lift_state_topic = 'lift_state'

        # Just for testing
        self.simu = False

        # Some flags
        self.info_lock = Lock()

    def init(self):
        if len(self.lift_info_array) > 0:
            # The Class that takes care of the different opc gateway and sequences to control the lift
            self.opc_controller = KoneOpcController(self.read_timeout, self.connection_timeout, self.simu)

            self.lift_request_sub = self.create_subscription(LiftRequest, self.lift_request_topic, self.lift_request_callback, 10)

            # Create a publisher to publish the lifts status
            self.lift_state_publisher = self.create_publisher(LiftState, self.lift_state_topic, _qos_profile_default)

            # Create an array containing the LiftState messages to publish
            self.lift_state_msg_array = []

            # Create a timer with the rate (2Hz) at which we update the lifts status
            self.lift_state_update_timer = self.create_timer(2, self.lift_state_update_timer_callback)

            # Create a timer with the rate (2Hz) at which we want to publish the lifts status
            self.lift_state_pub_timer = self.create_timer(2, self.lift_state_pub_timer_callback)

        else:
            return False
        return True

    # The timer callback that publishes the lift status every 2 sec
    def lift_state_pub_timer_callback(self):
        # self.get_logger().info("Publishing {} messages".format(len(self.lift_state_msg_array)))
        for lift_state_msg in self.lift_state_msg_array:
            self.lift_state_publisher.publish(lift_state_msg)

    # Service callback to send the lift to the given floor
    def lift_request_callback(self, msg):
        self.get_logger().info("Lift request callback called {}".format(msg))

        # Check if the lift is busy on another call
        lift_is_busy = True
        for lift_info in self.lift_info_array:
            if lift_info['id'] == msg.lift_name:
                lift_is_busy = lift_info['busy']

        if lift_is_busy:
            self.get_logger().error("Lift {} is busy".format(msg.lift_name))
        else:
            lift_index = -1
            # Check the session_id to know if the lift is already taken
            # or if it's used by someone else
            for index in range(len(self.lift_state_msg_array)):
                if self.lift_state_msg_array[index].lift_name == msg.lift_name:
                    lift_index = index

                    # If we already have a session id and it's different from the current one, we can't use the lift
                    if self.lift_state_msg_array[index].session_id and self.lift_state_msg_array[index].session_id != msg.session_id:
                        lift_index = -1

            if lift_index == -1:
                self.get_logger().error("Lift {} already have a different session id".format(msg.lift_name))
            else:
                got_lift_info = False
                # Get the lift info
                for lift_info in self.lift_info_array:
                    if lift_info['id'] == msg.lift_name:
                        got_lift_info = True

                        if msg.request_type == 1:
                            # Set the session id
                            self.get_logger().info("The lift is not used by anyone else so we lock it for this request")
                            self.lift_state_msg_array[lift_index].session_id = msg.session_id
                            # Set the lift as busy
                            lift_info['busy'] = True
                            thread = Thread(target=self.send_lift_to_floor, args=(msg, lift_info,))
                            thread.start()

                        elif msg.request_type == 0:
                            # Set the session id
                            self.get_logger().info("The lift is not used by anyone else so we lock it for this request")
                            self.lift_state_msg_array[lift_index].session_id = msg.session_id
                            # Set the lift as busy
                            lift_info['busy'] = True
                            thread = Thread(target=self.release_lift, args=(msg, lift_info,))
                            thread.start()

                        else:
                            self.get_logger().error("Could not find the lift info")

                if not got_lift_info:
                    self.get_logger().error("Could not find the lift info")
                    self.lift_state_msg_array[lift_index].session_id = ''

    def send_lift_to_floor(self, msg, lift_info):
        self.get_logger().info("\n\nBlip send_lift_to_floor {}\n".format(msg))

        try:
            # We try to connect to the OPC gateway
            if self.opc_controller.connect(self, lift_info['gateway_ip'], lift_info['gateway_port'], lift_info['opc_serv_ip'], lift_info['opc_serv_type']):

                # We check and set if needed the lift to AGV mode
                if self.opc_controller.set_mode(self, lift_info['gateway_ip'], lift_info['opc_group'], lift_info['site'], lift_info['location'], lift_info['group'], lift_info['lift'], 2):

                    door_side = 0
                    # We get the door side to open for this floor
                    for index in range(len(lift_info['available_floors'])):
                        if lift_info['available_floors'][index] == msg.destination_floor:
                            door_side = lift_info['available_door_sides'][index]

                    if door_side == 0:
                        self.get_logger().error("Could not find the door_side or the destination in the available lift parameters")
                    else:
                        # We finally send the lift to the requested floor
                        if not self.opc_controller.send_lift_to_floor(self, lift_info['gateway_ip'], lift_info['opc_group'], lift_info['site'], lift_info['location'], lift_info['group'], lift_info['lift'], msg.destination_floor, door_side):
                            self.get_logger().error("Could not send the lift to the requested floor")
                else:
                    self.get_logger().error("Could not change the mode of the lift")
            else:
                    self.get_logger().error("Could not connect to the gateway")
        except Exception as error_msg:
            self.get_logger().error("send_lift_to_floor error : [{}]\n\n[{}]".format(error_msg, "".join(Pyro4.util.getPyroTraceback())))
        finally:
            # Set the lift as not busy anymore
            for lift_info in self.lift_info_array:
                if lift_info['id'] == msg.lift_name:
                    lift_info['busy'] = False
                    self.get_logger().warn("Releasing {} so it's not busy anymore".format(lift_info['id']))
            self.get_logger().info("\n\nBlop send_lift_to_floor {}\n".format(msg.lift_name))

    def release_lift(self, msg, lift_info):
        self.get_logger().info("\n\nBlip release_lift {}\n".format(msg))

        try:
            # First, we try to connect to the OPC gateway
            if self.opc_controller.connect(self, lift_info['gateway_ip'], lift_info['gateway_port'], lift_info['opc_serv_ip'], lift_info['opc_serv_type']):

                # We set the lift to the default lift_mode
                if not self.opc_controller.set_mode(self, lift_info['gateway_ip'], lift_info['opc_group'], lift_info['site'], lift_info['location'], lift_info['group'], lift_info['lift'], lift_info['default_mode']):
                    self.get_logger().error("Could not set the lift to its default mode")
            else:
                self.status_map[gateway_ip][sub_key].status = status
        except Exception as error_msg:
            self.get_logger().error("set_lift_mode error : [{}]\n\n[{}]".format(error_msg, "".join(Pyro4.util.getPyroTraceback())))
        finally:
            # Set the lift as not busy anymore
            for lift_info in self.lift_info_array:
                if lift_info['id'] == msg.lift_name:
                    lift_info['busy'] = False
                    self.get_logger().warn("Releasing {} so it's not busy anymore".format(lift_info['id']))
            # Remove the session_id
            for lift_state_msg in self.lift_state_msg_array:
                if lift_state_msg.lift_name == msg.lift_name:
                    lift_state_msg.session_id = ''
            self.get_logger().info("\n\nBlop release_lift {}\n".format(msg.lift_name))

    # Update the LiftState msg with new status
    def lift_state_update_timer_callback(self):
        if self.info_lock.acquire(False):

            try:
                # Get each lift info
                for lift_info in self.lift_info_array:
                    try:
                        lift_state_msg = LiftState()
                        # If we are connected to the OPC gateway we can continue
                        if self.opc_controller.connect(self, lift_info['gateway_ip'], lift_info['gateway_port'], lift_info['opc_serv_ip'], lift_info['opc_serv_type']):
                            # We get the lift info
                            status, info = self.opc_controller.get_lift_info(self, lift_info['gateway_ip'], lift_info['opc_group'], lift_info['site'], lift_info['location'], lift_info['group'], lift_info['lift'])
                            # If we managed to read the data
                            if status:
                                # Create the message to publish
                                lift_state_msg.lift_time = self._clock.now().to_msg()
                                lift_state_msg.lift_name = lift_info['id']
                                lift_state_msg.available_floors = lift_info['available_floors']
                                lift_state_msg.available_modes = lift_info['available_modes']

                                # Convert the current mode from the OPC value to the LiftState msg value
                                # See the LiftState.msg from the rmf_msgs repo for more info
                                if info[0] == 0:
                                    lift_state_msg.current_mode = 1
                                elif info[0] == 26:
                                    lift_state_msg.current_mode = 2
                                elif info[0] == 2:
                                    lift_state_msg.current_mode = 3
                                elif info[0] == 14:
                                    lift_state_msg.current_mode = 4
                                else:
                                    lift_state_msg.current_mode = 0

                                lift_state_msg.current_floor = "{}".format(info[1])
                                lift_state_msg.destination_floor = "{}".format(info[2])

                                # Convert the moving direction from the OPC value to the LiftState msg value
                                if info[4] == 0:
                                    lift_state_msg.motion_state = 2
                                elif info[4] == 1:
                                    lift_state_msg.motion_state = 1
                                elif info[4] == 2:
                                    lift_state_msg.motion_state = 0

                                # Convert the door state from the OPC value to the LiftState msg value
                                if info[5] == 0 or info[6] == 0:
                                    lift_state_msg.door_state = 2
                                elif info[5] == 2 or info[6] == 2:
                                    lift_state_msg.door_state = 0
                                else:
                                    lift_state_msg.door_state = 1

                                # Get the previous session_id and replace the old message with a new one
                                found_old_msg = False
                                lift_state_msg.session_id = ''
                                for index in range(len(self.lift_state_msg_array)):
                                    if self.lift_state_msg_array[index].lift_name == lift_info['id']:
                                        lift_state_msg.session_id = self.lift_state_msg_array[index].session_id
                                        self.lift_state_msg_array[index] = lift_state_msg
                                        found_old_msg = True

                                # If we can't find the old message so we add a new one
                                if not found_old_msg:
                                    self.lift_state_msg_array.append(lift_state_msg)
                            else:
                                # TODO publish some error/diagnostic msg ?
                                self.get_logger().error("Could not check the lift info of {}".format(lift_info['id']))
                                self.send_lift_error(lift_info['id'])
                        else:
                            # TODO publish some error/diagnostic msg ?
                            self.get_logger().error("could not connect to {}".format(lift_info['gateway_ip']))
                            self.send_lift_error(lift_info['id'])
                    except Exception as error_msg:
                        self.get_logger().error("lift_state_update_timer_callback error {} : [{}]\n\n[{}]".format(lift_info['id'], error_msg, "".join(Pyro4.util.getPyroTraceback())))
                        self.send_lift_error(lift_info['id'])
                # End of the for loop, just making sure we release the lock no matter what
            except Exception as error_msg:
                self.get_logger().error("Unknown error : {}".format(error_msg))
            finally:
                # self.get_logger().info("Releasing lock")
                self.info_lock.release()
        else:
            self.get_logger().warn("Already trying to get some info, the refresh rate to get the lift info might be too high")

    # Change the published lift state to an error state
    def send_lift_error(self, lift_name):
        self.get_logger().warn("Creating the error message")
        lift_state_msg = LiftState()
        lift_state_msg.lift_time = self._clock.now().to_msg()
        lift_state_msg.lift_name = lift_name

        # Set the current mode to MODE_UNKNOWN
        lift_state_msg.current_mode = 0

        # Get the previous session_id and replace the old message with a new one
        found_old_msg = False
        lift_state_msg.session_id = ''
        for index in range(len(self.lift_state_msg_array)):
            if self.lift_state_msg_array[index].lift_name == lift_name:
                lift_state_msg.session_id = self.lift_state_msg_array[index].session_id
                self.lift_state_msg_array[index] = lift_state_msg
                found_old_msg = True

        # If we can't find the old message so we add a new one
        if not found_old_msg:
            self.lift_state_msg_array.append(lift_state_msg)

    # Parameters setters
    def set_connection_timeout(self, timeout):
        self.connection_timeout = timeout

    def set_read_timeout(self, timeout):
        self.read_timeout = timeout

    def set_lift_info_array(self, lift_info_array):
        self.lift_info_array = lift_info_array

    def set_simu(self, simu):
        self.simu = simu
        self.get_logger().info("Using simulation sequence \'{}\'".format(self.simu))

    def set_lift_request_topic(self, lift_request_topic):
        self.lift_request_topic = lift_request_topic
        self.get_logger().info("Set lift_request_topic to \'{}\'".format(self.lift_request_topic))

    def set_lift_state_topic(self, lift_state_topic):
        self.lift_state_topic = lift_state_topic
        self.get_logger().info("Set lift_state_topic to \'{}\'".format(self.lift_state_topic))
