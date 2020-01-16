#!/usr/bin/env python

import rclpy
from rclpy.parameter import Parameter
from .opc_node import OpcNode


# Get a specific lift info
def get_lift_param(node, count):
    lift_info = {}

    try:
        node.declare_parameter("lift_info_{}_id".format(count))
        lift_id = node.get_parameter_or("lift_info_{}_id".format(count))
        if lift_id.type_ == Parameter.Type.STRING:
            node.get_logger().info("lift_info_{}: {}".format(count, lift_id.value))
            lift_info['id'] = lift_id.value
        else:
            node.get_logger().warn("lift_id {}: no more lift or wrong parameter type {}".format(count, lift_id.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_gateway_ip".format(count))
        lift_gateway_ip = node.get_parameter("lift_info_{}_gateway_ip".format(count))
        if lift_gateway_ip.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_gateway_ip {}: {}".format(count, lift_gateway_ip.value))
            lift_info['gateway_ip'] = lift_gateway_ip.value
        else:
            node.get_logger().warn("lift_gateway_ip {}: no more lift or wrong parameter type {}".format(count, lift_gateway_ip.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_gateway_port".format(count))
        lift_gateway_port = node.get_parameter("lift_info_{}_gateway_port".format(count))
        if lift_gateway_port.type_ == Parameter.Type.INTEGER:
            # node.get_logger().info("lift_gateway_port {}: {}".format(count, lift_gateway_port.value))
            lift_info['gateway_port'] = lift_gateway_port.value
        else:
            node.get_logger().warn("lift_gateway_port {}: no more lift or wrong parameter type {}".format(count, lift_gateway_port.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_opc_serv_ip".format(count))
        lift_opc_serv_ip = node.get_parameter("lift_info_{}_opc_serv_ip".format(count))
        if lift_opc_serv_ip.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_opc_serv_ip {}: {}".format(count, lift_opc_serv_ip.value))
            lift_info['opc_serv_ip'] = lift_opc_serv_ip.value
        else:
            node.get_logger().warn("lift_opc_serv_ip {}: no more lift or wrong parameter type {}".format(count, lift_opc_serv_ip.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_opc_serv_type".format(count))
        lift_opc_serv_type = node.get_parameter("lift_info_{}_opc_serv_type".format(count))
        if lift_opc_serv_type.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_opc_serv_type {}: {}".format(count, lift_opc_serv_type.value))
            lift_info['opc_serv_type'] = lift_opc_serv_type.value
        else:
            node.get_logger().warn("lift_opc_serv_type {}: no more lift or wrong parameter type {}".format(count, lift_opc_serv_type.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_opc_group".format(count))
        lift_opc_group = node.get_parameter("lift_info_{}_opc_group".format(count))
        if lift_opc_group.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_opc_group {}: {}".format(count, lift_opc_group.value))
            lift_info['opc_group'] = lift_opc_group.value
        else:
            node.get_logger().warn("lift_opc_group {}: no more lift or wrong parameter type {}".format(count, lift_opc_group.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_site".format(count))
        lift_site = node.get_parameter("lift_info_{}_site".format(count))
        if lift_site.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_site {}: {}".format(count, lift_site.value))
            lift_info['site'] = lift_site.value
        else:
            node.get_logger().warn("lift_site {}: no more lift or wrong parameter type {}".format(count, lift_site.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_location".format(count))
        lift_location = node.get_parameter("lift_info_{}_location".format(count))
        if lift_location.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_location {}: {}".format(count, lift_location.value))
            lift_info['location'] = lift_location.value
        else:
            node.get_logger().warn("lift_location {}: no more lift or wrong parameter type {}".format(count, lift_location.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_group".format(count))
        lift_group = node.get_parameter("lift_info_{}_group".format(count))
        if lift_group.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_group {}: {}".format(count, lift_group.value))
            lift_info['group'] = lift_group.value
        else:
            node.get_logger().warn("lift_group {}: no more lift or wrong parameter type {}".format(count, lift_group.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_lift".format(count))
        lift_lift = node.get_parameter("lift_info_{}_lift".format(count))
        if lift_lift.type_ == Parameter.Type.STRING:
            # node.get_logger().info("lift_lift {}: {}".format(count, lift_lift.value))
            lift_info['lift'] = lift_lift.value
        else:
            node.get_logger().warn("lift_lift {}: no more lift or wrong parameter type {}".format(count, lift_lift.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_available_floors".format(count))
        lift_available_floors = node.get_parameter("lift_info_{}_available_floors".format(count))
        if lift_available_floors.type_ == Parameter.Type.STRING_ARRAY:
            # node.get_logger().info("lift_available_floors {}: {}".format(count, lift_available_floors.value))
            # This list needs to be escaped with a ':' in front of each element for the YAML parser to parse it properly, we can now remove it
            # TODO Hopefully a temporary solution, see https://answers.ros.org/question/325288/ros-2-crystal-cant-get-parameter-list-of-string-from-yaml-file/
            lift_info['available_floors'] = [i[1:] for i in lift_available_floors.value]
        else:
            node.get_logger().warn("lift_available_floors {}: no more lift or wrong parameter type {}".format(count, lift_available_floors.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_available_door_sides".format(count))
        lift_available_door_sides = node.get_parameter("lift_info_{}_available_door_sides".format(count))
        if lift_available_door_sides.type_ == Parameter.Type.INTEGER_ARRAY:
            # node.get_logger().info("lift_available_door_sides {}: {}".format(count, lift_available_door_sides.value))
            lift_info['available_door_sides'] = lift_available_door_sides.value
        else:
            node.get_logger().warn("lift_available_door_sides {}: no more lift or wrong parameter type {}".format(count, lift_available_door_sides.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_available_modes".format(count))
        lift_available_modes = node.get_parameter("lift_info_{}_available_modes".format(count))
        if lift_available_modes.type_ == Parameter.Type.INTEGER_ARRAY:
            # node.get_logger().info("lift_available_modes {}: {}".format(count, lift_available_modes.value))
            lift_info['available_modes'] = lift_available_modes.value
        else:
            node.get_logger().warn("lift_available_modes {}: no more lift or wrong parameter type {}".format(count, lift_available_modes.type_))
            return False, {}

        node.declare_parameter("lift_info_{}_default_mode".format(count))
        lift_default_mode = node.get_parameter("lift_info_{}_default_mode".format(count))
        if lift_default_mode.type_ == Parameter.Type.INTEGER:
            # node.get_logger().info("lift_default_mode {}: {}".format(count, lift_default_mode.value))
            lift_info['default_mode'] = lift_default_mode.value
        else:
            node.get_logger().warn("lift_default_mode {}: no more lift or wrong parameter type {}".format(count, lift_default_mode.type_))
            return False, {}

        return True, lift_info
    except rclpy.exceptions.ParameterNotDeclaredException as error_msg:
        node.get_logger().warn("lift_info_{}: No more lift {}".format(count, error_msg))
        return False, {}


# Get and set all the lifts info
def set_lift_array_param(node):
    node.get_logger().info("Initializing lift list")
    has_lift = True
    count = 1
    lift_info_array = []
    while (has_lift):
        has_lift, lift_info = get_lift_param(node, count)
        if has_lift:
            lift_info['busy'] = False
            lift_info_array.append(lift_info)
            count += 1
        else:
            count -= 1

    node.get_logger().info("Number of lifts: {}".format(len(lift_info_array)))
    # node.get_logger().info("Lifts: {}".format(lift_info_array))
    node.set_lift_info_array(lift_info_array)


# Get and set the timeout parameters
def set_timeout_params(node):
    node.declare_parameter("connection_timeout", 5)
    connection_timeout = node.get_parameter("connection_timeout")
    # Check if the parameter is set and of the right type
    if connection_timeout.type_ == Parameter.Type.INTEGER:
        node.set_connection_timeout(connection_timeout.value)
    elif connection_timeout.type_ != Parameter.Type.NOT_SET:
        node.get_logger().warn("connection_timeout is of the wrong type {} instead of type INTEGER".format(connection_timeout.type_))

    node.declare_parameter("read_timeout", 60)
    read_timeout = node.get_parameter("read_timeout")
    # Check if the parameter is set and of the right type
    if read_timeout.type_ == Parameter.Type.INTEGER:
        node.set_read_timeout(read_timeout.value)
    elif read_timeout.type_ != Parameter.Type.NOT_SET:
        node.get_logger().warn("read_timeout is of the wrong type {} instead of type INTEGER".format(read_timeout.type_))


# Get and set the topics/services parameters
def set_topics(node):
    node.declare_parameter("lift_request_topic", "lift_request")
    lift_request = node.get_parameter("lift_request_topic")
    # Check if the parameter is set and of the right type
    if lift_request.type_ == Parameter.Type.STRING:
        node.set_lift_request_topic(lift_request.value)
    elif lift_request.type_ != Parameter.Type.NOT_SET:
        node.get_logger().warn("lift_request is of the wrong type {} instead of type STRING".format(lift_request.type_))

    node.declare_parameter("lift_state_topic", "lift_state")
    lift_state = node.get_parameter("lift_state_topic")
    # Check if the parameter is set and of the right type
    if lift_state.type_ == Parameter.Type.STRING:
        node.set_lift_state_topic(lift_state.value)
    elif lift_state.type_ != Parameter.Type.NOT_SET:
        node.get_logger().warn("lift_state is of the wrong type {} instead of Type STRING".format(lift_state.type_))


# Get and set the topics/services parameters
def set_simu_param(node):
    node.declare_parameter("simu", False)
    simu = node.get_parameter("simu")
    # Check if the parameter is set and of the right type
    if simu.type_ == Parameter.Type.BOOL:
        node.set_simu(simu.value)
    elif simu.type_ != Parameter.Type.NOT_SET:
        node.get_logger().warn("simu is of the wrong type {} instead of type STRING".format(simu.type_))


# Get and set all the parameters from the YAML file
def set_params_from_yaml(node):
    # Get and set the topics/services parameters
    set_topics(node)

    # Get and set the timeout parameters
    set_timeout_params(node)

    # Get and set all the lifts info
    set_lift_array_param(node)

    # Set the simu param to select whether or not we are using the Matrikon simu so we need to simulates the KONE sequences to change the lift mode and do car calls
    set_simu_param(node)


def main(args=None):
    rclpy.init(args=args)

    node = OpcNode()

    set_params_from_yaml(node)

    if node.init():
        rclpy.spin(node)
    else:
        node.get_logger().info("Could not properly initialize the node, no lift info provided")

    # Destroy the node explicitly
    # (optional - Done automatically when node is garbage collected)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
