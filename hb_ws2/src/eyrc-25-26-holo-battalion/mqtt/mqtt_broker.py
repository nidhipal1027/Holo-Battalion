#!/usr/bin/env python3

"""
* Team Id :eYRC#1832
* Author List : Nidhi pal, Seeya kokam, Kashisa padhy
* Filename: ros2_mqtt_bridge.py
* Theme: Multi-Bot ROS2 MQTT Communication
* Functions: __init__, on_connect, mqtt_message_callback,
             attach_callback, detach_callback,
             bot_cmd_callback, publish_to_esp, main
* Global Variables: BOT_CONFIG
"""

import rclpy
from rclpy.node import Node
from hb_interfaces.msg import BotCmdArray
from std_msgs.msg import String
from linkattacher_msgs.srv import AttachLink, DetachLink
import paho.mqtt.client as mqtt
import json
from functools import partial


# BOT_CONFIG:
#dictionary mapping robot names to their unique bot IDs.
BOT_CONFIG = {
    'hb_crystal':   0,
    'hb_frostbite': 2,
    'hb_glacio':    4
}


class Ros2MqttBridge(Node):
    """
    ROS2 Node that bridges communication between ROS2 topics/services
    and MQTT topics for multiple bots.
    """

    """
    * Function Name: __init__
    * Input: self
    * Output: None
    * Logic: Initializes ROS2 node, creates publishers, subscribers,
             services and sets up MQTT client connection.
    * Example Call: bridge = Ros2MqttBridge()
    """
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # solenoid_states:
#stores ON/OFF state for each bot.
        self.solenoid_states = {cfg_id: 0.0 for cfg_id in BOT_CONFIG.values()}

        # ir_publishers:
#dictionary storing ROS2 publishers for IR sensor topics.
        self.ir_publishers = {}

        # last_cmds:
#stores last movement command for each bot to allow refresh.
        self.last_cmds = {cfg_id: None for cfg_id in BOT_CONFIG.values()}

#create subscriber to receive bot movement commands
        self.bot_cmd_sub = self.create_subscription(
            BotCmdArray, '/bot_cmd', self.bot_cmd_callback, 10)

#create publishers and services for each bot
        for name, bot_id in BOT_CONFIG.items():

#create IR sensor publisher
            topic_name = f'/ir_sensor_{bot_id}'
            self.ir_publishers[bot_id] = self.create_publisher(
                String, topic_name, 10)

#create attach service
            self.create_service(
                AttachLink,
                f'/{name}/attach_crate',
                partial(self.attach_callback, bot_id=bot_id)
            )

#create detach service
            self.create_service(
                DetachLink,
                f'/{name}/detach_crate',
                partial(self.detach_callback, bot_id=bot_id)
            )

            self.get_logger().info(
                f"Initialized interfaces for {name} (ID: {bot_id})")

# MQTT Client Setup
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.mqtt_message_callback

        try:
            self.mqtt_client.connect("localhost", 1883, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")

        self.get_logger().info(">>> Multi-Bot MQTT Bridge Running <<<")

    """
    * Function Name: on_connect
    * Input: client, userdata, flags, rc, properties
    * Output: None
    * Logic: Handles MQTT connection and subscribes to IR topics.
    """
    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info("MQTT Connected.")

#subscribe using wildcard '+' for all bot IR topics
            self.mqtt_client.subscribe("hb/sensor/+/ir", qos=1)
        else:
            self.get_logger().error(f"MQTT Connection failed: {rc}")

    """
    * Function Name: mqtt_message_callback
    * Input: client, userdata, msg
    * Output: None
    * Logic: Receives IR data from MQTT and republishes to ROS2 topic.
    """
    def mqtt_message_callback(self, client, userdata, msg):
        try:
            topic_parts = msg.topic.split('/')

            # Expected format: hb/sensor/{bot_id}/ir
            if len(topic_parts) == 4 and topic_parts[3] == 'ir':
                bot_id = int(topic_parts[2])

                if bot_id in self.ir_publishers:
                    payload = msg.payload.decode()

                    ir_msg = String()
                    ir_msg.data = payload

                    self.ir_publishers[bot_id].publish(ir_msg)

        except Exception as e:
            self.get_logger().error(f"MQTT Rx Error: {e}")

    """
    * Function Name: attach_callback
    * Input: request, response, bot_id
    * Output: response
    * Logic: Activates solenoid (grip ON) and refreshes last command.
    """
    def attach_callback(self, request, response, bot_id):
        self.solenoid_states[bot_id] = 1.0
        self.get_logger().info(f"Bot {bot_id}: GRIP ON")

#resend last command with updated solenoid state
        self.publish_to_esp(bot_id, self.last_cmds.get(bot_id))
        return response

    """
    * Function Name: detach_callback
    * Input: request, response, bot_id
    * Output: response
    * Logic: Deactivates solenoid (grip OFF) and refreshes last command.
    """
    def detach_callback(self, request, response, bot_id):
        self.solenoid_states[bot_id] = 0.0
        self.get_logger().info(f"Bot {bot_id}: GRIP OFF")

#resend last command with updated solenoid state
        self.publish_to_esp(bot_id, self.last_cmds.get(bot_id))
        return response

    """
    * Function Name: bot_cmd_callback
    * Input: msg (BotCmdArray)
    * Output: None
    * Logic: Stores latest command and publishes to corresponding ESP.
    """
    def bot_cmd_callback(self, msg):
        for cmd in msg.cmds:
            self.last_cmds[cmd.id] = cmd
            self.publish_to_esp(cmd.id, cmd)

    """
    * Function Name: publish_to_esp
    * Input: bot_id (int), cmd (BotCmd or None)
    * Output: None
    * Logic: Converts command into JSON format and publishes
             to MQTT topic hb/cmd/{bot_id}.
    """
    def publish_to_esp(self, bot_id, cmd=None):

        if bot_id not in self.solenoid_states:
            return

        # Construct JSON payload for ESP
        cmd_json = {
            "m1": float(cmd.m1) if cmd else 0.0,
            "m2": float(cmd.m2) if cmd else 0.0,
            "m3": float(cmd.m3) if cmd else 0.0,
            "base": float(cmd.base) if cmd else 120.0,
            "elbow": float(cmd.elbow) if cmd else 130.0,
            "solenoid": self.solenoid_states[bot_id]
        }

        topic = f"hb/cmd/{bot_id}"
        self.mqtt_client.publish(topic, json.dumps(cmd_json), qos=0)


"""
* Function Name: main
* Input: None
* Output: None
* Logic: Initializes ROS2, runs node, and safely shuts down.
"""
def main():
    rclpy.init()
    bridge = Ros2MqttBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.mqtt_client.loop_stop()
        bridge.mqtt_client.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()