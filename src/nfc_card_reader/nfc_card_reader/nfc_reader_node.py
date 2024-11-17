#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smartcard.CardMonitoring import CardMonitor, CardObserver
from smartcard.util import toHexString
from std_msgs.msg import String
import json

class NFCReaderNode(Node, CardObserver):
    def __init__(self):
        Node.__init__(self, 'nfc_reader_node')
        CardObserver.__init__(self)
        self.get_logger().info('NFC Reader Node has been started.')

        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'nfc_card_data', 10)

        # Set up the card monitor
        self.card_monitor = CardMonitor()
        self.card_monitor.addObserver(self)

        # Variables to track the current card
        self.current_card = None
        self.card_connection = None

    def update(self, observable, actions):
        added_cards, removed_cards = actions

        for card in added_cards:
            self.handle_card_inserted(card)

        for card in removed_cards:
            self.handle_card_removed(card)

    def handle_card_inserted(self, card):
        card_info = toHexString(card.atr)
        self.current_card = card

        try:
            self.card_connection = self.current_card.createConnection()
            self.card_connection.connect()

            self.get_logger().info(f'Card inserted with ATR: {card_info}')

            # Read data from the card
            card_data = self.read_card_data()
            if card_data:
                # Publish the card data
                msg = String()
                msg.data = json.dumps(card_data)
                self.publisher_.publish(msg)
                self.get_logger().info('Card data published.')
            else:
                self.get_logger().warn('No data read from card.')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to card: {e}')

    def handle_card_removed(self, card):
        card_info = toHexString(card.atr)
        self.get_logger().info(f'Card removed with ATR: {card_info}')

        self.current_card = None
        if self.card_connection:
            try:
                self.card_connection.disconnect()
                self.card_connection = None
            except Exception as e:
                self.get_logger().error(f'Failed to disconnect card: {e}')

    def read_card_data(self):
        """
        Implement the logic to read data from the NFC card.
        """
        try:
            # Example: Send a command APDU to read data
            # This should be replaced with actual commands for your NFC card
            GET_DATA = [0xFF, 0xCA, 0x00, 0x00, 0x00]
            response, sw1, sw2 = self.card_connection.transmit(GET_DATA)
            if sw1 == 0x90 and sw2 == 0x00:
                uid = toHexString(response)
                self.get_logger().info(f'Card UID: {uid}')
                return {'uid': uid}
            else:
                self.get_logger().warn(f'Unexpected response: SW1={sw1}, SW2={sw2}')
                return None
        except Exception as e:
            self.get_logger().error(f'Error reading card data: {e}')
            return None

    def destroy_node(self):
        # Clean up resources
        self.card_monitor.deleteObserver(self)
        if self.card_connection:
            try:
                self.card_connection.disconnect()
            except Exception as e:
                self.get_logger().error(f'Error disconnecting card: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = NFCReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('NFC Reader Node stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
