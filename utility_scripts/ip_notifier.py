import requests
import socket
import fcntl
import struct
import time
import sys
import logging

WEBHOOK_URL = 'https://discord.com/api/webhooks/1275822499107704843/qHf_0CfQ1CKGMtSkpD473bYbdgxLYcDZZGVz5x1w6nGBLCcaA3vG1BJumV-e8PVQsnHO'
current_ip = None  # Global variable to store the current IP address

# Configure logging to write to sys.stdout, which will be captured by systemd
logging.basicConfig(stream=sys.stdout, level=logging.INFO, format='%(asctime)s - %(message)s')

def get_internal_ip(interface='eth0'):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ip_address = fcntl.ioctl(
            sock.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', interface[:15].encode('utf-8'))
        )[20:24]
        return socket.inet_ntoa(ip_address)
    except Exception as e:
        logging.error(f"Error getting internal IP: {e}")
        return None

def send_discord_notification(ip):
    data = {
        "content": f"The current internal IP address of your Raspberry Pi is: {ip}"
    }
    response = requests.post(WEBHOOK_URL, json=data)
    if response.status_code == 204:
        logging.info(f'Notification sent successfully with IP: {ip}')
    else:
        logging.error(f"Failed to send notification. Status code: {response.status_code}")

def main():
    global current_ip

    while True:
        new_ip = get_internal_ip(interface='wlan0')
        if new_ip is None:
            logging.warning("Could not get IP address. Retrying...")
            time.sleep(10)
            continue

        if new_ip != current_ip:
            send_discord_notification(new_ip)
            current_ip = new_ip

        time.sleep(60)  # Check every 60 seconds (adjust as needed)

if __name__ == "__main__":
    main()
