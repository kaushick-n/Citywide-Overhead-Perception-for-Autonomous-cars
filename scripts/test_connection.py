#!/usr/bin/env python3
import carla, subprocess

def get_host_ip():
    try:
        res = subprocess.check_output("ip route show | grep default | awk '{print $3}'", shell=True)
        return res.decode('utf-8').strip()
    except Exception:
        return '172.21.32.1'

host_ip = get_host_ip()
print(f'Connecting to CARLA server on {host_ip}:2000...')
try:
    client = carla.Client(host_ip, 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    print('SUCCESS: Connected to CARLA Server!')
    print(f'Client Version: {client.get_client_version()}')
    print(f'Server Version: {client.get_server_version()}')
    print(f'Current Map: {world.get_map().name}')
except Exception as e:
    print(f'FAILED to connect: {e}')
