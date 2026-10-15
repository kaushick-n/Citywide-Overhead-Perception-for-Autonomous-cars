#!/usr/bin/env python3
import carla, time

def follow_vehicle(host='172.21.32.1', port=2000):
    client = carla.Client(host, port)
    world = client.get_world()

    vehicles = [a for a in world.get_actors() if 'vehicle' in a.type_id]
    if not vehicles:
        print("No ego vehicle found.")
        return

    ego = vehicles[0]
    spectator = world.get_spectator()
    print("Tracking vehicle live... Press Ctrl+C to exit.")

    try:
        while True:
            t = ego.get_transform()
            fwd = t.get_forward_vector()
            cam_loc = t.location - carla.Location(x=6*fwd.x, y=6*fwd.y, z=-2.5)
            spectator.set_transform(carla.Transform(cam_loc, carla.Rotation(pitch=-15, yaw=t.rotation.yaw)))
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nStopped tracking.")

if __name__ == '__main__':
    follow_vehicle()
