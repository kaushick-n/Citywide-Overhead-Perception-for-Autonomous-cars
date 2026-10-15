#!/usr/bin/env python3
import carla

def reset_ego_vehicle(host='172.21.32.1', port=2000):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()
    vehicles = [a for a in world.get_actors() if 'vehicle' in a.type_id]
    if not vehicles:
        print('No vehicle found to reset.')
        return
    ego = vehicles[0]
    spawn_points = world.get_map().get_spawn_points()
    if spawn_points:
        start_tf = spawn_points[0]
        ego.set_transform(start_tf)
        ego.set_target_velocity(carla.Vector3D(0, 0, 0))
        ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))
        spectator = world.get_spectator()
        forward = start_tf.get_forward_vector()
        cam_loc = start_tf.location - carla.Location(x=6*forward.x, y=6*forward.y, z=-2.5)
        spectator.set_transform(carla.Transform(cam_loc, carla.Rotation(pitch=-15, yaw=start_tf.rotation.yaw)))
        print('SUCCESS: Ego vehicle reset to start position.')

if __name__ == '__main__':
    reset_ego_vehicle()
