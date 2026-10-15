#!/usr/bin/env python3
import carla
import pygame
import numpy as np
import sys

def main():
    pygame.init()
    pygame.font.init()

    # Dashboard Dimensions: 1280x720 (4x Cameras Top, High-Res LiDAR Bottom)
    WINDOW_WIDTH, WINDOW_HEIGHT = 1280, 720
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("CARLA Autonomous Perception Suite - 360 Camera Array + High-Precision LiDAR")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Arial", 14, bold=True)
    title_font = pygame.font.SysFont("Arial", 16, bold=True)

    # Connect to CARLA
    client = carla.Client('172.21.32.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    vehicles = [a for a in world.get_actors() if 'vehicle' in a.type_id]
    if not vehicles:
        print("No vehicle found! Run carla_spawn_objects first.")
        sys.exit(1)

    ego = vehicles[0]
    bp_lib = world.get_blueprint_library()

    # Shared surface dictionary
    surfaces = {
        'front': None,
        'left': None,
        'right': None,
        'back': None,
        'lidar': None
    }

    # --- 1. Helper Function to Spawn RGB Cameras ---
    def create_camera(transform, label):
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '310')
        cam_bp.set_attribute('image_size_y', '220')
        cam_bp.set_attribute('fov', '100')
        cam_actor = world.spawn_actor(cam_bp, transform, attach_to=ego)

        def cb(image):
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = np.reshape(array, (image.height, image.width, 4))[:, :, :3][:, :, ::-1]
            surfaces[label] = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        cam_actor.listen(cb)
        return cam_actor

    # Spawn 360 Camera Array around vehicle chassis
    cams = [
        create_camera(carla.Transform(carla.Location(x=2.0, z=1.4), carla.Rotation(pitch=-5, yaw=0)), 'front'),
        create_camera(carla.Transform(carla.Location(x=0.0, y=-1.0, z=1.4), carla.Rotation(pitch=-5, yaw=-90)), 'left'),
        create_camera(carla.Transform(carla.Location(x=0.0, y=1.0, z=1.4), carla.Rotation(pitch=-5, yaw=90)), 'right'),
        create_camera(carla.Transform(carla.Location(x=-2.0, z=1.4), carla.Rotation(pitch=-5, yaw=180)), 'back')
    ]

    # --- 2. High-Precision 64-Channel LiDAR Sensor Setup ---
    lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '60')
    lidar_bp.set_attribute('channels', '64')              # High density 64-beam array
    lidar_bp.set_attribute('points_per_second', '600000') # 600k pts/sec automotive grade
    lidar_bp.set_attribute('rotation_frequency', '20')
    lidar_bp.set_attribute('upper_fov', '10')
    lidar_bp.set_attribute('lower_fov', '-30')

    lidar_tf = carla.Transform(carla.Location(x=0.0, z=2.0))
    lidar_sensor = world.spawn_actor(lidar_bp, lidar_tf, attach_to=ego)

    def process_highres_lidar(data):
        raw = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(raw, (int(len(raw) / 4), 4))

        # 860x380 resolution BEV projection
        GRID_W, GRID_H = 860, 380
        lidar_img = np.zeros((GRID_H, GRID_W, 3), dtype=np.uint8)

        x = points[:, 0]  # Forward
        y = points[:, 1]  # Lateral
        z = points[:, 2]  # Vertical
        intensity = points[:, 3]

        # Filter range: Front 40m, Rear 20m, Sides +/- 35m
        mask = (x >= -20.0) & (x <= 40.0) & (y >= -35.0) & (y <= 35.0)
        x_f, y_f, z_f, int_f = x[mask], y[mask], z[mask], intensity[mask]

        # Convert 3D world coordinates (meters) to 2D pixel coordinates
        px = np.int32((y_f + 35.0) * (GRID_W / 70.0))
        py = np.int32((40.0 - x_f) * (GRID_H / 60.0))

        # Filter points within display bounding box
        valid = (px >= 0) & (px < GRID_W) & (py >= 0) & (py < GRID_H)
        px_v, py_v, z_v, int_v = px[valid], py[valid], z_f[valid], int_f[valid]

        # Color palette: Road ground (Cyan/Blue), High obstacles/Walls (Green/Yellow)
        z_norm = np.clip((z_v + 1.8) / 3.5, 0.0, 1.0)
        r = np.int32(255 * (1 - z_norm))
        g = np.int32(255 * z_norm)
        b = np.int32(150 + 105 * (1 - z_norm))

        lidar_img[py_v, px_v] = np.column_stack((r, g, b))

        # Render Ego-Vehicle Footprint Center Box
        car_cx = int((0.0 + 35.0) * (GRID_W / 70.0))
        car_cy = int((40.0 - 0.0) * (GRID_H / 60.0))
        lidar_img[car_cy-10:car_cy+10, car_cx-5:car_cx+5] = [255, 30, 30]

        # Range Rings (10m, 20m, 30m)
        surfaces['lidar'] = pygame.surfarray.make_surface(lidar_img.swapaxes(0, 1))

    lidar_sensor.listen(process_highres_lidar)

    control = carla.VehicleControl()
    MAX_SPEED_KMH = 35.0
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:  # Reset vehicle position
                        spawn_points = world.get_map().get_spawn_points()
                        if spawn_points:
                            start_tf = spawn_points[0]
                            ego.set_transform(start_tf)
                            ego.set_target_velocity(carla.Vector3D(0, 0, 0))
                            ego.set_target_angular_velocity(carla.Vector3D(0, 0, 0))

            keys = pygame.key.get_pressed()
            vel_kmh = 3.6 * ego.get_velocity().length()

            # Smooth Steering
            target_steer = 0.0
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                target_steer = -0.5
            elif keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                target_steer = 0.5
            control.steer = control.steer * 0.7 + target_steer * 0.3

            # Proportional Speed Governor (35 km/h limit)
            speed_gap = MAX_SPEED_KMH - vel_kmh
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                control.reverse = False
                control.brake = 0.0
                if speed_gap > 5.0:
                    control.throttle = 1.0
                elif speed_gap > 0.0:
                    control.throttle = speed_gap / 5.0
                else:
                    control.throttle = 0.0
            elif keys[pygame.K_s] or keys[pygame.K_DOWN]:
                if vel_kmh > 0.8 and not control.reverse:
                    control.throttle = 0.0
                    control.brake = 0.8
                else:
                    control.brake = 0.0
                    control.reverse = True
                    control.throttle = 0.7 if speed_gap > 0 else 0.0
            else:
                control.throttle = 0.0
                control.brake = 0.1 if vel_kmh > 1.0 else 0.0

            control.hand_brake = bool(keys[pygame.K_SPACE])
            ego.apply_control(control)

            # --- Render Dashboard Interface ---
            screen.fill((15, 15, 20))

            # 1. Render Top 360 Camera Matrix
            cam_positions = [(10, 30), (328, 30), (646, 30), (960, 30)]
            cam_labels = ["FRONT CAM", "LEFT CAM", "RIGHT CAM", "REAR CAM"]
            cam_keys = ['front', 'left', 'right', 'back']

            for pos, label, key in zip(cam_positions, cam_labels, cam_keys):
                pygame.draw.rect(screen, (40, 40, 50), (pos[0]-2, pos[1]-2, 314, 224), 2)
                if surfaces[key] is not None:
                    screen.blit(surfaces[key], pos)
                screen.blit(title_font.render(label, True, (0, 220, 130)), (pos[0] + 5, pos[1] + 5))

            # 2. Render Bottom Left Telemetry Panel
            pygame.draw.rect(screen, (25, 25, 35), (10, 280, 380, 425))
            pygame.draw.rect(screen, (0, 220, 130), (10, 280, 380, 425), 2)
            screen.blit(title_font.render("VEHICLE TELEMETRY & CONTROLS", True, (255, 255, 255)), (25, 295))

            status_lines = [
                f"Speed Limit:  35.0 km/h",
                f"Cur Speed:    {vel_kmh:.1f} km/h",
                f"Throttle:     {control.throttle:.2f}",
                f"Steering:     {control.steer:.2f}",
                f"Brake:        {control.brake:.2f}",
                f"Reverse Mode: {control.reverse}",
                "",
                "Hotkeys:",
                "  W / A / S / D : Drive Vehicle",
                "  SPACE         : Handbrake",
                "  R             : Reset Car Position",
                "  ESC           : Exit Dashboard"
            ]

            y_off = 330
            for line in status_lines:
                color = (0, 220, 130) if "Cur Speed" in line else (200, 200, 200)
                screen.blit(font.render(line, True, color), (25, y_off))
                y_off += 28

            # 3. Render Bottom Right High-Precision LiDAR Map
            pygame.draw.rect(screen, (40, 40, 50), (408, 280, 864, 384), 2)
            if surfaces['lidar'] is not None:
                screen.blit(surfaces['lidar'], (410, 282))
            screen.blit(title_font.render("HIGH-PRECISION 64-BEAM LiDAR BEV MAP", True, (0, 180, 255)), (420, 290))

            pygame.display.flip()
            clock.tick(60)

    finally:
        for c in cams:
            c.destroy()
        lidar_sensor.destroy()
        pygame.quit()

if __name__ == '__main__':
    main()
