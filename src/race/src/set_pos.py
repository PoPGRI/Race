import carla 
import sys
import os
import argparse
from carla import Location, Transform, Rotation


def set_position(y, x, yaw, role_name):
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    actors = world.get_actors()
    for actor in actors:
        if 'vehicle' in actor.type_id and actor.attributes.get('role_name') == role_name:
            transform = Transform(Location(x,-y), Rotation(yaw=yaw))
            actor.set_transform(transform)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'Set the x, y position of the vehicle')
    # 97.47158813476562, -100.61478424072266
    x_default = 193
    y_default = 45
    yaw_default = 135
    role_name_default = 'ego_vehicle'

    parser.add_argument('--x', type = float, help='x position of the vehicle.', default=x_default)
    parser.add_argument('--y', type = float, help='y position of the vehicle.', default=y_default)
    parser.add_argument('--name', type = str, help='Rolename of the vehicle', default=role_name_default)

    argv = parser.parse_args()

    x = argv.x
    y = argv.y
    yaw = yaw_default
    role_name = argv.name

    set_position(y, x, yaw, role_name)



