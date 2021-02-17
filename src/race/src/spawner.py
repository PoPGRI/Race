import carla 

sp = carla.command.SpawnActor
from carla import ActorBlueprint

client = carla.Client('localhost', 2000)
world = client.get_world()
audi = world.get_blueprint_library().filter("a2")[0]
transform = carla.Transform(carla.Location(-73.9, -191.5), carla.Rotation())
world.spawn_actor(audi, transform)
transform = carla.Transform(carla.Location(-30, -205), carla.Rotation())
world.spawn_actor(audi, transform)
