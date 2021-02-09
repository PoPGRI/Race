import carla 

sp = carla.command.SpawnActor
from carla import ActorBlueprint

client = carla.Client('localhost', 2000)
world = client.get_world()
# blueprints = world.get_blueprint_library().filter("vehicle.*")
audi = world.get_blueprint_library().filter("a2")[0]
transform = carla.Transform(carla.Location(-30, -205), carla.Rotation())
# sp(audi, transform)
world.spawn_actor(audi, transform)
# print(audi)
