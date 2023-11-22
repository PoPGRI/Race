import carla
import time

class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle

    def run_step(self, img, vel):
        """
        Execute one step of navigation.

        Args:
        img
            - H * W * C (height, width, channel)
            - 720 * 1280 * 3 
            - 3 channels are R, G, B correspondingly
        vel

        Return: carla.VehicleControl()
        """
        
        # Print img 
        print(img.shape)


        print("Reach Customized Agent")
        control = carla.VehicleControl()
        control.throttle = 0.5
        return control