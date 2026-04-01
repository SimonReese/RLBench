from typing import List
from numpy import place
from rlbench.backend.task import Task

from pyrep.objects import Dummy, ProximitySensor, Shape


class CubeContainer(Task):

    def init_task(self) -> None:
        self.cube = Shape("cube")
        self.way3 = Dummy("waypoint3")
        self.register_graspable_objects([self.cube])
        pass

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
            - 0: left container
            - 1: right container
        """

        if index == 0:
            position = "left"
            reference = Dummy("placeL")
        else:
            position = "right"
            reference = Dummy("placeR")

        # Set waypoint3 to appropriate position
        self.way3.set_position(reference.get_position())
        # Generate appropriate task instuction
        instruction = f"Pick up the cube and place it on the {position} container with respect to the robot"
        return [instruction]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass
