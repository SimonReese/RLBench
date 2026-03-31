from asyncio import Condition
from ctypes import Union
from typing import List
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task

from pyrep.objects import ProximitySensor
from pyrep.objects import Dummy
from pyrep.objects import Shape

class SlideBlock(Task):

    def init_task(self) -> None:
        # Get reference to sensor
        self.sensor = ProximitySensor("sensor")
        # Get reference to each block
        self.blockC = Shape("blockC")
        self.blockL = Shape("blockL")
        self.blockR = Shape("blockR")
        # Get reference to movable dummy
        self.way0 = Dummy("waypoint0")
        self.way1 = Dummy("waypoint1")
        # Ensure waypoint has extension string
        ext_str = self.way0.get_extension_string()
        assert("ignore_collision" in ext_str)
        #self.condition: DetectedCondition = None

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
            - 0: slide central block
            - 1: slide left block
            - 2: slide right block
        """
        


        # Move waypoint0 to correct place
        if index == 0:
            reference = Dummy("placeC")
            position = "central"
            self.condition = DetectedCondition(self.blockC, self.sensor)
        elif index == 1: 
            reference = Dummy("placeL")
            position = "left"
            self.condition = DetectedCondition(self.blockL, self.sensor)
        else:
            reference = Dummy("placeR")
            position = "right"
            self.condition = DetectedCondition(self.blockR, self.sensor)

        self.way0.set_position(reference.get_position())
        # Orientate waypoints correctly
        self.way0.set_orientation(reference.get_orientation())
        self.way1.set_orientation(reference.get_orientation())

        instruction = f"Slide the {position} block with respect to the robot to the target area"
        #self.register_success_conditions([self.condition])
        return [instruction]

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 3

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        C = self.sensor.is_detected(self.blockC)
        L = self.sensor.is_detected(self.blockL)
        R = self.sensor.is_detected(self.blockR)
        print(f"Detection (C, L, R): ({C}, {L}, {R})")
        #print(f"Condition: {self.condition.condition_met()}")
        pass
