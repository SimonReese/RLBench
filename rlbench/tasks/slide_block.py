from typing import List

import numpy
from pyrep.objects import Dummy, ProximitySensor, Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task


class SlideBlock(Task):

    def init_task(self) -> None:
        # Get reference to plane
        self.plane = Shape("Plane")
        # Get reference to sensor
        self.sensor = ProximitySensor("sensor")
        # Get reference to each block
        self.blockC = Shape("blockC")
        self.blockL = Shape("blockL")
        self.blockR = Shape("blockR")
        # Get reference to movable dummy
        self.way0 = Dummy("waypoint0")
        self.way1 = Dummy("waypoint1")

        self.condition: DetectedCondition

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
            - 0: slide central block
            - 1: slide left block
            - 2: slide right block
        """
        self.variation_index = index

        # Select proper refrence dummy, offset and condition
        if index == 0:
            reference = Dummy("placeC")
            position = "central"
            self.condition = DetectedCondition(self.blockC, self.sensor)
            offset = numpy.array([-0.05, 0, 0])
        elif index == 1: 
            reference = Dummy("placeL")
            position = "left"
            self.condition = DetectedCondition(self.blockL, self.sensor)
            offset = numpy.array([0, +0.05, 0])
        else:
            reference = Dummy("placeR")
            position = "right"
            self.condition = DetectedCondition(self.blockR, self.sensor)
            offset = numpy.array([0, -0.05, 0])

        # Move waypoint0 to reference dummy and waypoint1 of appropriate offset over plane
        self.way0.set_position(reference.get_position())
        self.way1.set_position(
            self.way1.get_position(relative_to=self.plane) + offset, 
            relative_to=self.plane
        )
        # Orientate waypoints correctly to keep eef aligned
        self.way0.set_orientation(reference.get_orientation())
        self.way1.set_orientation(reference.get_orientation())

        # Generate appropriate task instuction
        instruction = f"Slide the {position} block with respect to the robot to the target area"
        self.register_success_conditions([self.condition])
        #print(instruction)
        return [instruction]

    def variation_count(self) -> int:
        return 3

    # def cleanup(self) -> None:
    #     # Called during at the end of each episode. Remove this if not using.
    #     C = self.sensor.is_detected(self.blockC)
    #     L = self.sensor.is_detected(self.blockL)
    #     R = self.sensor.is_detected(self.blockR)
    #     print(f"Detection (C, L, R): ({C}, {L}, {R})")
    #     if hasattr(self, 'condition'):
    #         print(f"Condition: {self.condition.condition_met()}")
    #     pass
