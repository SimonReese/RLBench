from typing import List
import numpy
from rlbench.backend.conditions import Condition, DetectedCondition
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.waypoints import Waypoint


class PickCupboard(Task):

    def init_task(self) -> None:
        # Gety reference to all objects
        self.objects = [
            Shape("cereals"),
            Shape("cola"),
            Shape("jar"),
            Shape("pringles"),
            Shape("milk"),
            Shape("grater")
        ]

        self.register_graspable_objects(self.objects)

        # Get reference to placing positions
        # self.place0 = Dummy("place0")
        # self.place1 = Dummy("place1")
        # self.place2 = Dummy("place2")
        # self.place3 = Dummy("place3")
        # self.place4 = Dummy("place4")
        # self.place5 = Dummy("place5")

        # Get refrence to waypoints
        self.way0 = Dummy("waypoint0")  # Approach <-| 
        self.way1 = Dummy("waypoint1")  # Grasp   -->| (root for 0 and 2)
        self.way2 = Dummy("waypoint2")  # Depart   <-|
        self.way3 = Dummy("waypoint3")  # Place
        
        # Get reference to possible place locations
        self.placeL = Dummy("placeL")
        self.placeR = Dummy("placeR")

        # Get reference to sensors
        self.sensorL = ProximitySensor("sensorL")
        self.sensorR = ProximitySensor("sensorR")

        # Register before move function
        self.register_waypoint_ability_start(0, self._start)

        # Init other vars
        self.target_obj = None

    def init_episode(self, index: int) -> List[str]:
        # Place objects randomly
        pos = numpy.random.randint(len(self.objects))
        for shape in self.objects:
            place = Dummy(f"place{pos}")
            shape.set_position(place.get_position())
            pos = (pos + 1) % len(self.objects) # Cycle every position
        
        # Choose random object
        n = numpy.random.randint(len(self.objects))
        self.target_obj = self.objects[n]

        # Set target place position
        if index == 0:
            # left
            self.way3.set_position(self.placeL.get_position())
            success_sensor = self.sensorL
            position = "left"
        else:
            # right
            self.way3.set_position(self.placeR.get_position())
            success_sensor = self.sensorR
            position = "right"

        success_conditon = DetectedCondition(self.target_obj, success_sensor)
        self.register_success_conditions([success_conditon])
        instruction = f"Pick up the -missing spatial position/{self.target_obj.get_name()}- object and place it on the {position} side of the table with respect to the robot"

        return [instruction]

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 2

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def is_static_workspace(self) -> bool:
        return True

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass
    
    def _start(self, waypoint: Waypoint):
        # Set waypoints
        self.target_obj: Shape
        self.way1.set_position(self.target_obj.get_position()) # This should move way0, 1, 2 accordingly
        pass