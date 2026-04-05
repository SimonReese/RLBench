import os
import time
from typing import List
import numpy
from rlbench.backend.task import Task

from pyrep.objects import Dummy, ProximitySensor, Shape
from pyrep.backend.sim import simLoadModel, simGetObjectName, simGetObjectHandle

class CubeContainer(Task):

    OBJECT_NAMES = [
        "toy car",
        "joystick controller",
        "mug",
        "pc mouse",
        "tv remote",
        "cube"
    ]

    OBJECT_PATHS = [
        "RLBench/rlbench/assets/cartoy/cartoy.ttm",
        "RLBench/rlbench/assets/controller/controller.ttm",
        "RLBench/rlbench/assets/mug/mug.ttm",
        "RLBench/rlbench/assets/pcmouse/pcmouse.ttm",
        "RLBench/rlbench/assets/tvremote/tvremote.ttm",
        "CUBE_NO_PATH"
    ]

    def init_task(self) -> None:
        # Object placement point
        self.obj_place = Dummy("object_place")

        self.cube = Shape("cube")

        # Waypoints
        self.way0 = Dummy("waypoint0") # approach waypoint
        self.way1 = Dummy("waypoint1") # grasping waypoint
        self.way2 = Dummy("waypoint2") # depart waypoint
        self.way3 = Dummy("waypoint3") # release waypoint

        self.register_graspable_objects([self.cube])
        
        self.object = None
        pass

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
            - 0: left container
            - 1: right container
        """

        # Random object
        object_index: int = numpy.random.randint(len(self.OBJECT_NAMES))
        obj_name = self.OBJECT_NAMES[object_index]
        obj_path = self.OBJECT_PATHS[object_index]
        print(f"Trying to load {obj_name}")

        # Load object and key positions
        if obj_name != "cube":
            self.cube.set_position([0, 0, 0]) # hide cube under the table
            model_handle = simLoadModel(obj_path) # spawn object
            self.object = Shape(model_handle)
            self.object.set_parent(self.obj_place)
            assert type(self.object) is Shape, f"Type of obj was {type(self.object)}: {self.object}"
            approach_pose = Dummy("approach_pose").get_pose(relative_to=self.object)
            grasp_pose = Dummy("grasp_pose").get_pose(relative_to=self.object)
        else: 
            self.object = self.cube
            approach_pose = [0, 0, 0.1, 0, 0, 0, 1]
            grasp_pose = [0, 0, 0, 0, 0, 0, 1]
        
        
        # Set object position
        self.object.set_pose(self.obj_place.get_pose())
        # Set approaching and grasping waypoint positions
        self.way0.set_pose(approach_pose, relative_to=self.object)
        self.way1.set_pose(grasp_pose, relative_to=self.object)
        self.way2.set_pose(approach_pose, relative_to=self.object)

        if index == 0:
            position = "left"
            reference = Dummy("placeL")
        else:
            position = "right"
            reference = Dummy("placeR")

        # Set waypoint3 to appropriate position
        self.way3.set_position(reference.get_position())
        # Generate appropriate task instuction
        instruction = f"Pick up the {obj_name} and place it on the container on its {position} side"
        return [instruction]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Restore objects
        if self.object and self.object.get_name() is not "cube" is not None and self.object.exists(self.object.get_name()):
            self.object.remove()
        self.cube.set_pose(self.obj_place.get_pose())
        pass
