import os
import time
from typing import List, Tuple
import numpy
from pyrep.objects.object import Object
from rlbench.backend.task import Task

from pyrep.objects import Dummy, ProximitySensor, Shape
from pyrep.backend.sim import simLoadModel, simGetObjectName, simGetObjectHandle
from rlbench.backend.waypoints import Waypoint

class CubeContainer(Task):

    OBJECT_NAMES = [
        "toy car",
        "joystick controller",
        "mug",
        "pc mouse",
        "tv remote",
        #"cube"
    ]

    OBJECT_PATHS = [
        "RLBench/rlbench/assets/cartoy/cartoy.ttm",
        "RLBench/rlbench/assets/controller/controller.ttm",
        "RLBench/rlbench/assets/mug/mug.ttm",
        "RLBench/rlbench/assets/pcmouse/pcmouse.ttm",
        "RLBench/rlbench/assets/tvremote/tvremote.ttm",
        #"CUBE_NO_PATH"
    ]

    def init_task(self) -> None:
        # Object placement point
        self.obj_place = Dummy("object_place")

        # Cube object
        self.cube = Shape("cube")

        # Waypoints
        self.way0 = Dummy("waypoint0") # approach waypoint
        self.way1 = Dummy("waypoint1") # grasping waypoint
        self.way2 = Dummy("waypoint2") # depart waypoint
        self.way3 = Dummy("waypoint3") # release waypoint

        # Call function after generic approach
        self.register_waypoint_ability_start(0, self._start)
        
        self.object = None
        self.approach_dummy = None
        self.grasp_dummy = None
        pass

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
            - 0: left container
            - 1: right container
        """
        # Load object
        object_index = numpy.random.randint(len(self.OBJECT_NAMES))
        obj_name = self.OBJECT_NAMES[object_index]
        obj_path = self.OBJECT_PATHS[object_index]
        print(f"Trying to load {obj_name}")
        
        model_handle = simLoadModel(obj_path) # spawn object
        self.object = Shape(model_handle)
        self.object.set_parent(self.obj_place)


        # Set object position
        self.object.set_pose(self.obj_place.get_pose())

        # Register object
        self.register_graspable_objects([self.object])

        # Get key position
        self.approach_dummy = Dummy("approach_pose")
        self.grasp_dummy = Dummy("grasp_pose")

        # Set position of waypoints
        self.way0.set_pose(self.approach_dummy.get_pose())
        self.way1.set_pose(self.grasp_dummy.get_pose())
        self.way2.set_pose(self.approach_dummy.get_pose())
        self.way3.set_pose(self.approach_dummy.get_pose())  # To rotate the waypoint3 pose similar to graping pose
    

        if index == 0:
            position = "left"
            reference = Dummy("placeL")
        else:
            position = "right"
            reference = Dummy("placeR")

        # Set waypoint3 to appropriate position (keeping grasping orientation)
        self.way3.set_position(reference.get_position())
        # Generate appropriate task instuction
        instruction = f"Pick up the {obj_name} and place it on the container on its {position} side"
        print(instruction)
        return [instruction]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Restore objects
        if self.object and self.object.get_name() != "cube" and self.object is not None and self.object.exists(self.object.get_name()):
            self.object.remove()
        #self.cube.set_pose(self.obj_place.get_pose())
        pass
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        return (0.0, 0.0, -numpy.pi/6), (0.0, 0.0, numpy.pi/6)
    
    def _start(self, waypoint: Waypoint):
        assert self.approach_dummy is not None
        assert self.grasp_dummy is not None
        self.way0.set_pose(self.approach_dummy.get_pose())
        # Set all waypoints to exact positions after falls of objects
        print(f"Setting grasping pose: {self.way1.get_pose()[:3]}", end="")
        #waypoint.get_waypoint_object().set_pose(self.grasp_dummy.get_pose())
        self.way1.set_pose(self.grasp_dummy.get_pose())
        print(f" -> {self.way1.get_pose()[:3]} - [{self.way1} vs {waypoint.get_waypoint_object()}]")
        self.way2.set_pose(self.approach_dummy.get_pose())
        pass