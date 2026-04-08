import os
import time
from typing import List, Tuple
import numpy
from pyrep.objects.object import Object
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
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
        "tv remote"
    ]

    OBJECT_PATHS = [
        "RLBench/rlbench/assets/cartoy/cartoy.ttm",
        "RLBench/rlbench/assets/controller/controller.ttm",
        "RLBench/rlbench/assets/mug/mug.ttm",
        "RLBench/rlbench/assets/pcmouse/pcmouse.ttm",
        "RLBench/rlbench/assets/tvremote/tvremote.ttm"
    ]

    def init_task(self) -> None:
        # Spawn boundary
        self.spawn_boundary = SpawnBoundary([Shape("spawn_boundary")])

        # Object placement point
        self.obj_place = Dummy("object_place")

        # Waypoints
        self.way0 = Dummy("waypoint0") # approach waypoint
        self.way1 = Dummy("waypoint1") # grasping waypoint
        self.way2 = Dummy("waypoint2") # depart waypoint
        self.way3 = Dummy("waypoint3") # release waypoint

        # Proximity sensors
        self.sensorL = ProximitySensor("sensorL")
        self.sensorR = ProximitySensor("sensorR")

        # Call function after generic approach
        self.register_waypoint_ability_start(0, self._start)
        
        self.object = None
        self.approach_dummy = None
        self.grasp_dummy = None
        self.success_sensor = None

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:

            WRT object (all possible roations):
            - 0: left container
            - 1: right container

            WRT robot (up to ± 45° rotations):
            - 2: left container
            - 3: right container
        """
        # We use this when we need to swap the target (i.e: if object is spawned with more than 90° rot, 
        # the L/R boxes will be inverted wrt robot reference frame- )
        inverted = False
        # TODO: Use inverted also to change the grasping behaviour -> flip the grasping, approaching, placing 
        # dummy along downords z by 180 degrees so the picking and placing will be natural
        # TODO: reduce angles a little bit, maybe 45 degrees
        # TODO: add 2 variations: nearest and furtest wrt robot, choose the default R box as furthest but check inversion
        if index in (0, 1):
            # Sample scene rotations
            min_rot = (0, 0, -numpy.pi)
            max_rot = (0, 0, +numpy.pi)
        else:
        #elif index in (2, 3):
            # We need 2 spaces: ±45 and (+135, +225)
            if numpy.random.random() > 0.5:     # ± 45
                min_rot = (0, 0, -numpy.pi/4)
                max_rot = (0, 0, numpy.pi/4)
            else:                               # 135->225
                inverted = True # The boxes will be opposite wrt the robot pov
                min_rot = (0, 0, numpy.pi * (3.0/4.0))
                max_rot = (0, 0, numpy.pi * (5.0/4.0))

        self.spawn_boundary.clear()
        self.spawn_boundary.sample(self.obj_place, min_rotation=min_rot, max_rotation=max_rot)

        # Load random object
        object_index = numpy.random.randint(len(self.OBJECT_NAMES))
        obj_name = self.OBJECT_NAMES[object_index]
        obj_path = self.OBJECT_PATHS[object_index]
        print(f"Trying to load {obj_name}")
        model_handle = simLoadModel(obj_path) # spawn object
        self.object = Shape(model_handle)
        self.object.set_parent(self.obj_place)

        # Set object position
        self.object.set_pose(self.obj_place.get_pose())

        # Register object as graspable
        self.register_graspable_objects([self.object])

        # Get key position
        self.approach_dummy = Dummy("approach_pose")
        self.grasp_dummy = Dummy("grasp_pose")

        # Set position of waypoints
        self.way0.set_pose(self.approach_dummy.get_pose())
        self.way1.set_pose(self.grasp_dummy.get_pose())
        self.way2.set_pose(self.approach_dummy.get_pose())
        self.way3.set_pose(self.approach_dummy.get_pose())  # To rotate the waypoint3 pose similar to graping pose

        # Choose appropriate variation
        # set textual position, refrence dummy, sensor and generate appropriate task instuction
        if index == 0:
            position = "left"
            reference = Dummy("placeL")
            self.success_sensor = self.sensorL
            instruction = f"Pick up the {obj_name} and place it on the container on its {position} side"
        elif index == 1:
            position = "right"
            reference = Dummy("placeR")
            self.success_sensor = self.sensorR
            instruction = f"Pick up the {obj_name} and place it on the container on its {position} side"
        elif index == 2:
            # Left wrt robot, but check if inverted spawn position
            position = "left"
            if inverted:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            else:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            instruction = f"Pick up the {obj_name} and place it on the container on the {position} side with respect to the robot"
        else:    # index == 3
            # Right wrt robot, but check if inverted spawn position
            position = "right"
            if inverted:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            else:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            instruction = f"Pick up the {obj_name} and place it on the container on the {position} side with respect to the robot"

        # Set waypoint3 to appropriate position (keeping grasping orientation)
        self.way3.set_position(reference.get_position())
        # Register success condition
        success_condition = DetectedCondition(self.object, self.success_sensor)
        self.register_success_conditions([success_condition])
        
        print(instruction)
        return [instruction]

    def variation_count(self) -> int:
        return 4

    def cleanup(self) -> None:
        # Remove added object
        if self.object is not None and self.object.still_exists():
            self.object.remove()
    
    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        #return (0.0, 0.0, -numpy.pi/6), (0.0, 0.0, numpy.pi/6)
        return (0, 0, 0), (0, 0, 0)
    
    def _start(self, waypoint: Waypoint):
        assert self.approach_dummy is not None
        assert self.grasp_dummy is not None
        # Set all waypoints to exact positions after falls of objects
        #waypoint.get_waypoint_object().set_pose(self.grasp_dummy.get_pose()) - not clear if this is has the same result
        self.way0.set_pose(self.approach_dummy.get_pose())
        self.way1.set_pose(self.grasp_dummy.get_pose())
        self.way2.set_pose(self.approach_dummy.get_pose())
        