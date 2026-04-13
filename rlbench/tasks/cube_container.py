from typing import List, Tuple

import numpy
from pyrep.backend.sim import simLoadModel
from pyrep.objects import Dummy, ProximitySensor, Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from rlbench.backend.waypoints import Waypoint


class CubeContainer(Task):
    """ Place an object in one of the two container on its side

        Variations (WRT robot):
            - (up to ± 45° rotations):
                - 0: left container
                - 1: right container
            - (from 45° to 135° rotations):
                - 2: nearest container
                - 3: furthest container
    """

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
             Variations (WRT robot):
            - (up to ± 45° rotations):
                - 0: left container
                - 1: right container
            - (from 45° to 135° rotations):
                - 2: nearest container (defualt is L container if not inverted)
                - 3: furthest container (default is R container if not inverted)
        """
        # We use this when we need to swap the target (i.e: if object is spawned with more than 90° rot, 
        # the L/R boxes will be inverted wrt robot reference frame- )
        inverted = False
        if index in (0, 1):
            # We need 2 spaces: ±45 and (+135, +225)
            if numpy.random.random() > 0.5:     # ± 45
                min_rot = (0, 0, -numpy.pi/4)
                max_rot = (0, 0, numpy.pi/4)
            else:                               # 135->225
                inverted = True # The boxes will be opposite wrt the robot pov
                min_rot = (0, 0, numpy.pi * (3.0/4.0))
                max_rot = (0, 0, numpy.pi * (5.0/4.0))

        else: #index in (2, 3):
            # We rotate in ranges (+45, +135) and (+225, +315)
            if numpy.random.random() > 0.5:     # (+45, +135)
                min_rot = (0, 0, numpy.pi/4)
                max_rot = (0, 0, numpy.pi * 3.0/4.0)
            else:                               # (+225, +315)
                inverted = True # The boxes will be opposite wrt the robot pov
                min_rot = (0, 0, numpy.pi * (5.0/4.0))
                max_rot = (0, 0, numpy.pi * (7.0/4.0))


        self.spawn_boundary.clear()
        self.spawn_boundary.sample(self.obj_place, min_rotation=min_rot, max_rotation=max_rot)

        # Load random object
        object_index = numpy.random.randint(len(self.OBJECT_NAMES))
        obj_name = self.OBJECT_NAMES[object_index]
        obj_path = self.OBJECT_PATHS[object_index]
        #print(f"Trying to load {obj_name}")
        model_handle = simLoadModel(obj_path) # spawn object
        self.object = Shape(model_handle)
        self.object.set_parent(self.obj_place)

        # Set object position
        self.object.set_pose(self.obj_place.get_pose())

        # Register object as graspable
        self.register_graspable_objects([self.object])

        # Get key position
        model_name = self.object.get_name()
        grasp_name = "top"
        self.approach_dummy = Dummy(f"{model_name}_{grasp_name}_approach_pose")
        self.grasp_dummy = Dummy(f"{model_name}_{grasp_name}_grasp_pose")

        # Rotate the waypoint3 pose similar to graping pose
        self.way3.set_pose(self.approach_dummy.get_pose())  # To rotate the waypoint3 pose similar to graping pose

        # Choose appropriate variation
        # set textual position, refrence dummy, sensor and generate appropriate task instuction        
        if index == 0:
            # Left wrt robot, but check if inverted spawn position
            position = "left"
            if inverted:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            else:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            instruction = f"Pick up the {obj_name} and place it on the container on the {position} side with respect to the robot"
        elif index == 1:
            # Right wrt robot, but check if inverted spawn position
            position = "right"
            if inverted:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            else:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            instruction = f"Pick up the {obj_name} and place it on the container on the {position} side with respect to the robot"
        
        elif index == 2:
            # Nearest container, left if not inverted
            position = "nearest"
            if inverted:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            else:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            instruction = f"Pick up the {obj_name} and place it on the {position} container with respect to the robot"
        
        else: #index == 3:
            # Furthest container, right if not inverted
            position = "furthest"
            if inverted:
                reference = Dummy("placeL")
                self.success_sensor = self.sensorL
            else:
                reference = Dummy("placeR")
                self.success_sensor = self.sensorR
            instruction = f"Pick up the {obj_name} and place it on the {position} container with respect to the robot"
        
        # Set waypoint3 to appropriate position (keeping grasping orientation)
        self.way3.set_position(reference.get_position())

        # If the task is in "inverted" condition, we can flip orientations of approach, grasp an place positions
        if inverted:
            flip = [0, 0, 1, 0] # (x, y, z, w) quaternion -> flip 180° around z axis
            self.approach_dummy.set_quaternion(flip, self.approach_dummy)
            self.grasp_dummy.set_quaternion(flip, self.grasp_dummy)
            self.way3.set_quaternion(flip, self.way3)
            reference.set_quaternion(flip, reference)   # At the moment is useful to do this since we only use its position

        # Register success condition
        success_condition = DetectedCondition(self.object, self.success_sensor)
        self.register_success_conditions([success_condition])
        
        #print(instruction)
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
        