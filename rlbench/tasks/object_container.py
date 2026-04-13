from typing import List, Tuple

import numpy
from pyrep.backend.sim import simLoadModel
from pyrep.objects import Dummy, ProximitySensor, Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from rlbench.backend.waypoints import Waypoint
from pyrep.backend import sim


class ObjectContainer(Task):
    """ Place an object in one of the four container around it

        Variations WRT object (all possible roations):
            - 0: left container
            - 1: right container
            - 2: front container
            - 3: back container
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

        # Gripper dummy
        self.gripper = Dummy("Panda_target")

        # Waypoints
        self.way0 = Dummy("waypoint0") # approach waypoint
        self.way1 = Dummy("waypoint1") # grasping waypoint
        self.way2 = Dummy("waypoint2") # depart waypoint
        self.way3 = Dummy("waypoint3") # release waypoint

        # Proximity sensors
        self.sensorL = ProximitySensor("sensorL")
        self.sensorR = ProximitySensor("sensorR")
        self.sensorFront = ProximitySensor("sensorFront")
        self.sensorBack = ProximitySensor("sensorBack")

        # Call function after generic approach
        self.register_waypoint_ability_start(0, self._start)
        
        self.object = None
        self.approach_dummy = None
        self.grasp_dummy = None
        self.success_sensor = None
        self.hidden: list[Shape] = []

    def init_episode(self, index: int) -> List[str]:
        """
            Variations WRT object (all possible roations):
            - 0: left container
            - 1: right container
            - 2: front container
            - 3: back container
        """
        # Place bject in any rotation
        self.spawn_boundary.clear()
        self.spawn_boundary.sample(self.obj_place)

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
            position = "front"
            reference = Dummy("placeFront")
            self.success_sensor = self.sensorFront
            instruction = f"Pick up the {obj_name} and place it on the container in {position} of it"
        else: # index == 3:
            position = "behind"
            reference = Dummy("placeBack")
            self.success_sensor = self.sensorBack
            instruction = f"Pick up the {obj_name} and place it on the container {position} it"
        
        # Randomly remove unused boxes
        boxes = ["boxL", "boxR", "boxFront", "boxBack"]
        for idx, box in enumerate(boxes):
            if idx == index: continue
            if numpy.random.random() > 0.5:
                obj = Shape(box)
                sim.simSetObjectInt32Parameter(obj.get_handle(), sim.sim_objintparam_visibility_layer, 0)
                self.hidden.append(obj)
                obj.set_renderable(False)
        
        # Set waypoint3 to appropriate position (keeping grasping orientation)
        self.way3.set_position(reference.get_position())

        # Check if we need to flip orientations of approach, grasp an place positions
        _, _, z = self.grasp_dummy.get_orientation(self.gripper)
        if abs(z) > numpy.pi/2:
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
        # Restore hidden boxes
        for box in self.hidden:
            box.set_renderable(True)
            value:int = sim.simGetObjectInt32Parameter(box.get_handle(), sim.sim_objintparam_visibility_layer)
            #print(f"Box has visibility value: {value}({'0:b'.format(value)})")
            sim.simSetObjectInt32Parameter(box.get_handle(), sim.sim_objintparam_visibility_layer, 1)
    
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
        