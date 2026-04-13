from pickle import OBJ
import time
from typing import List
import numpy
from rlbench.backend.task import Task
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.waypoints import Waypoint
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition

class ContainerPlace(Task):
    """ Place one of the two objects on the sides of the table in one of the three positions 
        in the middle of the table, with respect to the objecs (in front, behind or in between them)
        Variations:
        - WRT Robot:
            - 0: middle
            - 1: front
            - 2: behind
        - WRT Object:
            - 3: middle
            - 4: front
            - 5: behind
    """

    # Available spawn positions
    POSITIONS = [
        "spawnL",
        "spawnR",
        "placeA",
        "placeB"
    ]

    # Default objects
    OBJECTS = [
        "cartoy",
        "controller",
        "pcmouse",
        "mug"
    ]

    def init_task(self) -> None:

        self.spawnL = SpawnBoundary([Shape("spawnL")])
        self.spawnR = SpawnBoundary([Shape("spawnR")])

        self.scene_placement = Dummy("placement_positions") # root for placements

        self.way0 = Dummy("waypoint0") # Approach
        self.way1 = Dummy("waypoint1") # Grasp
        self.way2 = Dummy("waypoint2") # Depart
        self.way3 = Dummy("waypoint3") # Place

        self.target_object = None
        self.target_name = ""
        self.target_position = None

        self.register_waypoint_ability_start(0, self._start)
        self.grasp_reversed = False

    def init_episode(self, index: int) -> List[str]:
        """
            Variations:
                - WRT Robot:
                    - 0: middle
                    - 1: front
                    - 2: behind
                - WRT Object:
                    - 3: middle
                    - 4: front
                    - 5: behind
        """

        # NOTE: this code is messy and requires a lot of refactoring
        # TODO: fix the messy code

        OBJECTS = list(self.OBJECTS)
        POSITIONS = list(self.POSITIONS)
        OBJ_POS_MAPPING = []
        inverted = False

        self.spawnL.clear()
        self.spawnR.clear()

        # 0: Choose which side to pick
        # 1: Spawn objects in positions
        # 2: Adjust grasping positions
        # 3: Translate scene depending on variation and choose target position
        # 4: Rotate placement depending on variation
        # 5: Generate instructions
        # 6: Set success conditions
        
        # 0: Choose which side to pick
        txt_pick_position = ""
        if numpy.random.random() < 0.5:
            target_box = "spawnL"
            txt_pick_position = "left"
        else:
            target_box = "spawnR"
            txt_pick_position = "right"

        middle_reference = None
        if index == 3:
            # Middle wrt objects, we choose one object as reference 
            # Choose one of the two objects in the middle
                middle_reference = numpy.random.choice(("placeA", "placeB"))

        # 1: Spawn objects in positions
        # Spaw objects randomly
        for position in POSITIONS:
            
            # Pick random object
            idx = numpy.random.randint(len(OBJECTS))
            object_name = OBJECTS.pop(idx)
            obj = Shape(object_name)

            # Place object
            if position not in ("spawnL", "spawnR"):
                location = Dummy(position)
                obj.set_parent(location) # Set object parent accordingly
                obj.set_position(location.get_position())
                # Randomly set front or backward orientation
                z = numpy.random.choice((0, numpy.pi))
                obj.set_orientation([0, 0, z], location)
                
                # Set inverted if necessary
                if index == 3 and position == middle_reference and z == 0: inverted = True
                elif index == 4 and position == "placeB" and z == 0: inverted = True
                elif index == 5 and position == "placeA" and z == 0: inverted = True
            elif position == "spawnL":
                self.spawnL.sample(obj)
            else: #position == "spawnR":
                self.spawnR.sample(obj)

            # Store object to position
            OBJ_POS_MAPPING.append((object_name, position))
            # If we are in target box, we store the object as target
            if position == target_box:
                self.target_object = obj
                self.target_name = obj.get_name()
                self.register_graspable_objects([obj])
        assert self.target_object is not None

        # 2: Adjust grasping positions
        # Check if we need to rotate grasp and approach pose for target object
        gripper = Dummy("Panda_target")
        grasp_pose = Dummy(f"{self.target_name}_top_grasp_pose")
        approach_pose = Dummy(f"{self.target_name}_top_approach_pose")
        if abs(grasp_pose.get_orientation(gripper)[2]) > numpy.pi/2:
            # Flip grasp and approach points
            grasp_pose.set_orientation([0, 0, numpy.pi], grasp_pose)
            approach_pose.set_orientation([0, 0, numpy.pi], approach_pose)
            self.grasp_reversed = True
            
        # 3: Translate scene depending on variation    
        # variations: middle, front and back
        # Move the scene accordingly
        if index in (0, 3):
            scene_placement = Dummy("placement_positions_central")
            self.target_position = Dummy("position_central")
        elif index in (1, 4):
            scene_placement = Dummy("placement_positions_front")
            self.target_position = Dummy("position_front")
        else:
            scene_placement = Dummy("placement_positions_back")
            self.target_position = Dummy("position_back")

        # Move the scene accordingly
        self.scene_placement.set_position(scene_placement.get_position())

        # 4: Rotate placement depending on variation
        # Now, rotate the scene if necessary
        if index in (3, 4, 5):
            rotation_z = numpy.random.uniform(-numpy.pi/6, numpy.pi/6)
            self.scene_placement.set_orientation([0, 0, rotation_z], self.scene_placement)

        # 5: Generate instructions
        # variations: middle, front and back
        instruction = ""
        txt_position = ""
        txt_reference = ""
        if index in (0, 1, 2):
            # WRT Robot
            txt_reference = "with respect to the robot"
            if index == 0: # middle
                txt_position = "in between the objects in the center of the table"
            elif index == 1: # front
                txt_position = "in front of the two objects the center of the table"
            else: # behind
                txt_position = "behind the two objects the center of the table"
        else:
            # WRT objects
            if index == 3: # middle
                if middle_reference == "placeA": obj_name, _ = OBJ_POS_MAPPING[2]
                else:  obj_name, _ = OBJ_POS_MAPPING[3]
                if (middle_reference == "placeA" and not inverted) or (middle_reference == "placeB" and inverted):
                    txt_position = f"in front of the {obj_name}"
                else:
                    txt_position = f"behind of the {obj_name}"   
            elif index == 4: # Front
                # Check the object at LOCB
                obj_name, _ = OBJ_POS_MAPPING[3]
                txt_position = f"in front of the {obj_name}"
                if inverted: txt_position = f"behind the {obj_name}"
            else: # back
                obj_name, _ = OBJ_POS_MAPPING[2]
                txt_position = f"behind the {obj_name}"
                if inverted: txt_position = f"in front of the {obj_name}"
            txt_reference = f"with respect to the {obj_name}"


        instruction = (
            f"Pick the object from the {txt_pick_position} side of the table with respect to the robot, " +
            f"and place it {txt_position}, " + 
            f"{txt_reference}"
        )
        #print(instruction)

        # 6: Success conditions
        if index in (0, 3):
            success_sensor = ProximitySensor("central_sensor")
        elif index in (1, 4):
            success_sensor = ProximitySensor("front_sensor")
        else:
            success_sensor = ProximitySensor("back_sensor")
        condition = DetectedCondition(self.target_object, success_sensor)
        self.register_success_conditions([condition])

        return [instruction]

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 6

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        if self.grasp_reversed:
            grasp_pose = Dummy(f"{self.target_name}_top_grasp_pose")
            approach_pose = Dummy(f"{self.target_name}_top_approach_pose")
            grasp_pose.set_orientation([0, 0, numpy.pi], grasp_pose)
            approach_pose.set_orientation([0, 0, numpy.pi], approach_pose)
        self.grasp_reversed = False


    def is_static_workspace(self) -> bool:
        return True
    
    def _start(self, waypoint: Waypoint):
        assert self.target_object is not None
        assert self.target_position is not None
        self.way0.set_pose(
            Dummy(f"{self.target_name}_top_approach_pose").get_pose()
        )
        self.way1.set_pose(
            Dummy(f"{self.target_name}_top_grasp_pose").get_pose()
        )
        self.way2.set_pose(
            Dummy(f"{self.target_name}_top_approach_pose").get_pose()
        )
        self.way3.set_pose( # Set pose as grasping for orientation
            Dummy(f"{self.target_name}_top_grasp_pose").get_pose()
        )

        self.way3.set_position(self.target_position.get_position())