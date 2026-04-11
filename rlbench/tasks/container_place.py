from pickle import OBJ
import time
from typing import List
import numpy
from rlbench.backend.task import Task
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from rlbench.backend.waypoints import Waypoint


class ContainerPlace(Task):

    # Available spawn positions
    POSITIONS = [
        "placeL",
        "placeR",
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
        self.scene_placement = Dummy("placement_positions") # root for placements

        self.way0 = Dummy("waypoint0") # Approach
        self.way1 = Dummy("waypoint1") # Grasp
        self.way2 = Dummy("waypoint2") # Depart
        self.way3 = Dummy("waypoint3") # Place

        self.target_object = None
        self.target_position = None

        self.register_waypoint_ability_start(0, self._start)

    def init_episode(self, index: int) -> List[str]:
        OBJECTS = list(self.OBJECTS)
        POSITIONS = list(self.POSITIONS)
        OBJ_POS_MAPPING = []

        # Select which box
        if numpy.random.random() < 0.5:
            target_box = "placeL"
        else:
            target_box = "placeR"

        # Spaw objects randomly
        for position in POSITIONS:
            location = Dummy(position)
            # Pick random object
            idx = numpy.random.randint(len(OBJECTS))
            object_name = OBJECTS.pop(idx)

            # Place object
            obj = Shape(object_name)
            obj.set_position(location.get_position())
            obj.set_parent(location) # Set object parent accordingly
            # TODO: can we set ^ above with keep_position = False to let position adjust in one call?

            # Store object to position
            OBJ_POS_MAPPING.append((object_name, position))
            # If we are in target box, we store the object as target
            if position == target_box:
                self.target_object = obj
            

        #self.target_object = Shape("cartoy")
              
        # 3 variations: middle, front and back
        # Move the scene accordingly
        if index == 0:
            scene_placement = Dummy("placement_positions_central")
            self.target_position = Dummy("position_central")
        elif index == 1:
            scene_placement = Dummy("placement_positions_front")
            self.target_position = Dummy("position_front")
        else:
            scene_placement = Dummy("placement_positions_back")
            self.target_position = Dummy("position_back")

        # Move the scene accordingly
        self.scene_placement.set_position(scene_placement.get_position())
        
        # Set waypoints
        self.way3.set_pose( # Set pose as grasping for orientation
            Dummy(f"{self.target_object.get_name()}_top_grasp_pose").get_pose()
        )

        return ['']

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 3

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass

    def is_static_workspace(self) -> bool:
        return True
    
    def _start(self, waypoint: Waypoint):
        assert self.target_object is not None
        assert self.target_position is not None
        self.way0.set_pose(
            Dummy(f"{self.target_object.get_name()}_top_approach_pose").get_pose()
        )
        self.way1.set_pose(
            Dummy(f"{self.target_object.get_name()}_top_grasp_pose").get_pose()
        )
        self.way2.set_pose(
            Dummy(f"{self.target_object.get_name()}_top_approach_pose").get_pose()
        )
        self.way3.set_pose( # Set pose as grasping for orientation
            Dummy(f"{self.target_object.get_name()}_top_grasp_pose").get_pose()
        )

        self.way3.set_position(self.target_position.get_position())