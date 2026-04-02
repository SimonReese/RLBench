from turtle import pos, position
from typing import List, Sequence
from rlbench.backend.task import Task

from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.waypoints import Waypoint
from rlbench.const import colors
from rlbench.backend.task_utils import sample_procedural_objects
from rlbench.backend.conditions import ConditionSet, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object


class UnloadContainer(Task):

    def init_task(self) -> None:
        
        # Get handles to relevant objects
        self.containerC = Shape("containerC")
        self.containerL = Shape("containerL")
        self.containerR = Shape("containerR")

        self.sensorL = ProximitySensor("sensorL")
        self.sensorR = ProximitySensor("sensorR")

        self.spawn = SpawnBoundary([Shape("spawn")])

        self.way0 = Dummy("waypoint0") # approach waypoint
        self.way1 = Dummy("waypoint1") # grasping waypoint
        self.way2 = Dummy("waypoint2") # depart waypoint
        self.way3 = Dummy("waypoint3") # release waypoint

        # Register
        self.register_waypoint_ability_start(1, self._move_above_object)
        self.register_waypoints_should_repeat(self._repeat)
        self.success_conditions = []
        self.sampled = []
        self.init_episode_objs = []

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        self.spawn.clear()
        self.success_conditions = []
        # get reference placement position
        if index == 0:
            reference = Dummy("placeL")
            self.success_sensor = self.sensorL
            position_str = "left"
        else:
            reference = Dummy("placeR")
            self.ssuccess_sensor = self.sensorR
            position_str = "right"

        # Sample objects
        
        self.sampled = sample_procedural_objects(self.get_base(), 2)
        self.remaining_objs = list(self.sampled)    # we need a deepcopy list, otherwise a shallow copy will 
                                                    # remove objs from sampled when remaining.remnove() is called in 
                                                    # step(), causing the cleanup() to fail
        # Register all objs as graspable
        self.register_graspable_objects(self.sampled)
        # Place objs
        
        for obj in self.sampled:
            obj.set_position(
                position=[0, 0, 0.2],
                relative_to=self.containerC,
                reset_dynamics=False
            )
            self.spawn.sample(obj, ignore_collisions=True, min_distance=0.05)

            # Create success condition
            self.success_conditions.append(DetectedCondition(obj, self.success_sensor))            
        
        # Move target position
        self.way3.set_position(reference.get_position())
        
        # We randomly color our containers
        colorL = np.random.choice(len(colors))
        colorR = np.random.choice(len(colors))
        
        self.containerL.set_color(colors[colorL][1])
        self.containerR.set_color(colors[colorR][1])
        
        # Register success conditions
        self.register_success_conditions([ConditionSet(self.success_conditions)])

        instruction_str = f"Pick all the objects from middle container and place them on the {position} container"
        return [instruction_str]

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 2

    def step(self) -> None:
        # Update the list of remaining objects
        for obj in self.remaining_objs:
            if self.success_sensor.is_detected(obj):
                self.remaining_objs.remove(obj)
                print(f"Detected {obj.get_name()} in {self.success_sensor.get_name()}")


    def cleanup(self) -> None:
        # Remove every objs from the scene
        for obj in self.sampled:
            if obj.still_exists(): obj.remove()
        self.sampled = []
        

    def _move_above_object(self, waypoint: Waypoint):
        target_obj = self.remaining_objs[0]
        # Question: is it possible to move objs during the simulation? We are trying to move the Dummy directly
        #self.way1.set_position(target_obj.get_position())
        # Fail-Safe: move as proposed in tutorial
        waypoint.get_waypoint_object().set_position(target_obj.get_position())
        #if self.way1.get_position() != waypoint.get_waypoint_object().get_position():
            #print(f"Warning: setting waypoint1 dummy failed: {self.way1.get_position()} while {waypoint.get_waypoint_object().get_position()}")
        pass

    def _repeat(self) -> bool:
        return len(self.remaining_objs) > 0