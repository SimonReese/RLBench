from typing import List
import numpy
from rlbench.backend.conditions import ConditionSet, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task

from rlbench.const import colors
from pyrep.objects.shape import Shape
from pyrep.objects import Object
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task_utils import sample_procedural_objects

class ComplexTask(Task):

    def init_task(self) -> None:
        self.large_container = Shape("large_container")
        self.small_cont1 = Shape("small_cont1")
        self.small_cont2 = Shape("small_cont2")
        self.spawn: List[Object] = [Shape("spawn")]
        self.det1 = ProximitySensor("sensor1")
        self.det2 = ProximitySensor("sensor2")

        self.register_waypoint_ability_start(1, self._move_above_object)
        self.register_waypoints_should_repeat(self._repeat)

    def init_episode(self, index: int) -> List[str]:
        # TODO: This is called at the start of each episode.
        self._variation_index = index
        
        self.bin_objects = sample_procedural_objects(self.get_base(), 5)
        self.bin_objects_not_done = list(self.bin_objects)
        self.register_graspable_objects(self.bin_objects)
        spawn = SpawnBoundary(self.spawn)
        ob: Object
        for ob in self.bin_objects:
            ob.set_position(
                position=[0, 0, 0.2],
                relative_to=self.large_container,
                reset_dynamics=False
            )
            spawn.sample(ob, ignore_collisions=True, min_distance=0.05)

        target_waypoint = Dummy("waypoint3")
        target_pos = [0, 0, 0.17]

        conditions = []
        target_color, target_rgb = colors[index]
        color_choice = numpy.random.choice(
            list(range(index)) + 
            list(range(index + 1, len(colors))),
            size=1,
            replace=False
        )[0]
        _, distractor_rgb = colors[color_choice]
        if index % 2 == 0:
            self.small_cont1.set_color(target_color)
            self.small_cont2.set_color(distractor_rgb)
            for ob in self.bin_objects:
                conditions.append(DetectedCondition(ob, self.det1))
            target_waypoint.set_position(
                position=target_pos,
                relative_to=self.small_cont1,
                reset_dynamics=True
            )

        else:
            self.small_cont2.set_color(target_color)
            self.small_cont1.set_color(distractor_rgb)
            for ob in self.bin_objects:
                conditions.append(DetectedCondition(ob, self.det2))
            target_waypoint.set_position(
                position=target_pos,
                relative_to=self.small_cont2,
                reset_dynamics=True
            )
            
        self.register_success_conditions([ConditionSet(conditions, simultaneously_met=True)])
        return ['empty the container in the to %s container'
        % target_color,
        'clear all items from the large tray and put them in the %s '
        'tray' % target_color,
        'move all objects from the large container and drop them into '
        'the smaller %s one' % target_color,
        'remove whatever you find in the big box in the middle and '
        'leave them in the %s one' % target_color,
        'grasp and move all objects into the %s container'
        % target_color]

    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 2*len(colors)

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        for ob in self.bin_objects_not_done:
            if self._variation_index % 2 == 0:
                if self.det1.is_detected(ob):
                    self.bin_objects_not_done.remove(ob)
            else:
                if self.det2.is_detected(ob):
                    self.bin_objects_not_done.remove(ob)
        pass

    def _move_above_object(self, waypoint):
        if len(self.bin_objects_not_done) <= 0:
            raise RuntimeError('Should not be here.')
        x, y, z = self.bin_objects_not_done[0].get_position()
        waypoint.get_waypoint_object().set_position([x, y, z])

    def _repeat(self):
        return len(self.bin_objects_not_done) > 0

    def cleanup(self) -> None:
        if self.bin_objects is not None:
            [ob.remove() for ob in self.bin_objects if ob.still_exists()]
            self.bin_objects = []
