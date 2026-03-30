from typing import List
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor


class SlideBlock(Task):

    def init_task(self) -> None:
        self._block = Shape('block')
        self._target = ProximitySensor('success')

        self.register_success_conditions([
            DetectedCondition(self._block, self._target)])

    def init_episode(self, index: int) -> List[str]:
        # TODO: This is called at the start of each episode.
        self._variation_index = index
       
        return ['slide the block to target']



    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 1

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass
