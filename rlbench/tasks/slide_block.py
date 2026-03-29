from typing import List
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor


class SlideBlock(Task):

    def init_task(self) -> None:
        # TODO: This is called once when a task is initialised.
        self.block = Shape("block")
        self.target = Shape("target")
        self.boundary = SpawnBoundary([Shape("boundary")])
        success_detector = ProximitySensor("Success")
        success_condition = DetectedCondition(self.block, success_detector)
        self.register_success_conditions([success_condition])
        pass

    def init_episode(self, index: int) -> List[str]:
        # TODO: This is called at the start of each episode.
        self._variation_index = index
        block_color_name = "red"
        #block_color_name, block_rgb = colors[index]
        #self.block.set_color(block_rgb)

        # Remove all objects and replace them
        self.boundary.clear()
        self.boundary.sample(self.target)
        return ['slide the %s block to target' % block_color_name,
        'push the %s cube to the red plane' % block_color_name,
        'nudge the %s block so that it covers the red target' % block_color_name,
        'Find the %s item on the table and manipulate its position so that it reaches the red plane' % block_color_name]



    def variation_count(self) -> int:
        # TODO: The number of variations for this task.
        return 1

    def step(self) -> None:
        # Called during each sim step. Remove this if not using.
        pass

    def cleanup(self) -> None:
        # Called during at the end of each episode. Remove this if not using.
        pass
