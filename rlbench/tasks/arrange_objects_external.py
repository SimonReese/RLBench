import os
from typing import List, Union
import numpy
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.waypoints import Waypoint

ASSETS_DIR = os.path.join(os.path.dirname(__file__), '../assets')

class ArrangeObjectsExternal(Task):
    """Arrange an object next to the central one

        Variations:
            - 0: Front          (wrt robot)
            - 1: Behind
            - 2: Left
            - 3: Right
            - 4: Front          (wrt front camera)
            - 5: Behind
            - 6: Left
            - 7: Right          

        In the scene, sensors are placed:

        +------ROBOT------+
        |-------}{--------|
        |-------[B]-------|
        |---[R]-OBJ-[L]---|
        |-------[F]-------|
        +-----------------+
    """
    VARIATION_MAPPING_STRINGS = [
        # position wrt robot
        "in front of",
        "behind",
        "in on the left side of",
        "in on the right side of",
        # wrt camera
        "in front of",
        "behind",
        "in on the left side of",
        "in on the right side of"
    ]

    VARIATION_MAPPING_SENSORS = [
        # sensors wrt robot
        "behind",
        "front",
        "left",
        "right",
        # sensors wrt camera
        "front",
        "behind",
        "right",
        "left"
    ]

    OBJECTS = [
        "cartoy",
        "controller",
        "mug",
        "pcmouse",
        "tvremote",
    ]

    def init_task(self) -> None:
        # TODO: This is called once when a task is initialised.
        self.spawnL = SpawnBoundary([Shape("spawnL")])
        self.spawnR = SpawnBoundary([Shape("spawnR")])
        self.spawnBoundary = SpawnBoundary([Shape("spawnBoundary")])
        
        self.imported_models: List[Object]= []
        self.reference_model: Union[Object, None] = None
        self.target_object: Union[Object, None] = None
        # Grasping dummies
        self.approach_dummy: Union[Dummy, None] = None
        self.grasp_dummy: Union[Dummy, None] = None

        self.way0 = Dummy("waypoint0") # Approach
        self.way1 = Dummy("waypoint1") # Grasp
        self.way2 = Dummy("waypoint2") # Depart
        self.way3 = Dummy("waypoint3") # Reach
        self.way4 = Dummy("waypoint4") # Place

        self.sensors_root = Dummy("sensors_root")
        self.target_sensor: Union[ProximitySensor, None] = None

        self.register_waypoint_ability_start(0, self._start)


    def init_episode(self, index: int) -> List[str]:
        # Ensure task is model free
        assert len(self.imported_models) == 0
        assert self.reference_model is None
        assert self.target_object is None
        assert self.target_sensor is None
        
        # Select 3 random objects
        selected = numpy.random.choice(self.OBJECTS, size=3, replace=False)
        print(f"Spawning {selected[0]}")
        # Import models
        for model in selected:
            selected_path = f"{ASSETS_DIR}/{model}/{model}.ttm"
            imported = self.pyrep.import_model(selected_path)
            self.imported_models.append(imported)
            imported.set_parent(self.get_base())
            imported.set_position(numpy.array(self.get_base().get_position()) + numpy.array([0.5, 0.5, 0.1])) # try setting quota
        self.reference_model = self.imported_models[0]
        # Place objects
        self.spawnL.clear()
        self.spawnR.clear()
        self.spawnBoundary.clear()
        self.spawnBoundary.sample(self.reference_model) # Model 0 on center
        self.spawnL.sample(self.imported_models[1])                   # Model 1 on left wrt robot
        self.spawnR.sample(self.imported_models[2])                   # Model 2 on right

        # Register objects as graspable
        self.register_graspable_objects(self.imported_models[1:])

        # Select pick object
        pick_sides = ["left", "right"]
        pick_side = numpy.random.choice(pick_sides)
        pick_str = f"Pick the object on the {pick_side} side with respect to the robot" # ...
        # Store target object
        self.target_object = self.imported_models[1] if pick_side == "left" else self.imported_models[2]

        # Depending on variation, select target sensor
        self.target_sensor = ProximitySensor(f"proximity_{self.VARIATION_MAPPING_SENSORS[index]}")

        # Construct string
        place_str = f"place it {self.VARIATION_MAPPING_STRINGS[index]} the central object"

        # Place waypoints done in _start after falling of objects
        # but store approaching dummies
        self.approach_dummy = Dummy(f"{self.target_object.get_name()}_top_approach_pose")
        self.grasp_dummy = Dummy(f"{self.target_object.get_name()}_top_grasp_pose")
        
        self.register_success_conditions([  
            DetectedCondition(self.target_object, self.target_sensor),  
            NothingGrasped(self.robot.gripper)  
        ])  
        
        if index < 4:
            task_instr = f"{pick_str} and {place_str} with respect to the robot."
        else:
            task_instr = f"{pick_str} and {place_str} with respect to the external camera."
        print(f"Variation: {index}\nInstruction: {task_instr}")
        return [task_instr]

    def variation_count(self) -> int:
        return 8
    
    def is_static_workspace(self) -> bool:  
        return True
    
    # def success(self) -> Tuple[bool, bool]:
    #     if self.target_sensor is not None and self.target_object is not None:  
    #         detected = self.target_sensor.is_detected(self.target_object)  
    #         grasped = len(self.robot.gripper.get_grasped_objects())  
    #         if detected or grasped > 0:  
    #             print(f"[success] detected={detected}, grasped={grasped}")
    #     return super().success()

    def cleanup(self) -> None:  
        for obj in self.imported_models:
            obj.remove()
        self.imported_models.clear()
        self.target_object = None
        self.target_sensor = None
        self.reference_model = None
            
    def _start(self, waypoint: Waypoint):
        assert self.target_object is not None
        assert self.reference_model is not None
        assert self.approach_dummy is not None
        assert self.grasp_dummy is not None
        assert self.target_sensor is not None

        # Place sensor_root at model position along xy
        ref_position = self.reference_model.get_position()  
        sensors_pose = self.sensors_root.get_pose()  
        sensors_pose[0] = ref_position[0]   # x  
        sensors_pose[1] = ref_position[1]   # y    
        self.sensors_root.set_position(sensors_pose)

        # Set destination waypoints position and orientation for all (mainly for quota and grasping orientation)
        self.way0.set_pose(self.approach_dummy.get_pose())  # Approach
        self.way1.set_pose(self.grasp_dummy.get_pose())     # Grasp
        self.way2.set_pose(self.approach_dummy.get_pose())  # Depart
        self.way3.set_pose(self.grasp_dummy.get_pose())     # Reach 
        self.way4.set_pose(self.grasp_dummy.get_pose())     # Place

        if self._check_inverted_grasp_pose(self.way0):
            self._reverse_grasp_pose([self.way0, self.way1, self.way2, self.way3, self.way4])

        # Refine positions for place
        APPROACH_PLACE_OFFSET_Z = [0, 0, 0.2]
        PLACE_OFFSET_Z = [0, 0, 0.05]
        self.way3.set_position(numpy.array(self.target_sensor.get_position()) + numpy.array(APPROACH_PLACE_OFFSET_Z))
        self.way4.set_position(numpy.array(self.target_sensor.get_position()) + numpy.array(PLACE_OFFSET_Z))

    def _check_inverted_grasp_pose(self, grasp_waypoint: Dummy):
        pose = grasp_waypoint.get_pose()
        # Check if x is aligned with world x or opposite -> gripper default is opposite
        qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        x_axis_x = 1.0 - 2.0 * (qy**2 + qz**2)
        # gripper has x = -x_world → if x_axis_x > 0 gripper is opposite
        if x_axis_x > 0: # gripper is opposite
            return True
        return False
        
    def _reverse_grasp_pose(self, waypoints: List[Dummy]):
        for waypoint in waypoints:
            pose = waypoint.get_pose()
            qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]     
            # flip around [0,0,1,0]
            pose[3], pose[4], pose[5], pose[6] = qy, -qx, qw, -qz  
            waypoint.set_pose(pose)