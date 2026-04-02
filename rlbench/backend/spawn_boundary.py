from typing import List
import numpy as np
import math
from rlbench.backend.exceptions import BoundaryError
from pyrep.objects.object import Object


class BoundingBox(object):
    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float,
                 min_z: float, max_z: float):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.min_z = min_z
        self.max_z = max_z
        self.points = [[min_x, min_y, min_z], [max_x, min_y, min_z],
                       [min_x, max_y, min_z], [max_x, max_y, min_z],
                       [min_x, min_y, max_z], [max_x, min_y, max_z],
                       [min_x, max_y, max_z], [max_x, max_y, max_z]]

    def rotate(self, theta: np.ndarray) -> 'BoundingBox':
        r_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])
        r_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])
        r_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])
        r = np.dot(r_z, np.dot(r_y, r_x))
        new_points = np.dot(self.points, r)
        return BoundingBox(np.amin(new_points[:, 0]), np.amax(new_points[:, 0]),
                           np.amin(new_points[:, 1]), np.amax(new_points[:, 1]),
                           np.amin(new_points[:, 2]), np.amax(new_points[:, 2]))

    def within_boundary(self, boundary: 'BoundingBox', is_plane: bool) -> bool:
        z_cond = True
        if not is_plane:
            z_cond = self.min_z > boundary.min_z and self.max_z < boundary.max_z
        return (self.min_x > boundary.min_x and self.max_x < boundary.max_x and
                self.min_y > boundary.min_y and self.max_y < boundary.max_y and
                z_cond)


class BoundaryObject(object):

    def __init__(self, boundary: Object):
        self._boundary = boundary
        self._is_plane = False
        self._contained_objects = []

        if boundary.is_model():
            (minx, maxx, miny,
             maxy, minz, maxz) = boundary.get_model_bounding_box()
        else:
            minx, maxx, miny, maxy, minz, maxz = boundary.get_bounding_box()
        self._boundary_bbox = BoundingBox(minx, maxx, miny, maxy, minz, maxz)

        height = np.abs(maxz - minz)
        if height == 0:
            height = 1.0
            self._is_plane = True
        self._area = np.abs(maxx - minx) * np.abs(maxy - miny) * height

    def _get_position_within_boundary(self, obj: Object, obj_bbox: BoundingBox
                                      ) -> List[float]:
        x = np.random.uniform(
            self._boundary_bbox.min_x + np.abs(obj_bbox.min_x),
            self._boundary_bbox.max_x - np.abs(obj_bbox.max_x))
        y = np.random.uniform(
            self._boundary_bbox.min_y + np.abs(obj_bbox.min_y),
            self._boundary_bbox.max_y - np.abs(obj_bbox.max_y))
        if self._is_plane:
            _, _, z = obj.get_position(self._boundary)
        else:
            z = np.random.uniform(
                self._boundary_bbox.min_z + np.abs(obj_bbox.min_z),
                self._boundary_bbox.max_z - np.abs(obj_bbox.max_z))
        return [x, y, z]

    def get_area(self) -> float:
        return self._area

    def add(self, obj: Object, ignore_collisions: bool = False,
            min_rotation: tuple = (0.0, 0.0, -3.14),
            max_rotation: tuple = (0.0, 0.0, 3.14),
            min_distance: float = 0.01) -> int:
        """ Adds object inside the boundary, setting its position

        :params obj: the Object to place in the Boundary. 
        :params ignore_collsions: if ignore collisions when placing the object in the bboxes.
            Set to true if you are happy with things being stacked on top of each other
        :params min_rotation: a tuple (x, y, z) of minimum possible rotation along each axes
        :params max_rotation: a tuple (x, y, z) of maximum possible rotation along each axes.
            set rotation range to be (0, 0, 0) -> (0, 0, 0) if you don't want it to rotate
        :params min_distance: min distance between objects
        
        
        :returns 1: if the object is placed correctly
        :returns -1: if the obj doesn't fit in the bbox
        :returns -2: if the obj collides witgh another
        :returns -3: if the obj is too close to another obj

        """

        # Rotate the bounding box randomly
        if obj.is_model():
            bb = obj.get_model_bounding_box()
        else:
            bb = obj.get_bounding_box()
        obj_bbox = BoundingBox(*bb)
        rotation = np.random.uniform(list(min_rotation), list(max_rotation))
        obj_bbox = obj_bbox.rotate(rotation)

        if not obj_bbox.within_boundary(self._boundary_bbox, self._is_plane):
            # We rotated the object bbox, but it doesnt fit inside this bbox
            return -1
        # Set obj position and rotiation wrt this boundary
        new_pos = self._get_position_within_boundary(obj, obj_bbox)
        obj.set_position(new_pos, self._boundary)
        obj.rotate(list(rotation))
        new_pos = np.array(new_pos)

        # Check if collinding with any other objects inside this bbox
        if not ignore_collisions:
            for contained_obj in self._contained_objects:
                # Check for collision between each child
                for cont_ob in contained_obj.get_objects_in_tree(
                        exclude_base=False):
                    for placing_ob in obj.get_objects_in_tree(
                            exclude_base=False):
                        if placing_ob.check_collision(cont_ob):
                            return -2   # Collision detected
                # Check min distance
                dist = np.linalg.norm(
                    new_pos - contained_obj.get_position(self._boundary))
                if dist < min_distance:
                    return -3   # min distance check fail
            self._contained_objects.append(obj)
        return 1

    def clear(self) -> None:
        self._contained_objects = []


class SpawnBoundary(object):
    """A collection of one or more boundary boxes where objects can be placed"""

    MAX_SAMPLES = 100
    """Maxumim number of retries when placing an object"""

    def __init__(self, boundaries: List[Object]):
        """Creates the spawn boundary

        :params boundaries: a List[Objects] that contains the possible spawn locations
        """
        self._boundaries = []
        areas = []
        for b in boundaries:
            bo = BoundaryObject(b)
            areas.append(bo.get_area())
            self._boundaries.append(bo)
        self._probabilities = np.array(areas) / np.sum(areas)

    def sample(self, obj: Object, ignore_collisions=False,
               min_rotation=(0.0, 0.0, -3.14), max_rotation=(0.0, 0.0, 3.14),
               min_distance=0.01) -> None:
        """ Set object to a random position in random boundary box in this SpawnBoundary

        Tries to place an object up to 100 times (see SpawnBoundary.MAX_SAMPLES)

        :params obj: the Object to place in the SpawnBoundary. 
            Its position will be set to be inside one of the boundary boxes provided when constructing this SpawnBoundary
        :params ignore_collsions: if ignore collisions when placing the object in the bboxes.
            Set to true if you are happy with things being stacked on top of each other
        :params min_rotation: a tuple (x, y, z) of minimum possible rotation along each axes
        :params max_rotation: a tuple (x, y, z) of maximum possible rotation along each axes.
            set rotation range to be (0, 0, 0) -> (0, 0, 0) if you don't want it to rotate
        :params min_distance: min distance between objects

        :raise BoundaryError: if obj could not be placed within boundary due to sizes
        :raise BoundaryError: if obj could not be placed the object within the boundary due 
            to collision with other objects in the boundary
        """
        collision_fails = boundary_fails = self.MAX_SAMPLES
        while collision_fails > 0 and boundary_fails > 0:
            sampled_boundary: BoundaryObject = np.random.choice(self._boundaries,
                                                p=self._probabilities)
            result = sampled_boundary.add(
                obj, ignore_collisions, min_rotation, max_rotation, min_distance)
            # each fail decrements attempts
            if result == -1:
                boundary_fails -= 1
            elif result == -2:
                collision_fails -= 1
            elif result == -3:
                boundary_fails -= 1
            else:
                break
        if boundary_fails <= 0:
            raise BoundaryError('Could not place within boundary.'
                                'Perhaps the object is too big for it?')
        elif collision_fails <= 0:
            raise BoundaryError(
                'Could not place the object within the boundary due to '
                'collision with other objects in the boundary.')

    def clear(self) -> None:
        [b.clear() for b in self._boundaries]
