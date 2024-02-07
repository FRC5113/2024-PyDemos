import math

from photonlibpy.photonCamera import PhotonCamera
from wpimath.filter import MedianFilter


class Vision:
    camera: PhotonCamera
    # size of filter window: larger values are less accurate but better at filtering
    filter_window: int

    _x = 0
    _y = 0
    _z = 0
    _id = 0
    _latency = 0

    def setup(self):
        # setup() required because variables need to be injected
        self._x_filter = MedianFilter(self.filter_window)
        self._y_filter = MedianFilter(self.filter_window)
        self._z_filter = MedianFilter(self.filter_window)

        """If no targets are found within a certain window of time, 
        this component will consider there to be no targets. 
        This is to prevent momentary lapses in detection from 
        causing the robot to jerk
        """
        self.drought = self.filter_window

    def hasTargets(self) -> bool:
        return self.drought < self.filter_window

    def getX(self) -> float:
        if self.drought < self.filter_window:
            return self._x
        return None

    def getY(self) -> float:
        if self.drought < self.filter_window:
            return self._y
        return None

    def getZ(self) -> float:
        if self.drought < self.filter_window:
            return self._z
        return None

    def getId(self) -> int:
        if self.drought < self.filter_window:
            return self._id
        return None
    
    def getLatency(self) -> float:
        if self.drought < self.filter_window:
            return self._latency
        return None

    # returns angle that robot must turn to face tag
    def getHeading(self) -> float:
        if self.drought < self.filter_window:
            return math.atan2(-self._y, self._x) * 180 / math.pi
        return None

    def execute(self):
        result = self.camera.getLatestResult()
        if result.hasTargets():
            self.drought = 0
            """replace this garbage with `result.getBestTarget()` whenever
            photonvision decides to implement it in their lovely library"""
            target = min(result.getTargets(), key=lambda t: t.getPoseAmbiguity())
            transform = target.getBestCameraToTarget()
            self._id = target.getFiducialId()
            self._latency = result.getLatencyMillis() / 1000
            self._x = self._x_filter.calculate(transform.X())
            self._y = self._y_filter.calculate(transform.Y())
            self._z = self._z_filter.calculate(transform.Z())
        else:
            self.drought += 1
