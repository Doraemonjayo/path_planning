import numpy as np

class StraightPath:
    def __init__(self, startPoint : tuple[float, float], endPoint : tuple[float, float], startVelocity : float, endVelocity : float) -> None:
        
        self.startPoint = np.array(startPoint)
        self.endPoint = np.array(endPoint)

        self.startVelocity = startVelocity
        self.endVelocity = endVelocity
        
        self.vector = self.endPoint - self.startPoint
        self.length = float(np.linalg.norm(self.vector))
        self.unitVector = self.vector / self.length

        self.duration = 2 * self.length / (startVelocity + endVelocity)
        self.acceleration = (endVelocity - startVelocity) / self.duration

    def pointToTime(self, point : tuple[float, float]) -> float | None:
        n = (np.array(point) - self.startPoint).dot(self.unitVector)
        if self.acceleration == 0:
            time = n / self.startVelocity
        else:
            if self.startVelocity**2 + 2 * self.acceleration * n >= 0:
                time = (np.sqrt(self.startVelocity**2 + 2 * self.acceleration * n) - self.startVelocity) / self.acceleration
            else:
                time = -1
        if 0 <= time <= self.duration:
            return time
        else:
            return None
    
    def timeToPoint(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.startPoint + self.unitVector * (self.startVelocity * time + 0.5 * self.acceleration * time**2))
        else:
            return None
    
    def timeToVelocity(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.unitVector * (self.startVelocity + self.acceleration * time))
        else:
            return None
    
    def timeToAcceleration(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.unitVector * self.acceleration)
        else:
            return None
        

class CornerPath:
    def __init__(self, cornerPoint : tuple[float, float], startVector : tuple[float, float], endVector : tuple[float, float], velocity : float, acceleration : float) -> None:
        self.cornerPoint = np.array(cornerPoint)
        self.startVector = np.array(startVector)
        self.endVector = np.array(endVector)
        self.velocity = abs(velocity)
        self.acceleration = abs(acceleration)

        self.startAngle = np.arctan2(self.startVector[1], self.startVector[0])
        self.endAngle = np.arctan2(self.endVector[1], self.endVector[0])
        self.deltaAngle = (self.endAngle - self.startAngle + np.pi) % (2 * np.pi) - np.pi

        self.radius = self.velocity**2 / self.acceleration
        
        if self.radius == 0:
            self.duration = 0
        else:
            self.duration = self.deltaAngle * self.radius / self.velocity
        if self.duration < 0:
            self.duration = -self.duration
            self.radius = -self.radius
            # self.deltaAngle = -self.deltaAngle
            # self.velocity = -self.velocity

        self.cutLength = self.radius * np.tan(self.deltaAngle / 2)

        self.centerPoint = self.cornerPoint + self.cutLength / np.cos(np.pi / 2 - self.deltaAngle / 2) * np.array((np.cos(self.startAngle + self.deltaAngle / 2 + np.pi / 2), np.sin(self.startAngle + self.deltaAngle / 2 + np.pi / 2)))
        # print(1 / np.cos(self.deltaAngle / 2))
        # self.centerPoint = self.cornerPoint - self.cutLength * startVector / np.linalg.norm(startVector) + self.radius * np.array((np.cos(self.startAngle + np.pi / 2), np.sin(self.startAngle + np.pi / 2)))

        self.cutLength = abs(self.cutLength)

        self.arcStartAngle = self.startAngle - np.pi / 2

    def pointToTime(self, point : tuple[float, float]) -> float | None:
        angle = np.arctan2(point[1] - self.centerPoint[1], point[0] - self.centerPoint[0]) - self.arcStartAngle
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        time = angle / self.deltaAngle * self.duration
        if 0 <= time <= self.duration:
            return time
        angle += np.pi
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        time = angle / self.deltaAngle * self.duration
        if 0 <= time <= self.duration:
            return time
        return None

    def timeToPoint(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.centerPoint + self.radius * np.array((np.cos(self.arcStartAngle + time / self.duration * self.deltaAngle), np.sin(self.arcStartAngle + time / self.duration * self.deltaAngle))))
        else:
            return None
        
    def timeToVelocity(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.velocity * np.array((-np.sin(self.arcStartAngle + time / self.duration * self.deltaAngle), np.cos(self.arcStartAngle + time / self.duration * self.deltaAngle))))
        else:
            return None
        
    def timeToAcceleration(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.acceleration * np.array((-np.cos(self.arcStartAngle + time / self.duration * self.deltaAngle), -np.sin(self.arcStartAngle + time / self.duration * self.deltaAngle))))
        else:
            return None
        
class PointPath:
    def __init__(self, point : tuple[float, float], velocity : tuple[float, float]) -> None:
        self.point = np.array(point)
        self.velocity = np.array(velocity)
        self.duration = 0

    def pointToTime(self, point : tuple[float, float]) -> float | None:
        return 0
    
    def timeToPoint(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.point)
        else:
            return None
        
    def timeToVelocity(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return tuple(self.velocity)
        else:
            return None
        
    def timeToAcceleration(self, time : float) -> tuple[float, float] | None:
        if 0 <= time <= self.duration:
            return (0, 0)
        else:
            return None
        
class Path:
    def __init__(self, pointsAndVelocities : tuple[tuple[tuple[float, float], float], ...], maxVelocity : float, maxAcceleration : float, maxLateralAcceleration : float) -> None:
        self.points = np.array([_[0] for _ in pointsAndVelocities])
        self.velocities = np.array([_[1] for _ in pointsAndVelocities])
        self.maxVelocity = maxVelocity
        self.maxAcceleration = maxAcceleration
        self.maxLateralAcceleration = maxLateralAcceleration

        self.paths : list[StraightPath | CornerPath | PointPath] = []

        vector0 = self.points[1] - self.points[0]
        vector0 = vector0 / np.linalg.norm(vector0)
        vector1 = self.points[2] - self.points[1]
        vector1 = vector1 / np.linalg.norm(vector1)

        self.paths.append(PointPath(self.points[0], self.velocities[0] * vector0))

        cutLength0 = 0
        corner = CornerPath(self.points[1], vector0, vector1, self.velocities[1], self.maxAcceleration)
        cutLength1 = corner.cutLength
        self.paths.append(StraightPath(self.points[0] + cutLength0 * vector0, self.points[1] - cutLength1 * vector0, self.velocities[0], self.velocities[1]))
        self.paths.append(corner)

        for i in range(2, self.points.shape[0] - 1):
            vector0 = vector1
            vector1 = self.points[i + 1] - self.points[i]
            vector1 = vector1 / np.linalg.norm(vector1)
            cutLength0 = cutLength1
            corner = CornerPath(self.points[i], vector0, vector1, self.velocities[i], self.maxAcceleration)
            cutLength1 = corner.cutLength
            self.paths.append(StraightPath(self.points[i - 1] + cutLength0 * vector0, self.points[i] - cutLength1 * vector0, self.velocities[i - 1], self.velocities[i]))
            self.paths.append(corner)

        vector0 = vector1
        self.paths.append(StraightPath(self.points[-2] + cutLength0 * vector0, self.points[-1], self.velocities[-2], self.velocities[-1]))
        self.paths.append(PointPath(self.points[-1], self.velocities[-1] * vector0))

        self.forwardVelocity = np.array((0, 0))
        self.lateralVelocity = np.array((0, 0))

    def calcVelocity(self, point : tuple[float, float]) -> tuple[float, float]:
        min_dist = np.inf
        forwardVelocity = np.array((0, 0))
        lateralVelocity = np.array((0, 0))
        
        for path in self.paths:
            time = path.pointToTime(point)
            if time is not None:
                p = path.timeToPoint(time)
                v = path.timeToVelocity(time)
                if point is not None and v is not None:
                    dist = np.linalg.norm(np.array(point) - np.array(p))
                    if dist < min_dist:
                        min_dist = dist
                        forwardVelocity = np.array(v)
                        lateralVelocity = np.array(p) - np.array(point)
                        lateralVelocity = (lateralVelocity / np.linalg.norm(lateralVelocity)) if np.linalg.norm(lateralVelocity) != 0 else np.array((0,0))

        lateralVelocity = lateralVelocity * min((self.maxVelocity, np.sqrt(2 * self.maxLateralAcceleration * min_dist)))
        forwardVelocitySize = np.linalg.norm(forwardVelocity)
        if forwardVelocitySize == 0:
            forwardVelocity = np.array((0, 0))
        else:
            forwardVelocity = forwardVelocity / forwardVelocitySize * min((self.maxVelocity, forwardVelocitySize, np.sqrt(max((self.maxVelocity**2 - np.linalg.norm(lateralVelocity)**2, 0)))))

        self.forwardVelocity = forwardVelocity
        self.lateralVelocity = lateralVelocity

        output = forwardVelocity + lateralVelocity

        if np.linalg.norm(output) > self.maxVelocity:
            output = output / np.linalg.norm(output) * self.maxVelocity

        return tuple(output)