import numpy as np

class LinePath:
    def __init__(self, p0: tuple[float, float], p1: tuple[float, float]) -> None:
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)

        # ベクトルの長さ (t1)
        self.t1 = np.linalg.norm(self.p1 - self.p0)

        # 単位ベクトル (方向ベクトル)
        self.vec = (self.p1 - self.p0) / self.t1

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら座標を返す
        if 0 <= t <= self.t1:
            return tuple(self.p0 + t * self.vec)
        else:
            return None
        
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら方向ベクトルを返す
        if 0 <= t <= self.t1:
            return tuple(self.vec)
        else:
            return None
        
    def t_to_a(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら加速度ベクトルを返す
        if 0 <= t <= self.t1:
            return (0, 0)
        else:
            return None

    def prep(self, p: tuple[float, float]) -> float | None:
        # pから直線のベクトルに垂直な位置を計算
        _p = np.array(p)
        t = np.dot(_p - self.p0, self.vec)  # ベクトルの内積
        if 0 <= t <= self.t1:
            return t
        else:
            return None

        
class CirclePath:
    def __init__(self, p0: tuple[float, float], theta0: float, theta1: float, r: float) -> None:
        self.theta0 = theta0 % (2 * np.pi)
        theta = (theta1 - theta0 + np.pi) % (2 * np.pi) - np.pi
        self.r = r if theta > 0 else -r
        self.t1 = abs(self.r * theta)
        self.o = np.array(p0) - self.r * np.array([np.sin(self.theta0), -np.cos(self.theta0)])

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら座標を返す
        if 0 <= t <= self.t1:
            theta = t / self.r
            return tuple(self.o + self.r * np.array([np.sin(self.theta0 + theta), -np.cos(self.theta0 + theta)]))
        else:
            return None
        
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら方向ベクトルを返す
        if 0 <= t <= self.t1:
            theta = t / self.r
            return (np.cos(self.theta0 + theta), np.sin(self.theta0 + theta))
        else:
            return None

    def t_to_a(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら加速度ベクトルを返す
        if 0 <= t <= self.t1:
            theta = t / self.r
            return (- np.sin(self.theta0 + theta) / self.r, np.cos(self.theta0 + theta) / self.r)
        else:
            return None

    def prep(self, p: tuple[float, float]) -> float | None:
        # pから直線のベクトルに垂直な位置を計算
        _p = np.array(p)
        theta = (np.arctan2(_p[1] - self.o[1], _p[0] - self.o[0]) + np.pi / 2 - self.theta0 + np.pi) % (2 * np.pi) - np.pi
        t = theta * self.r
        if 0 <= t <= self.t1:
            return t
        else:
            theta = (theta + np.pi + np.pi) % (2 * np.pi) - np.pi
            t = theta * self.r
            if 0 <= t <= self.t1:
                return t
            else:
                return None
            
class PointPath:
    def __init__(self, p: tuple[float, float], v: tuple[float, float]) -> None:
        self.p = np.array(p)
        self.t1 = 0
        self.v = np.array(v) / np.linalg.norm(v)

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら座標を返す
        if 0 <= t <= self.t1:
            return tuple(self.p)
        else:
            return None

    def t_to_a(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら加速度ベクトルを返す
        if 0 <= t <= self.t1:
            return (0, 0)
        else:
            return None
                
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        # tが範囲内なら方向ベクトルを返す
        if 0 <= t <= self.t1:
            return tuple(self.v)
        else:
            return None
        
    def prep(self, p: tuple[float, float]) -> float | None:
        return 0

class Path:
    def __init__(self, points: tuple[tuple[float, float], ...], vmax, amax):
        self.points = np.array(points)
        self.vmax = vmax
        self.amax = amax

        r = vmax**2 / amax

        self.total_t = 0

        self.path = (PointPath(self.points[0], self.points[1] - self.points[0]),)

        p0 = self.points[0]

        for i in range(self.points.shape[0] - 2):
            vec0 = self.points[i + 1] - self.points[i]
            vec0 = vec0 / np.linalg.norm(vec0)
            vec1 = self.points[i + 2] - self.points[i + 1]
            vec1 = vec1 / np.linalg.norm(vec1)
            cos_theta = np.dot(vec0, vec1)
            abs_tan_05theta = np.sqrt((1 - cos_theta) / (1 + cos_theta))
            p1 = self.points[i + 1] - r * abs_tan_05theta * vec0
            
            self.path = self.path + (LinePath(p0, p1),CirclePath(p1, np.arctan2(vec0[1], vec0[0]), np.arctan2(vec1[1], vec1[0]), r),)
            self.total_t += self.path[-2].t1 + self.path[-1].t1

            p0 = self.points[i + 1] + r * abs_tan_05theta * vec1

        p1 = self.points[-1]
        self.path = self.path + (LinePath(p0, p1),)
        self.total_t += self.path[-1].t1

        self.path = self.path + (PointPath(p1, p1 - p0),)

        self.v_forward = np.array((0,0))
        self.v_lateral = np.array((0,0))

    def calcVelocity(self, point: tuple[float, float]) -> tuple[float, float]:
        min_dist = np.inf
        v_forward = np.array((0, 0))
        total_t1 = 0
        from_prep = 0
        from_start = 0
        from_end = 0
        for path in self.path:
            t = path.prep(point)
            if t is not None:
                p = path.t_to_p(t)
                v = path.t_to_v(t)
                if p is not None:
                    dist = np.linalg.norm(np.array(point) - np.array(p))
                    if dist < min_dist:
                        min_dist = dist
                        if v is not None:
                            v_forward = np.array(v)
                            v_lateral = np.array(p) - np.array(point)
                            from_prep = np.linalg.norm(v_lateral)
                            v_lateral = (v_lateral / from_prep) if from_prep != 0 else np.array((0,0))
                            from_start = total_t1 + t
                            from_end = self.total_t - from_start

            total_t1 += path.t1

        v_lateral = v_lateral * min((self.vmax, np.sqrt(0.5 * self.amax * from_prep)))
        v_forward = v_forward * min((np.sqrt(max((self.vmax**2 - np.linalg.norm(v_lateral)**2, 0))), self.vmax, np.sqrt(2 * 0.5*self.amax * from_start), np.sqrt(2 * 0.5*self.amax * from_end)))
        # v_forward = v_forward * min((np.sqrt(max((self.vmax**2 - np.linalg.norm(v_lateral)**2, 0))), self.vmax))

        self.v_forward = v_forward
        self.v_lateral = v_lateral

        return tuple(v_forward + v_lateral)

class Robot:
    def __init__(self, points: tuple[tuple[float, float], ...], vmax, amax) -> None:
        self.path = Path(points, vmax, amax)
        self.vmaxes = np.array(((self.path.total_t,0),))