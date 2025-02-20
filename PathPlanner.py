import numpy as np

# 直線パスを表すクラス
class LinePath:
    def __init__(self, p0: tuple[float, float], p1: tuple[float, float]) -> None:
        """
        初期化。2点を指定して直線を定義。
        - p0: 始点の座標 (x, y)
        - p1: 終点の座標 (x, y)
        """
        self.p0 = np.array(p0)  # 始点
        self.p1 = np.array(p1)  # 終点
        self.length = np.linalg.norm(self.p1 - self.p0)  # 線分の長さ
        self.vec = (self.p1 - self.p0) / self.length  # 単位方向ベクトル

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t に基づき位置を計算。
        - t: 始点からの距離
        - 戻り値: (x, y) 座標または None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            return tuple(self.p0 + t * self.vec)
        else:
            return None
        
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t における速度ベクトル（方向）を計算。
        - 戻り値: 単位ベクトルまたは None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            return tuple(self.vec)
        else:
            return None
        
    def t_to_a(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t における加速度を計算（直線なので常に 0）。
        - 戻り値: (0, 0) または None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            return (0, 0)
        else:
            return None

    def prep(self, p: tuple[float, float]) -> float | None:
        """
        座標 p から直線上の最近接点までのパラメータ t を計算。
        - p: 座標 (x, y)
        - 戻り値: 最近接点のパラメータ t または None（範囲外の場合）
        """
        _p = np.array(p)
        t = np.dot(_p - self.p0, self.vec)
        if 0 <= t <= self.length:
            return t
        else:
            return None


# 円弧パスを表すクラス
class CirclePath:
    def __init__(self, p0: tuple[float, float], theta0: float, theta1: float, r: float) -> None:
        """
        初期化。中心点、開始角度、終了角度、半径を指定して円弧を定義。
        - p0: 円弧上の開始点
        - theta0: 開始角度（ラジアン）
        - theta1: 終了角度（ラジアン）
        - r: 半径（正負で方向指定）
        """
        self.theta0 = theta0 % (2 * np.pi)  # 正規化された開始角度
        theta = (theta1 - theta0 + np.pi) % (2 * np.pi) - np.pi  # 角度差（-π～π）
        self.r = r if theta > 0 else -r  # 半径（方向に応じて符号を調整）
        self.length = abs(self.r * theta)  # 円弧の長さ
        # 円の中心座標を計算
        self.o = np.array(p0) - self.r * np.array([np.sin(self.theta0), -np.cos(self.theta0)])

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t に基づき位置を計算。
        - t: 円弧上の長さ
        - 戻り値: (x, y) 座標または None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            theta = t / self.r
            return tuple(self.o + self.r * np.array([np.sin(self.theta0 + theta), -np.cos(self.theta0 + theta)]))
        else:
            return None
        
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t における速度ベクトル（方向）を計算。
        - 戻り値: 単位ベクトルまたは None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            theta = t / self.r
            return (np.cos(self.theta0 + theta), np.sin(self.theta0 + theta))
        else:
            return None

    def t_to_a(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t における加速度を計算（円の曲率に基づく）。
        - 戻り値: 加速度ベクトルまたは None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            theta = t / self.r
            return (- np.sin(self.theta0 + theta) / self.r, np.cos(self.theta0 + theta) / self.r)
        else:
            return None

    def prep(self, p: tuple[float, float]) -> float | None:
        """
        座標 p から円弧上の最近接点までのパラメータ t を計算。
        - p: 座標 (x, y)
        - 戻り値: 最近接点のパラメータ t または None（範囲外の場合）
        """
        _p = np.array(p)
        theta = (np.arctan2(_p[1] - self.o[1], _p[0] - self.o[0]) + np.pi / 2 - self.theta0 + np.pi) % (2 * np.pi) - np.pi
        t = theta * self.r
        if 0 <= t <= self.length:
            return t
        else:
            theta = (theta + np.pi + np.pi) % (2 * np.pi) - np.pi
            t = theta * self.r
            if 0 <= t <= self.length:
                return t
            else:
                return None


# 定点を表すクラス
class PointPath:
    def __init__(self, p: tuple[float, float], v: tuple[float, float]) -> None:
        """
        初期化。定点と方向を指定。
        - p: 座標 (x, y)
        - v: 方向ベクトル
        """
        self.p = np.array(p)  # 点の座標
        self.length = 0  # 定点なので長さは常に 0
        self.v = np.array(v) / np.linalg.norm(v)  # 単位方向ベクトル

    def t_to_p(self, t: float) -> tuple[float, float] | None:
        """
        パラメータ t に基づき位置を返却。
        - 戻り値: 点の座標または None（範囲外の場合）
        """
        if 0 <= t <= self.length:
            return tuple(self.p)
        else:
            return None

    def t_to_a(self, t: float) -> tuple[float, float] | None:
        """
        加速度を計算（常に 0）。
        - 戻り値: (0, 0)
        """
        if 0 <= t <= self.length:
            return (0, 0)
        else:
            return None
                
    def t_to_v(self, t: float) -> tuple[float, float] | None:
        """
        方向ベクトルを返却。
        - 戻り値: 方向ベクトル
        """
        if 0 <= t <= self.length:
            return tuple(self.v)
        else:
            return None
        
    def prep(self, p: tuple[float, float]) -> float | None:
        """
        定点なので t は常に 0 を返却。
        """
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
            self.total_t += self.path[-2].length + self.path[-1].length

            p0 = self.points[i + 1] + r * abs_tan_05theta * vec1

        p1 = self.points[-1]
        self.path = self.path + (LinePath(p0, p1),)
        self.total_t += self.path[-1].length

        self.path = self.path + (PointPath(p1, p1 - p0),)

        self.v_forward = np.array((0,0))
        self.v_lateral = np.array((0,0))

    def calcVelocity(self, point: tuple[float, float]) -> tuple[float, float]:
        min_dist = np.inf
        v_forward = np.array((0, 0))
        total_length = 0
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
                            from_start = total_length + t
                            from_end = self.total_t - from_start

            total_length += path.length

        v_lateral = v_lateral * min((self.vmax, np.sqrt(0.5 * self.amax * from_prep)))
        v_forward = v_forward * min((np.sqrt(max((self.vmax**2 - np.linalg.norm(v_lateral)**2, 0))), self.vmax, np.sqrt(2 * 0.5*self.amax * from_start), np.sqrt(2 * 0.5*self.amax * from_end)))

        self.v_forward = v_forward
        self.v_lateral = v_lateral

        return tuple(v_forward + v_lateral)

class Robot:
    def __init__(self, points: tuple[tuple[float, float], ...], vmax, amax) -> None:
        self.path = Path(points, vmax, amax)
        self.vmaxes = np.array(((self.path.total_t,0),))