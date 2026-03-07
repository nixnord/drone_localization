import numpy

class KalmanFilterConstantVelocity:
    def __init__(
            self,
            initial_x,
            initial_y,
            initial_vx,
            initial_vy,
            x_err,
            y_err,
            vx_err,
            vy_err
    ):
        # assign the state vector
        self._X = numpy.array([
            [initial_x],
            [initial_y],
            [initial_vx],
            [initial_vy]
        ])
        # assign the process covariance matrix
        self._P = numpy.array([ 
            [ x_err**2, 0, x_err*vx_err, 0 ],
            [ 0, y_err**2, 0, y_err*vy_err ],
            [ x_err*vx_err, 0, vx_err**2, 0 ],
            [ 0, y_err*vy_err, 0, vy_err**2 ]
        ])
        self._Xp = None
        self._Pp = None

    def predict_state(self, dt: float) -> None:

        F = numpy.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # some consistent noise to not let the model trust
        # wholly on the estimates
        Q = numpy.array([
            [0.0001, 0, 0, 0],
            [0, 0.0001, 0, 0],
            [0, 0, 0.01, 0],
            [0, 0, 0, 0.01]
        ])

        self._Xp = F.dot(self._X)
        self._Pp = F.dot(self._P).dot(F.T) + Q
    
    def update_state(self, measurement, R):
        H = numpy.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        Y = measurement - H.dot(self._Xp)
        S = H.dot(self._Pp).dot(H.T) + R
        kalman_gain = self._Pp.dot(H.T).dot(numpy.linalg.inv(S))
        self._X = self._Xp + kalman_gain.dot(Y)
        self._P = (numpy.eye(4) - kalman_gain.dot(H)).dot(self._Pp)

    @property
    def state_vector(self):
        return self._X
    @property
    def process_covariance(self):
        return self._P
    @property
    def x_pred(self):
        return self._Xp
    @property
    def p_pred(self):
        return self._Pp


class KalmanFilterConstantAcceleration:

    def __init__(
        self, initial_x, initial_y,
        initial_vx, initial_vy, initial_ax,
        initial_ay, x_err, y_err,
        vx_err, vy_err,
        ax_err, ay_err
    ):
        self._X = numpy.array([
            [initial_x], [initial_y], [initial_vx],
            [initial_vy], [initial_ax], [initial_ay]
        ])

        # process covariance matrix
        self._P = numpy.diag([
            x_err**2, y_err**2, vx_err**2,
            vy_err**2, ax_err**2, ay_err**2
        ]) # basically a diagonal matrix for unrelated elements


        self._Xp = None
        self._Pp = None

        self.H_LoRa = numpy.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]
        ])

        self.H_Imu = numpy.array([
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

    def predict_state(self, dt: float, sigma_x, sigma_y):

        F = numpy.array([
            [1, 0, dt, 0, (1/2)*(dt**2), 0],
            [0, 1, 0, dt, 0, (1/2)*(dt**2)],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        # variance
        sx = sigma_x**2
        sy = sigma_y**2 
        # jerk in the acceleration in the x and y direction is defined by the sigma values
        Q = numpy.zeros((6,6))
        def q_block(variance):
            return variance * numpy.array([
                [dt**5/20, dt**4/8, dt**3/6],
                [dt**4/8, dt**3/3, dt**2/2],
                [dt**3/6, dt**2/2, dt]
            ])
        
        Qx = q_block(sx)
        Qy = q_block(sy)

        Q[numpy.ix_([0, 2, 4], [0, 2, 4])] = Qx
        Q[numpy.ix_([1, 3, 5], [1, 3, 5])] = Qy

        self._Xp = F.dot(self._X)
        self._Pp = F.dot(self._P).dot(F.T) + Q

    def _update_state(self, measurement, R, H):
        # internal helper function to update state vector
        Z = numpy.array(measurement).reshape(len(measurement), 1)
        Y = Z - H.dot(self._Xp)
        S = H.dot(self._Pp).dot(H.T) + R
        kalman_gain = self._Pp.dot(H.T).dot(numpy.linalg.inv(S))
        self._X = self._Xp + kalman_gain.dot(Y)
        self._P = (numpy.eye(6) - kalman_gain.dot(H)).dot(self._Pp)
    # Since LoRa and IMU are highly asynchronous
    # (1-10 Hz vs 100-1000 Hz respectivly) we perform
    # the measurement inputs seperately
    def update_state_position(self, measurement: list, R):
        # this is for the LoRa measurement input where 
        # we will get x and y. Add it to filter seperately
        self._update_state(measurement, R, self.H_LoRa)

    def update_state_acceleration(self, measurement: list, R):
        # this is for the IMU measurement input where 
        # we will get ax and ay. Add it to filter seperately
        self._update_state(measurement, R, self.H_Imu)

    @property
    def state_vector(self):
        return self._X
