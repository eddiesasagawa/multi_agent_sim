import numpy as np
from geometry_msgs import msg


class Quaternion(object):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=0.0)
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    @property
    def msg(self):
        q = msg.Quaternion()
        q.x = self.x
        q.y = self.y
        q.z = self.z
        q.w = self.w
        return q

    @msg.setter
    def msg(self, q_msg):
        self.x = q_msg.x
        self.y = q_msg.y
        self.z = q_msg.z
        self.w = q_msg.w

    def magnitude(self):
        return np.linalg.norm([self.x, self.y, self.z, self.w])

    def conjugate(self):
        '''
        returns another Quaternion with negative vector
        '''
        return Quaternion(-self.x, -self.y, -self.z, self.w)

    def normalize(self):
        '''
        in-place normalization (magnitude = 1)
        '''
        d = self.magnitude()
        self.x = self.x / d
        self.y = self.y / d
        self.z = self.z / d
        self.w = self.w / d

    def inverse(self):
        '''
        returns another Quaternion that is the inverse
        '''
        return self.conjugate() / (self.magnitude())^2

    def multiply_left(self, other_q):
        pass

    def __div__(self, other):
        if type(other) != Quaternion:
            # scalar division
            self.x /= other
            self.y /= other
            self.z /= other
            self.w /= other
        else:
            # quaternion division...
            raise NotImplementedError

    def __mul__(self, other):
        if type(other) != Quaternion:
            # scalar multiplication
            self.x *= other
            self.y *= other
            self.z *= other
            self.w *= other
        else:
            # quaternion multiplication
            raise NotImplementedError

    def __rmul__(self, other):
        if type(other) != Quaternion:
            self.__mul__(other)
        else:
            # quaternion right multiply...
            raise NotImplementedError

