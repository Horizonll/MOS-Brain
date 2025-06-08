# angle.py
#   @description:  the class for angle operations
#

class angle:
    def __init__(self, _angle : float):
        self.angle = _angle
    def __float__(self):
        return self.angle
    def __lt__(self, ano: "angle"):
        return self.angle < ano.angle
    def __gt__(self, ano: "angle"):
        return self.angle > ano.angle

    @staticmethod
    def normalize_angle(angle : float) -> float:
        if(angle > 180):
            return angle - 360
        elif(angle < -180):
            return angle + 360
        else:
            return angle

    # operator +
    #   a + b means rotate a by angle b anti-clockwise
    @classmethod
    def __add__(self, ano: "angle"):
        return normalize_angle(self.angle + ano.angle)

    # operator -
    #   a - b returns the degree you have to turn from 
    #   b to a anti-clockwise
    @classmethod
    def __sub__(self, ano : "angle"):
        return normalize_angle(self.angle - ano.angle)

    # operation abs()
    #   It is useful in computing the distance between
    #   a and b (both are angle type):
    #       angle diff = abs(a - b)
    @classmethod
    def __abs__(self):
        return abs(self.angle)
