from geometry_msgs.msg import Quaternion, Vector3


def encode_vec3(ls: tuple[float, float, float]) -> Vector3:
    v = Vector3()
    v.x = ls[0]
    v.y = ls[1]
    v.z = ls[2]
    return v


# [w, x, y, z]
def encode_quat(ls: tuple[float, float, float, float]) -> Quaternion:
    q = Quaternion()
    q.x = ls[1]
    q.y = ls[2]
    q.z = ls[3]
    q.w = ls[0]
    return q
