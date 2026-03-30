import math
import tf_transformations as tf_trans

from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import TransformStamped

def transform_to_matrix(transform):
    """
    Convert a TransformStamped to a 4x4 matrix.
    """
    translation = [
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ]
    rotation = [
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ]
    return tf_trans.translation_matrix(translation) @ tf_trans.quaternion_matrix(
        rotation
    )


def compose_transforms(t1, t2, time):
    """
    Compose two TransformStamped messages: t1 * t2.
    Returns a new TransformStamped.
    """
    # Convert TransformStamped to 4x4 matrices
    m1 = transform_to_matrix(t1)
    m2 = transform_to_matrix(t2)

    # Multiply the matrices
    m_composed = tf_trans.concatenate_matrices(m1, m2)

    # Convert back to TransformStamped
    composed_transform = TransformStamped()
    composed_transform.header.stamp = time
    composed_transform.header.frame_id = t1.header.frame_id
    composed_transform.child_frame_id = t2.child_frame_id

    # Extract translation and rotation from the composed matrix
    translation = tf_trans.translation_from_matrix(m_composed)
    quaternion = tf_trans.quaternion_from_matrix(m_composed)

    composed_transform.transform.translation.x = translation[0]
    composed_transform.transform.translation.y = translation[1]
    composed_transform.transform.translation.z = translation[2]
    composed_transform.transform.rotation.x = quaternion[0]
    composed_transform.transform.rotation.y = quaternion[1]
    composed_transform.transform.rotation.z = quaternion[2]
    composed_transform.transform.rotation.w = quaternion[3]

    return composed_transform


def yaw_to_quat(yaw):
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q

def interpolate_segment(p0, p1, ds):
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0
    L = math.hypot(dx, dy)
    if L < 1e-6:
        return [p0]
    n = max(1, int(math.floor(L / ds)))
    pts = [(x0 + i * dx / n, y0 + i * dy / n) for i in range(1, n)]
    pts.append((x1, y1))
    return pts


def densify_path(p0, p1, header, spacing=0.1):
    pts = interpolate_segment(p0, p1, spacing)
    poses = []
    for i in range(len(pts)-1):
        px, py = pts[i]
        x, y = pts[i + 1]
        yaw = math.atan2(y - py, x - px)

        if (i == len(pts) - 1) and (len(p1) == 3):
            # last pose: use provided yaw of final waypoint if available, else keep previous
            yaw = p1[2]

        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = yaw_to_quat(yaw)
        poses.append(ps)

    return poses
