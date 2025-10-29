
from geometry_msgs.msg import Pose

def square_to_pose(square: str,) -> Pose:
    chsb_mid = [0.3, 0.0]
    sqr_dim = [0.037, 0.037]

    pose = Pose()
    row = int(square[1]) - 1
    col = ord(square[0]) - 97

    a1_cords = chsb_mid[:]
    a1_cords[0] -= sqr_dim[0] * 3.5
    a1_cords[1] -= sqr_dim[1] * 3.5

    pose.position.x = a1_cords[0] + col * sqr_dim[0]
    pose.position.y = a1_cords[1] + row * sqr_dim[1]
    pose.position.z = 0.2
    pose.orientation.x = 1.0
    print(row, col)
    return pose

if __name__ == "__main__":
    p = square_to_pose("c3")
    print(p)
