#========================================================================||====#
import omni 

from omni.isaac.core.utils.rotations import euler_angles_to_quat 
from omni.isaac.core.utils.rotations import quat_to_euler_angles 
from omni.isaac.core.utils.rotations import gf_quat_to_np_array

import numpy as np 


#========================================================================||====#
def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0])



#========================================================================||====#
class eulerRotation:
    """
         Rotates a quaternion around an angle of the given Euler angles
    """
    @staticmethod
    def compute(db) -> bool :
        try:
            euler = np.deg2rad( db.inputs.euler )

            q1 = euler_angles_to_quat( euler ) 
            q2 = quaternion_multiply(q1, db.inputs.quaternion)

            q2mod = np.linalg.norm(q2)  
            if q2mod > 0.0 :
                db.outputs.rotated = q2 / q2mod 

        except Exception as error:
            db.log_error(str(error))
            return False

        return True
