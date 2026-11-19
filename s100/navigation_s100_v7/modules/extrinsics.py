import math,json
import numpy as np
from common import ROOT

def transforms():
    calibration=json.loads((ROOT/'config/lidar_calibration.json').read_text());result={}
    for sensor,prefix in [('front','left'),('back','right')]:
        r,p,y=np.deg2rad(calibration[prefix+'_to_base_R']);cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        matrix=np.eye(4);matrix[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];matrix[:3,3]=calibration[prefix+'_to_base_T'];result[sensor]=matrix
    return result
