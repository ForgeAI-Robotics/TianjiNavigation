"""Camera basis and logical-pixel pan displacement for the 3D viewer."""
import numpy as np

def basis(yaw,pitch,special=None):
    if special=='top':offset=np.array([0.,0.,1.]);up=np.array([0.,1.,0.])
    elif special=='side':offset=np.array([0.,-1.,0.]);up=np.array([0.,0.,1.])
    elif special=='front':offset=np.array([1.,0.,0.]);up=np.array([0.,0.,1.])
    else:offset=np.array([np.cos(yaw)*np.cos(pitch),np.sin(yaw)*np.cos(pitch),np.sin(pitch)]);up=np.array([0.,0.,1.])
    forward=-offset;right=np.cross(forward,up);right/=np.linalg.norm(right);vertical=np.cross(right,forward)
    return offset,up,right,vertical

def pan_delta(dx,dy,height,distance,yaw,pitch,special=None):
    _,_,right,vertical=basis(yaw,pitch,special)
    scale=2*distance*np.tan(np.deg2rad(21))/max(1,height)
    return (-dx*right+dy*vertical)*scale
