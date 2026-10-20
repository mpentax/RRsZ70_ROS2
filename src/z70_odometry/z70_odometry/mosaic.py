"""Metric floor mosaic in the odom frame (rows +y, columns +x)."""
import math
import cv2
import numpy as np
from .resources import initial_camera_x_m


class FloorMosaic:
    def __init__(self, cam, resolution=.005, size_m=4., max_range=1.2, camera_x_m=None):
        if camera_x_m is None: camera_x_m=initial_camera_x_m()
        if resolution <= 0 or size_m <= 0 or max_range <= .06:
            raise ValueError('positive map size/resolution and range > 0.06 required')
        self.cam, self.resolution, self.max_range, self.cam_x = cam, resolution, max_range, camera_x_m
        self.n = int(math.ceil(size_m / resolution))
        self.origin = (-self.n * resolution / 2, -self.n * resolution / 2)
        self.mean = np.zeros((self.n, self.n, 3), np.float32)
        self.count = np.zeros((self.n, self.n), np.uint32)
        yy, xx = np.meshgrid(np.arange(self.n), np.arange(self.n), indexing='ij')
        self.world = np.stack([self.origin[0]+(xx.ravel()+.5)*resolution,
                               self.origin[1]+(yy.ravel()+.5)*resolution],1)

    def update(self, image, pose):
        if image.shape != (self.cam.H,self.cam.W,3):
            raise ValueError('image dimensions must match camera calibration')
        x,y,th = pose
        c,s = math.cos(th), math.sin(th)
        R=np.array([[c,-s],[s,c]])
        camera_origin=np.array([x,y])+R@np.array([self.cam_x,0])
        local=(self.world-camera_origin)@R
        r=np.linalg.norm(local,axis=1)
        candidate=np.flatnonzero((r>.06)&(r<self.max_range)&(local[:,0]>0))
        uv,ok=self.cam.floor_to_pixels(local[candidate])
        ok &= (uv[:,0]>=1)&(uv[:,0]<self.cam.W-2)&(uv[:,1]>=1)&(uv[:,1]<self.cam.H-2)
        ids=candidate[ok]; uv=uv[ok].astype(np.float32)
        sample=np.empty((len(ids),3),np.float32)
        for start in range(0,len(ids),30000):
            part=uv[start:start+30000]
            sample[start:start+len(part)]=cv2.remap(
                image,part[:,0].reshape(-1,1),part[:,1].reshape(-1,1),cv2.INTER_LINEAR).reshape(-1,3)
        supported=np.any(sample!=0,axis=1)
        ids,sample=ids[supported],sample[supported]
        count=self.count.ravel(); mean=self.mean.reshape(-1,3)
        overlap=count[ids]>0
        ncc=None
        if overlap.sum()>20:
            a=mean[ids[overlap]].mean(1); b=sample[overlap].mean(1)
            a=a-a.mean(); b=b-b.mean(); denom=np.linalg.norm(a)*np.linalg.norm(b)
            if denom>1e-6: ncc=float(a@b/denom)
        mean[ids]+=(sample-mean[ids])/(count[ids,None]+1)
        count[ids]+=1
        return {'observed_cells':int(len(ids)), 'overlap_cells':int(overlap.sum()),
                'overlap_ncc':ncc,'total_cells':int(np.count_nonzero(count))}

    def image(self):
        return np.clip(np.rint(self.mean),0,255).astype(np.uint8)
