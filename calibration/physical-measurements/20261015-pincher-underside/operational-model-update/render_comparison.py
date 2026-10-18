"""Offline mesh comparison from actual expanded URDF and retained triangles.
Run from ros2 through pixi with install/setup.bash sourced. No robot/preview.
"""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import math
import cv2
import numpy as np
import xacro

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PKG=ROOT/'ros2/src/z70_description'
sys.path.insert(0,str(PKG/'test'))
from test_physical_visuals import relative_frames,origin
from test_pincher_mesh import triangles


def box(size):
    v=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*np.array(size)/2
    indices=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
             [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]]
    return v[indices]


def cylinder(radius,length):
    theta=np.arange(64)*2*np.pi/64
    a=np.column_stack([radius*np.cos(theta),radius*np.sin(theta),np.full(64,-length/2)])
    b=a.copy();b[:,2]=length/2
    return np.array([f for i in range(64) for f in ([a[i],a[(i+1)%64],b[(i+1)%64]],
        [a[i],b[(i+1)%64],b[i]],[[0,0,-length/2],a[(i+1)%64],a[i]],[[0,0,length/2],b[i],b[(i+1)%64]])])


def geometry(before,m5,m6,full_state=None):
    source=HERE/'before/z70.urdf.xacro' if before else PKG/'urdf/z70.urdf.xacro'
    robot=ET.fromstring(xacro.process_file(str(source)).toxml())
    angles={'m5_joint':-math.radians(m5),'m6_left_joint':math.radians(m6)/2,'m6_right_joint':math.radians(m6)/2}
    if full_state:
        angles.update(m3_joint=math.radians(full_state[0]-270),m4_joint=math.radians(180-full_state[1]))
    frames=relative_frames(robot,'base_footprint' if full_state else 'arm_link2',angles)
    result=[]
    names=[link.get('name') for link in robot.findall('link')] if full_state else ('arm_link2','arm_wrist_link','finger_left','finger_right','pincher_marker_68','gripper_camera_link')
    for name in names:
        for visual in robot.find(f"link[@name='{name}']").findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is not None:
                filename=Path(mesh.get('filename')).name
                points=triangles((HERE/'before' if before else PKG/'meshes')/filename)
            elif visual.find('geometry/box') is not None:points=box(np.fromstring(visual.find('geometry/box').get('size'),sep=' '))
            else:
                c=visual.find('geometry/cylinder');points=cylinder(float(c.get('radius')),float(c.get('length')))
            T=frames[name]@origin(visual.find('origin'))
            points=points@T[:3,:3].T+T[:3,3]
            color={'arm_link2':(100,103,110),'arm_wrist_link':(77,81,90),'finger_left':(55,60,65),
                   'finger_right':(55,60,65),'pincher_marker_68':(215,218,220),'gripper_camera_link':(24,28,32)}.get(name,(120,125,130))
            result.append((points,color))
    return result


def view(parts,kind,title,scale=2400,center=None,rotation=None,footer=None,z_buffer=False):
    # Orthographic views in the anatomical unrolled link2 frame. Not screen-up
    # interpreted from a rolled photo. Depth sorting exposes the real recess.
    R=rotation if rotation is not None else {'top':np.array([[1,0,0],[0,-1,0],[0,0,1]]),
       'underside':np.array([[1,0,0],[0,1,0],[0,0,-1]]),
       'side':np.array([[1,0,0],[0,0,-1],[0,-1,0]])}[kind]
    image=np.full((600,760,3),(238,240,243),np.uint8)
    center=np.array([.11,0]) if center is None else np.asarray(center)
    faces=[]
    for mesh,color in parts:
        projected=mesh@R.T
        for face,world in zip(projected,mesh):
            normal=np.cross(world[1]-world[0],world[2]-world[0]);normal/=np.linalg.norm(normal)
            shade=.65+.35*abs(np.dot(normal,np.array([.3,-.4,.866])))
            c=tuple(int(x*shade) for x in color)
            uv=np.round((face[:,:2]-center)*scale+[380,315]).astype(np.int32)
            faces.append((float(face[:,2].mean()),uv,c,face[:,2]))
    if z_buffer:
        depth_buffer=np.full(image.shape[:2],-np.inf)
        for _,uv,c,z in faces:
            x0,y0=np.maximum(uv.min(0),[0,0]);x1,y1=np.minimum(uv.max(0),[759,599])
            if x0>x1 or y0>y1:continue
            a,b,d=uv.astype(float);den=(b[1]-d[1])*(a[0]-d[0])+(d[0]-b[0])*(a[1]-d[1])
            if abs(den)<1e-9:continue
            X,Y=np.meshgrid(np.arange(x0,x1+1),np.arange(y0,y1+1))
            u=((b[1]-d[1])*(X-d[0])+(d[0]-b[0])*(Y-d[1]))/den
            v=((d[1]-a[1])*(X-d[0])+(a[0]-d[0])*(Y-d[1]))/den;w=1-u-v
            depth=u*z[0]+v*z[1]+w*z[2]
            patch=depth_buffer[y0:y1+1,x0:x1+1];mask=(u>=0)&(v>=0)&(w>=0)&(depth>patch)
            patch[mask]=depth[mask];image[y0:y1+1,x0:x1+1][mask]=c
    else:
        for _,uv,c,_ in sorted(faces,key=lambda f:f[0]):cv2.fillConvexPoly(image,uv,c,lineType=cv2.LINE_AA)
    cv2.putText(image,title,(24,35),cv2.FONT_HERSHEY_SIMPLEX,.65,(35,35,35),2,cv2.LINE_AA)
    cv2.putText(image,footer or kind+' | wrist cover INCLUDED | metres',(24,570),cv2.FONT_HERSHEY_SIMPLEX,.48,(50,50,50),1,cv2.LINE_AA)
    bar_m=.01 if scale>10000 else .05
    cv2.line(image,(25,530),(25+round(bar_m*scale),530),(30,30,30),2)
    cv2.putText(image,f'{bar_m*1000:.0f} mm nominal',(25,520),cv2.FONT_HERSHEY_SIMPLEX,.45,(40,40,40),1,cv2.LINE_AA)
    return image


for m6,label in ((90,'open'),(0,'closed'),(45,'half-open')):
    rows=[]
    for before in (True,False):
        parts=geometry(before,0,m6)
        rows.append(np.hstack([view(parts,k,('BEFORE' if before else 'AFTER')+f' | M5=0 M6={m6} | {label}') for k in ('top','underside','side')]))
    cv2.imwrite(str(HERE/f'before-after-{label}.png'),np.vstack(rows))
cv2.imwrite(str(HERE/'roll-comparison.png'),np.vstack([
    np.hstack([view(geometry(False,m5,90),k,f'AFTER | M5={m5} M6=90') for k in ('top','underside','side')]) for m5 in (0,40,90)]))

# Default small optical recess viewed along its forward ray and from below.
# White underside rectangle in wider renders is ID68 PAPER, not another camera.
from z70_description import pincher_geometry as g
right=np.array([0.,1.,0.]);depth=np.array([1.,0.,-.30]);depth/=np.linalg.norm(depth)
down=np.cross(depth,right)*-1
R=np.array([right,down,depth]);C=(g.CAM_C+[.205,0,0])@R.T
close=view(geometry(False,0,90),'camera recess','DEFAULT M5=0 | under-plate recess CLOSE-UP',
    scale=18000,center=C[:2],rotation=R,footer='2.8mm nominal lens; rim masked; optical-axis check only',z_buffer=True)
cv2.putText(close,'Lens at centre; white ID68 backing is NOT camera',(24,65),cv2.FONT_HERSHEY_SIMPLEX,.48,(30,30,30),1,cv2.LINE_AA)
cv2.imwrite(str(HERE/'camera-cavity-close-up.png'),close)

full=[]
for state in ((200,95),(250,105)):
    parts=geometry(False,0,90,full_state=state)
    for kind,R in [('top',np.array([[1,0,0],[0,-1,0],[0,0,1]])),('side',np.array([[1,0,0],[0,0,-1],[0,-1,0]]))]:
        pts=np.concatenate([mesh.reshape(-1,3) for mesh,_ in parts])@R.T
        lo,hi=pts[:,:2].min(0),pts[:,:2].max(0)
        scale=min(660/(hi[0]-lo[0]),400/(hi[1]-lo[1]))
        full.append(view(parts,kind,f'FULL MODEL | {state} M5=0 M6=90',scale=scale,center=(lo+hi)/2,
            footer=kind+' | offline URDF; provisional body/pose, NOT hardware'))
cv2.imwrite(str(HERE/'full-model-context.png'),np.vstack([np.hstack(full[:2]),np.hstack(full[2:])]))

# Paired owner evidence, labelled without inventing metric photo landmarks.
photos=[]
for filename,title,notes in [('IMG_0647_4ec758.png','UNDERSIDE - owner rolled the assembly',
    ['Opposed lateral root hardware','ID68 backing is local, not shell up','Camera cavity below broad upper plate']),
    ('IMG_0648_87aeb8.png','DEFAULT TOP - broad wrist-aligned face',
    ['Wide flat plate continues wrist axis','Fingers emerge from opposing sides','No upright middle plate or camera pod'])]:
    im=cv2.imread(str(HERE/('decoded-'+filename)));h,w=im.shape[:2]
    scale=min(1000/w,650/h);im=cv2.resize(im,(round(w*scale),round(h*scale)))
    canvas=np.full((900,1000,3),245,np.uint8)
    left=(1000-im.shape[1])//2;canvas[70:70+im.shape[0],left:left+im.shape[1]]=im
    cv2.putText(canvas,title,(20,35),cv2.FONT_HERSHEY_SIMPLEX,.65,(20,20,20),2,cv2.LINE_AA)
    for i,line in enumerate(notes):cv2.putText(canvas,line,(20,790+i*30),cv2.FONT_HERSHEY_SIMPLEX,.6,(20,20,20),1,cv2.LINE_AA)
    photos.append(canvas)
cv2.imwrite(str(HERE/'paired-owner-evidence.png'),np.hstack(photos))
model=cv2.imread(str(HERE/'before-after-open.png'))[600:]
model=cv2.resize(model,(2000,round(model.shape[0]*2000/model.shape[1])))
cv2.imwrite(str(HERE/'photo-model-final-review.png'),np.vstack([np.hstack(photos),model]))
print('Rendered',HERE)
