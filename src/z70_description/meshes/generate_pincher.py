#!/usr/bin/env python3
"""Deterministic offline metre-scale meshes and collisions from shared anatomy."""
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from z70_description.arm_model import N,THETA,shell_rings,finger_rings,rest_bodies


def loft(rings):
    faces=[]
    for a,b in zip(rings,rings[1:]):
        for i in range(N):
            j=(i+1)%N
            faces.extend([[a[i],a[j],b[j]],[a[i],b[j],b[i]]])
    for ring,reverse in [(rings[0],True),(rings[-1],False)]:
        center=ring.mean(axis=0)
        for i in range(N):
            j=(i+1)%N
            faces.append([center,ring[j],ring[i]] if reverse else [center,ring[i],ring[j]])
    return np.array(faces)


def shell(): return loft(shell_rings())
def finger(sign): return loft(finger_rings(sign))


def collar():
    # Low rounded boss, local +Z toward the shell, -Z proud of the underside.
    profile=[(.006,.008),(.002,.0084),(0,.009),(-.0014,.0082),(-.0028,.0062),(-.004,.0042),(-.0048,.003)]
    rings=[np.column_stack([radius*np.cos(THETA),radius*np.sin(THETA),np.full(N,z)]) for z,radius in profile]
    return loft(rings)


def lens():
    return loft([np.column_stack([np.full(N,x),radius*np.cos(THETA),radius*np.sin(THETA)])
                 for x,radius in [(-.0014,.0026),(-.00025,.0028)]])


def write_collision():
    robot=ET.Element('robot',{'xmlns:xacro':'http://www.ros.org/wiki/xacro'})
    for name in ('arm_wrist_link','finger_left','finger_right'):
        macro=ET.SubElement(robot,'xacro:macro',{'name':name+'_collisions'})
        for body in rest_bodies():
            if body.name!=name: continue
            collision=ET.SubElement(macro,'collision')
            ET.SubElement(collision,'origin',{'xyz':' '.join(format(v,'.12g') for v in body.center)})
            geometry=ET.SubElement(collision,'geometry')
            ET.SubElement(geometry,'box',{'size':' '.join(format(v,'.12g') for v in body.half*2)})
    ET.indent(robot)
    (HERE.parent/'urdf/pincher_collisions.xacro').write_text(ET.tostring(robot,encoding='unicode')+'\n')


def write_stl(name,points):
    if np.einsum('ij,ij->',points[:,0],np.cross(points[:,1],points[:,2]))/6<0:
        points=points[:,[0,2,1]]
    with (HERE/name).open('wb') as file:
        file.write(b'Z70 provisional shared pincher geometry; metres'.ljust(80,b'\0'))
        file.write(struct.pack('<I',len(points)))
        for face in points:
            normal=np.cross(face[1]-face[0],face[2]-face[0]);normal/=np.linalg.norm(normal)
            file.write(struct.pack('<12fH',*normal,*face.ravel(),0))
    print(name,len(points),'triangles')


if __name__=='__main__':
    write_collision()
    write_stl('pincher_shell.stl',shell())
    write_stl('pincher_lens.stl',lens())
    write_stl('pincher_collar.stl',collar())
    write_stl('pincher_finger_left.stl',finger(1))
    write_stl('pincher_finger_right.stl',finger(-1))
