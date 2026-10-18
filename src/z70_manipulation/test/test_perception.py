import importlib.util
from pathlib import Path
import cv2
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]

def api():
    spec=importlib.util.find_spec('z70_manipulation.perception')
    assert spec is not None, 'deterministic perception implementation missing'
    from z70_manipulation import perception
    return perception

def test_coloured_blobs_reject_tiles_and_noise():
    p=api(); image=np.full((1200,1600,3),210,np.uint8)
    for x in range(0,1600,200): cv2.line(image,(x,0),(x,1199),(100,100,100),3)
    for y in range(0,1200,200): cv2.line(image,(0,y),(1599,y),(100,100,100),3)
    cv2.circle(image,(780,900),35,(0,150,240),-1)
    cv2.circle(image,(500,950),30,(220,30,220),-1)
    image[100,100]=(0,170,255)
    blobs=p.detect(image)
    assert sorted(b['color'] for b in blobs)==['purple','yellow']
    yellow=next(b for b in blobs if b['color']=='yellow')
    assert yellow['contact_px']==pytest.approx([780,934],abs=3)
    assert yellow['size_px']>=65
    # Metric scene: 4-6 cm objects, independent of the detector-only scene above.
    image=np.full((1200,1600,3),210,np.uint8)
    cv2.circle(image,(780,690),90,(0,150,240),-1)   # ~6.5 cm, the real dodecahedron size
    cv2.circle(image,(500,720),40,(220,30,220),-1)
    objects=p.front_objects(image,ROOT)
    assert len(objects)==2
    assert all(o['position'][0]>.275 and abs(o['position'][1])<1 for o in objects)

def test_ray_floor_intersection_and_horizon():
    p=api(); K=np.array([[800.,0,800],[0,800,450],[0,0,1]])
    R=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    assert p.ray_floor((800,1250),K,np.zeros(5),[.4,.02,.2],R)==pytest.approx([.6,.02,0])
    assert p.ray_floor((800,450),K,np.zeros(5),[.4,.02,.2],R) is None

def test_front_contact_roundtrip():
    p=api(); cam=p.front_camera(ROOT)
    pixel=cam.floor_to_pixel(400,50)
    image=np.full((1200,1600,3),190,np.uint8)
    u,v=map(round,pixel)
    cv2.rectangle(image,(u-50,v-80),(u+50,v),(0,170,255),-1)
    obj=p.front_objects(image,ROOT)[0]
    assert obj['position']==pytest.approx([.675,.05,0],abs=.005)

def test_front_rejects_unphysical_sized_regions_and_coloured_labels():
    p=api();image=np.full((1200,1600,3),190,np.uint8)
    # 8px width is below18mm with the shared new-run floor; the old24px
    # fixture is now29mm (a physically eligible object), not a rejection case.
    cv2.rectangle(image,(1240,643),(1247,652),(255,20,20),-1)  # too small in metres
    cv2.rectangle(image,(463,464),(735,672),(0,100,240),-1)  # too large in metres
    cv2.rectangle(image,(561,593),(590,606),(0,170,255),-1)  # squat label, not dodecahedron
    cv2.rectangle(image,(300,528),(345,656),(0,100,240),-1)  # narrow tall region
    assert p.detect(image)
    assert p.front_objects(image,ROOT)==[]

def test_yellow_facets_include_warm_shadow_hues_without_losing_contact():
    p=api();hsv=np.zeros((1200,1600,3),np.uint8);hsv[:,:,2]=190
    cv2.rectangle(hsv,(600,650),(800,850),(19,220,170),-1)
    cv2.rectangle(hsv,(680,650),(800,850),(13,240,150),-1)
    image=cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    yellow=next(b for b in p.detect(image) if b['color']=='yellow')
    assert yellow['bbox']==[600,650,201,201]
    assert yellow['contact_px'][0]==pytest.approx(700,abs=2)


def test_purple_pale_facets_are_not_replaced_by_an_offset_saturated_core():
    p = api()
    hsv = np.zeros((1200, 1600, 3), np.uint8)
    hsv[:, :, 2] = 190
    cv2.rectangle(hsv, (600, 650), (800, 850), (145, 95, 170), -1)
    cv2.rectangle(hsv, (600, 650), (710, 780), (145, 220, 170), -1)
    image = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    purple = next(b for b in p.front_objects(image, ROOT) if b['color'] == 'purple')
    assert purple['bbox'] == [600, 650, 201, 201]
    assert purple['contact_px'] == pytest.approx([700, 850], abs=2)


def test_contact_uses_bottom_pixels_not_sparse_contour_vertices():
    p=api();image=np.full((1200,1600,3),200,np.uint8)
    cv2.rectangle(image,(599,670),(789,820),(0,160,240),-1)
    jpeg=cv2.imencode('.jpg',image)[1]
    blob=p.detect(cv2.imdecode(jpeg,cv2.IMREAD_COLOR))[0]
    assert blob['contact_px'][0]==pytest.approx(693,abs=3)

