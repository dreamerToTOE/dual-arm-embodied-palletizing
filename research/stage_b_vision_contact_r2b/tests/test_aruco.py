import sys,cv2,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from visual.aruco_pose import detect_aruco

def test_pixel_aruco_detection():
 d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50); img=np.full((480,640),255,np.uint8); marker=np.zeros((160,160),np.uint8); cv2.aruco.drawMarker(d,23,160,marker,1); img[160:320,240:400]=marker
 K=np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
 r=detect_aruco(cv2.cvtColor(img,cv2.COLOR_GRAY2BGR),K,np.zeros(5),.06,23)
 assert r.status=='PRESENT' and r.marker_id==23 and r.tvec_m is not None and r.reprojection_error_px<1.
