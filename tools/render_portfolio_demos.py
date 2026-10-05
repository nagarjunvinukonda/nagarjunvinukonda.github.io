"""Render original scripted illustrations, not recorded policy/Gazebo rollouts.
Requires Pillow, numpy, ffmpeg. Run from the repository root.
"""
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import math, subprocess
from pathlib import Path
W,H,S=640,360,2
OUT=Path(__file__).resolve().parents[1]/'media'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'

def project(p):
 x,y,z=p
 return ((320+43*x-28*y)*S,(183+13*x+21*y-42*z)*S)
def line(d,pts,fill,width=1): d.line([project(p) for p in pts],fill=fill,width=max(1,int(width*S)),joint='curve')
def poly(d,pts,fill): d.polygon([project(p) for p in pts],fill=fill)
def ring(d,x,y,r,z,color,width=1):
 line(d,[(x+r*math.cos(a),y+r*math.sin(a),z) for a in np.linspace(0,2*math.pi,65)],color,width)
def disk(d,x,y,r,z,color):
 poly(d,[(x+r*math.cos(a),y+r*math.sin(a),z) for a in np.linspace(0,2*math.pi,49)],color)
def cylinder(d,x,y,r,z,h,color):
 for a in np.linspace(0,2*math.pi,49)[:-1]:
  b=a+2*math.pi/48
  shade=.68+.18*math.cos(a-.7)
  c=tuple(int(v*shade) for v in color)
  poly(d,[(x+r*math.cos(a),y+r*math.sin(a),z),(x+r*math.cos(b),y+r*math.sin(b),z),(x+r*math.cos(b),y+r*math.sin(b),z+h),(x+r*math.cos(a),y+r*math.sin(a),z+h)],c)
 disk(d,x,y,r,z+h,color)
def box(d,x,y,z,dx,dy,dz,color):
 pts=[(x-dx/2,y-dy/2,z),(x+dx/2,y-dy/2,z),(x+dx/2,y+dy/2,z),(x-dx/2,y+dy/2,z)]
 for i in [0,1,2,3]:
  a,b=pts[i],pts[(i+1)%4]
  poly(d,[a,b,(b[0],b[1],z+dz),(a[0],a[1],z+dz)],tuple(int(c*(.65+.07*i)) for c in color))
 poly(d,[(a,b,z+dz) for a,b,_ in pts],color)
def scene(label):
 im=Image.new('RGB',(W*S,H*S),(21,31,42)); d=ImageDraw.Draw(im)
 poly(d,[(-4.6,-4.1,0),(4.6,-4.1,0),(4.6,4.1,0),(-4.6,4.1,0)],(45,60,72))
 for x in np.arange(-4.5,4.6,.5): line(d,[(x,-4,0),(x,4,0)],(57,73,84))
 for y in np.arange(-4,4.1,.5): line(d,[(-4.5,y,0),(4.5,y,0)],(57,73,84))
 d.text((22*S,17*S),label,font=ImageFont.truetype(FONT,13*S),fill=(218,231,236))
 d.text((22*S,338*S),'SCRIPTED SIMULATION',font=ImageFont.truetype(FONT,9*S),fill=(144,163,175))
 return im,d

def obstacles(t):
 return [(2.35*math.cos(2*math.pi*t/12+.7),2.35*math.sin(2*math.pi*t/12+.7),.38),
         (2.4*math.sin(2*math.pi*t/6),.9,.42),(-1.25,2.8*math.sin(2*math.pi*t/6+1),.38)]

def navigation():
 # Smooth reference path bends outward as moving obstacles approach.
 frames=[]; trajectory=[]; clearances=[]
 for i in range(240):
  t=i/20; a=2*math.pi*t/12; obs=obstacles(t)
  r=2.45
  for ox,oy,orr in obs:
   oa=math.atan2(oy,ox); delta=math.atan2(math.sin(a-oa),math.cos(a-oa))
   r=max(r,math.hypot(ox,oy)+.85*math.exp(-(delta/.5)**2))
  x,y=r*math.cos(a),r*math.sin(a)
  clearances.append(min(math.hypot(x-ox,y-oy)-rr-.28 for ox,oy,rr in obs))
  trajectory.append((x,y,.025))
  im,d=scene('DYNAMIC OBSTACLE AVOIDANCE')
  box(d,0,-4.1,0,9.1,.13,.38,(99,116,129));box(d,-4.5,0,0,.13,8.2,.38,(99,116,129))
  box(d,0,0,0,1.2,1.1,.62,(112,132,151))
  if len(trajectory)>1: line(d,trajectory[-75:],(77,190,179),2)
  # Lidar rays stop at obstacles or the arena boundary.
  for th in np.linspace(0,2*math.pi,44,endpoint=False):
   dist=1.75
   for q in np.arange(.32,1.76,.06):
    px,py=x+q*math.cos(th),y+q*math.sin(th)
    if abs(px)>4.4 or abs(py)>3.95 or (abs(px)<.63 and abs(py)<.58) or any(math.hypot(px-ox,py-oy)<rr for ox,oy,rr in obs): dist=q;break
   line(d,[(x,y,.6),(x+dist*math.cos(th),y+dist*math.sin(th),.05)],(58,115,114),.6)
  objects=[('ob',ox,oy,rr) for ox,oy,rr in obs]+[('bot',x,y,.3)]
  for kind,ox,oy,rr in sorted(objects,key=lambda o:13*o[1]+21*o[2]):
   disk(d,ox+.1,oy+.1,rr*1.25,.015,(34,45,54))
   if kind=='ob':
    ring(d,ox,oy,rr+.13,.02,(220,149,73),1)
    cylinder(d,ox,oy,rr,0,.77,(226,155,69))
    cylinder(d,ox,oy,rr+.009,.55,.10,(245,220,166))
   else:
    for side in [-1,1]:
     wx,wy=x+side*.32*math.cos(a),y+side*.32*math.sin(a)
     cylinder(d,wx,wy,.14,.02,.28,(25,30,37))
    cylinder(d,x,y,.32,.16,.15,(220,228,229))
    for dx,dy in [(-.18,-.16),(.18,-.16),(.18,.16),(-.18,.16)]: line(d,[(x+dx,y+dy,.25),(x+dx,y+dy,.52)],(193,205,211),2)
    cylinder(d,x,y,.3,.49,.08,(225,231,233));cylinder(d,x,y,.125,.57,.16,(51,59,69))
    disk(d,x,y,.09,.735,(231,184,54))
    line(d,[(x,y,.59),(x-.24*math.sin(a),y+.24*math.cos(a),.59)],(43,180,213),4)
  frames.append(im.resize((W,H),Image.Resampling.LANCZOS))
 print('Minimum illustrated obstacle clearance:',round(min(clearances),3))
 return frames

def t_shape(x,y):
 return [(x+u,y+v,.08) for u,v in [(-.52,-.45),(.52,-.45),(.52,-.16),(.15,-.16),(.15,.58),(-.15,.58),(-.15,-.16),(-.52,-.16)]]
def policy():
 frames=[]
 for i in range(180):
  t=i/20
  # Approach, push, settle, retract; crossfade resets the next demonstration.
  u=np.clip((t-1.5)/4.5,0,1); ease=u*u*(3-2*u)
  x=-.6+2.0*ease;y=1.15-1.55*ease
  im,d=scene('POLICY EVALUATION  /  PUSH & ALIGN')
  box(d,0,0,.01,7.3,5.6,.12,(190,201,209))
  for gx in np.arange(-3.5,3.51,.5): line(d,[(gx,-2.6,.135),(gx,2.6,.135)],(174,188,198),.5)
  for gy in np.arange(-2.5,2.51,.5): line(d,[(-3.5,gy,.135),(3.5,gy,.135)],(174,188,198),.5)
  target=[(a,b,.145) for a,b,_ in t_shape(1.4,-.4)]
  poly(d,target,(131,191,153));line(d,target+[target[0]],(43,123,84),2)
  for step in np.linspace(ease,1,12):
   tx=-.6+2*step;ty=1.15-1.55*step
   disk(d,tx,ty,.035,.15,(67,142,121))
  pts=[(a,b,.27) for a,b,_ in t_shape(x,y)]
  for j in range(len(pts)):
   a,b=pts[j],pts[(j+1)%len(pts)];poly(d,[a,b,(b[0],b[1],.14),(a[0],a[1],.14)],(35,105,153))
  poly(d,pts,(58,151,206))
  # Two-link arm with inverse kinematics; tool stays behind the moving block.
  base=np.array([-2.5,-1.55,.45]); end=np.array([x-.58,y+.43,.32])
  if t<1.5: end+=np.array([-.4*(1-t/1.5),.1,.9*(1-t/1.5)])
  if t>6.5: end+=np.array([-.22,0,min((t-6.5)*.55,1)])
  delta=end-base; dist=np.linalg.norm(delta); mid=(base+end)/2
  up=np.array([0.,0.,1.]); perp=up-delta*np.dot(up,delta)/dist**2;perp/=np.linalg.norm(perp)
  elbow=mid+perp*math.sqrt(max(0,2.15**2-(dist/2)**2))
  cylinder(d,-2.5,-1.55,.44,.14,.20,(52,68,81));cylinder(d,-2.5,-1.55,.25,.34,.23,(202,214,224))
  for a,b in [(base,elbow),(elbow,end)]:
   line(d,[a,b],(74,91,108),18);line(d,[a+np.array([-.025,-.025,.04]),b+np.array([-.025,-.025,.04])],(221,230,235),12)
  for joint in [base,elbow,end]:
   px,py=project(joint);rr=9*S;d.ellipse((px-rr,py-rr,px+rr,py+rr),fill=(64,87,106),outline=(153,176,192),width=2*S)
  # Gripper fingers visibly track the contact point.
  for side in [-1,1]:
   e=end+np.array([0,side*.1,0]);line(d,[e,e+np.array([.16,0,-.11])],(33,48,60),3)
  if t>=6.0:
   ring(d,1.4,-.4,.82,.15,(52,146,101),2)
  if t>8: im=Image.blend(im,frames[0].resize((W*S,H*S)),(t-8)/1)
  frames.append(im.resize((W,H),Image.Resampling.LANCZOS))
 return frames

def save(name,frames):
 cmd=['ffmpeg','-v','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r','20','-i','-','-an','-c:v','libx264','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/(name+'.mp4'))]
 proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
 for frame in frames: proc.stdin.write(frame.tobytes())
 proc.stdin.close();assert proc.wait()==0
 frames[35].save(OUT/(name+'.jpg'),quality=90)
 print(name,len(frames),'frames')
if __name__=='__main__':
 save('dqn-obstacles',navigation());save('policy-eval',policy())
