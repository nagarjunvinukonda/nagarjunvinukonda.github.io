"""Collision-checked illustrative TurtleBot navigation; not a DQN policy rollout.
Moving discs use rigid collision responses. Space-time A* routes the robot around
predicted traffic and a fixed box. The rendered spline is checked at 200 Hz.
"""
import heapq, math, json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicSpline
from render_portfolio_demos import scene, box, disk, ring, cylinder, line, project, save, W,H,S,FONT,OUT
from PIL import Image, ImageDraw, ImageFont

DT=.01; STEP=.2; TICK=.35; ROBOT_R=.46; OB_R=.40; MARGIN=.20
START=np.array([-3.2,-2.4]); GOAL=np.array([3.2,2.4]); HALF=np.array([.70,.65])

def cube_clear(p):
 return np.linalg.norm(np.maximum(np.abs(p)-HALF,0),axis=-1)

def obstacle_tracks(duration=28):
 p=np.array([[-1.3,-2.5],[2.7,.15],[-2.5,2.3]],float)
 v=np.array([[.43,.31],[-.39,.47],[.49,-.32]],float)
 frames=[]
 for i in range(int(duration/DT)+1):
  frames.append(p.copy());p+=v*DT
  # Bodies keep a small visible air gap. Rebound rather than intersect.
  for k in range(3):
   for axis,limit in [(0,4.35),(1,3.8)]:
    bound=limit-OB_R-.12
    if abs(p[k,axis])>bound:
     p[k,axis]=np.sign(p[k,axis])*bound;v[k,axis]=-np.sign(p[k,axis])*abs(v[k,axis])
   nearest=np.clip(p[k],-HALF,HALF);delta=p[k]-nearest;dist=np.linalg.norm(delta)
   if dist<OB_R+.16:
    assert dist>0
    n=delta/dist;p[k]=nearest+n*(OB_R+.16)
    if np.dot(v[k],n)<0:v[k]-=2*np.dot(v[k],n)*n
   # Destination is a reserved docking pad, away from traffic.
   delta=p[k]-GOAL;dist=np.linalg.norm(delta)
   if dist<OB_R+.8:
    n=delta/dist;p[k]=GOAL+n*(OB_R+.8)
    if np.dot(v[k],n)<0:v[k]-=2*np.dot(v[k],n)*n
  for a,b in [(0,1),(0,2),(1,2)]:
   delta=p[a]-p[b];dist=np.linalg.norm(delta)
   if dist<2*OB_R+.16:
    n=delta/dist;correction=(2*OB_R+.16-dist)*.5
    p[a]+=n*correction;p[b]-=n*correction
    relative=np.dot(v[a]-v[b],n)
    if relative<0:v[a]-=relative*n;v[b]+=relative*n
 return np.array(frames)

def obs_at(tracks,t):
 q=np.asarray(t)/DT;a=np.floor(q).astype(int);a=np.clip(a,0,len(tracks)-2);f=q-a
 return tracks[a]*(1-f)[...,None,None]+tracks[a+1]*f[...,None,None]

def plan(tracks):
 moves=[(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1),(0,0)]
 goal=tuple(np.rint((GOAL-START)/STEP).astype(int));origin=(0,0,0,1)
 queue=[(0.,0.,origin)];cost={origin:0.};parent={};expanded=0
 fractions=np.array([.0,.25,.5,.75,1.])
 # Cache predicted obstacle positions at every edge sample.
 forecasts=[obs_at(tracks,(k+fractions)*TICK) for k in range(75)]
 while queue:
  _,g,state=heapq.heappop(queue)
  if g>cost[state]+1e-8:continue
  ix,iy,k,last=state;expanded+=1
  if (ix,iy)==goal:
   states=[state]
   while states[-1]!=origin:states.append(parent[states[-1]])
   states.reverse();times=np.array([q[2]*TICK for q in states]);points=np.array([START+STEP*np.array(q[:2]) for q in states])
   print('A* states expanded:',expanded,'travel seconds:',times[-1]);return times,points
  if k>=74:continue
  p=START+STEP*np.array([ix,iy])
  for direction,(dx,dy) in enumerate(moves):
   nx,ny=ix+dx,iy+dy;end=START+STEP*np.array([nx,ny])
   if abs(end[0])>3.75 or abs(end[1])>3.2:continue
   samples=p+fractions[:,None]*(end-p)
   if np.min(cube_clear(samples))<ROBOT_R+MARGIN:continue
   if np.min(np.linalg.norm(samples[:,None,:]-forecasts[k],axis=2))<ROBOT_R+OB_R+MARGIN:continue
   turn=min((direction-last)%8,(last-direction)%8) if direction<8 and last<8 else 0
   ng=g+TICK+.055*turn+(.015 if direction==8 else 0)
   ns=(nx,ny,k+1,direction)
   if ng<cost.get(ns,1e9):
    cost[ns]=ng;parent[ns]=state
    heuristic=np.linalg.norm(end-GOAL)/(STEP*2**.5/TICK)
    heapq.heappush(queue,(ng+heuristic,ng,ns))
 raise RuntimeError('No safe path')

def verify(tracks,times,points):
 spline=CubicSpline(times,points,bc_type=((1,[0,0]),(1,[0,0])))
 t=np.arange(0,times[-1]+1.5,.005);p=spline(np.minimum(t,times[-1]));obs=obs_at(tracks,t)
 robot_dynamic=float(np.min(np.linalg.norm(p[:,None,:]-obs,axis=2)-ROBOT_R-OB_R))
 robot_static=float(np.min(cube_clear(p)-ROBOT_R))
 pairs=[float(np.min(np.linalg.norm(obs[:,a]-obs[:,b],axis=1)-2*OB_R)) for a,b in [(0,1),(0,2),(1,2)]]
 moving_static=float(np.min(cube_clear(obs)-OB_R))
 wall=float(min(np.min(4.35-np.abs(p[:,0])-ROBOT_R),np.min(3.8-np.abs(p[:,1])-ROBOT_R)))
 metrics={'sample_rate_hz':200,'robot_moving_obstacle_clearance':robot_dynamic,'robot_cube_clearance':robot_static,'moving_obstacle_pair_clearance':min(pairs),'moving_obstacle_cube_clearance':moving_static,'robot_wall_clearance':wall,'goal_error':float(np.linalg.norm(spline(times[-1])-GOAL)),'duration_seconds':float(times[-1]+1.5)}
 assert min(robot_dynamic,robot_static,min(pairs),moving_static,wall)>.10,metrics
 assert metrics['goal_error']<.001,metrics
 print(json.dumps(metrics,indent=2));return spline,metrics

def robot(d,x,y,heading):
 # TurtleBot-style circular decks, two driven wheels, standoffs, LiDAR and bumper.
 c,s=math.cos(heading),math.sin(heading)
 def local(dx,dy,z):return (x+c*dx-s*dy,y+s*dx+c*dy,z)
 disk(d,x+.07,y+.1,.51,.005,(30,40,48))
 for side in [-1,1]:
  wx,wy,_=local(0,side*.35,0)
  box(d,wx,wy,.03,.23,.17,.28,(28,34,39))
 cylinder(d,x,y,.35,.13,.14,(74,91,106))
 cylinder(d,x,y,.36,.27,.055,(230,235,235))
 for a,b in [(-.23,-.20),(.23,-.20),(.23,.20),(-.23,.20)]:line(d,[local(a,b,.31),local(a,b,.55)],(173,188,197),2.5)
 cylinder(d,x,y,.34,.54,.06,(226,231,231));cylinder(d,x,y,.13,.60,.15,(45,53,65))
 disk(d,x,y,.08,.755,(203,157,42))
 line(d,[local(.2,-.14,.61),local(.28,0,.61),local(.2,.14,.61)],(33,162,205),3)
 px,py,_=local(-.18,0,.6);box(d,px,py,.6,.14,.17,.10,(66,80,87))

def render():
 tracks=obstacle_tracks();times,points=plan(tracks);spline,metrics=verify(tracks,times,points)
 frames=[];trail=[];heading=math.atan2(*(points[1]-points[0])[::-1]);endtime=times[-1]
 for frame_t in np.arange(0,endtime+1.5,.05):
  p=spline(min(frame_t,endtime));vel=spline(min(frame_t,endtime),1)
  if np.linalg.norm(vel)>.02:
   target=math.atan2(vel[1],vel[0]);delta=math.atan2(math.sin(target-heading),math.cos(target-heading));heading+=np.clip(delta,-.12,.12)
  x,y=p;obs=obs_at(tracks,frame_t)
  im,d=scene('TURTLEBOT  /  DYNAMIC OBSTACLE AVOIDANCE')
  box(d,0,-4.1,0,9.1,.13,.38,(99,116,129));box(d,-4.5,0,0,.13,8.2,.38,(99,116,129))
  disk(d,*GOAL,.63,.018,(53,91,75));ring(d,*GOAL,.65,.023,(103,222,146),2)
  gx,gy=project((*GOAL,.04));d.text((gx-15*S,gy+16*S),'GOAL',font=ImageFont.truetype(FONT,10*S),fill=(155,239,176))
  trail.append((x,y,.025))
  if len(trail)>1:line(d,trail,(72,191,180),2)
  # Ground-level LiDAR footprint, clipped to the first solid surface.
  for th in np.linspace(0,2*math.pi,48,endpoint=False):
   dist=1.9
   for q in np.arange(.47,1.91,.04):
    px,py=x+q*math.cos(th),y+q*math.sin(th)
    if abs(px)>4.35 or abs(py)>3.8 or (abs(px)<HALF[0] and abs(py)<HALF[1]) or np.min(np.linalg.norm(obs-np.array([px,py]),axis=1))<OB_R:
     dist=q;break
   line(d,[(x,y,.025),(x+dist*math.cos(th),y+dist*math.sin(th),.025)],(64,108,111),.6)
  objects=[('ob',*o) for o in obs]+[('cube',0.,0.),('bot',x,y)]
  # Include the cube in depth ordering: solid objects correctly occlude one another.
  for kind,ox,oy in sorted(objects,key=lambda o:13*o[1]+21*o[2]):
   if kind=='cube':box(d,0,0,0,2*HALF[0],2*HALF[1],.8,(132,153,170))
   elif kind=='ob':
    disk(d,ox+.08,oy+.1,OB_R*1.13,.01,(29,41,49))
    cylinder(d,ox,oy,OB_R,0,.7,(221,155,71));cylinder(d,ox,oy,OB_R+.003,.48,.10,(245,218,155))
   else:robot(d,x,y,heading)
  status='GOAL REACHED' if frame_t>=endtime else 'NAVIGATING'
  d.text((490*S,338*S),status,font=ImageFont.truetype(FONT,10*S),fill=(131,218,163))
  frames.append(im.resize((W,H),Image.Resampling.LANCZOS))
 # Fade through an empty dark frame to make the episode restart explicit.
 dark=Image.new('RGB',(W,H),(21,31,42));last=frames[-1];first=frames[0]
 frames += [Image.blend(last,dark,float(a)) for a in np.linspace(0,1,7)]
 frames += [dark]*3
 frames += [Image.blend(dark,first,float(a)) for a in np.linspace(0,1,7)]
 save('dqn-obstacles',frames)
 (OUT/'dqn-validation.json').write_text(json.dumps(metrics,indent=2)+'\n')
 frames[len(frames)//2].save('/tmp/turtlebot-preview.jpg')
 # Small contact sheet to inspect several times in the same rollout.
 contact=Image.new('RGB',(W*3,H*2))
 for j,i in enumerate(np.linspace(0,len(frames)-20,6).astype(int)):contact.paste(frames[i],((j%3)*W,(j//3)*H))
 contact.save('/tmp/turtlebot-contact.jpg')
if __name__=='__main__':render()
