import random

TOP,C = -0.01,0.04
COL = {'red':'0.9 0.1 0.1','green':'0.1 0.75 0.1','blue':'0.1 0.2 0.9'}
BINS = {'red':(0.25,0.45),'green':(0.40,0.45),'blue':(0.55,0.45)}


def box(n,size,pose,col,mu=''):
    if len(pose.split())==3:
        pose += ' 0 0 0'   # SDF needs x y z roll pitch yaw
    g = f'<geometry><box><size>{size}</size></box></geometry>'
    return (f'<visual name="{n}v"><pose>{pose}</pose>{g}<material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse>'
            f'</material></visual><collision name="{n}c"><pose>{pose}</pose>{g}{mu}</collision>')


def model(name,pose,body,static=True,inertial=''):
    return (f'<model name="{name}"><static>{str(static).lower()}</static><pose>{pose}</pose>'
            f'<link name="l">{inertial}{body}</link></model>')


def build(seed=3,n=6):
    rnd,pts,out = random.Random(seed),[],[]
    out.append(model('table',f'0.35 0 {TOP - 0.025} 0 0 0',box('t','1.4 1.6 0.05','0 0 0','0.55 0.55 0.55')))

    for c,(x,y) in BINS.items():
        w=[box('f','0.12 0.12 0.01','0 0 0.005',COL[c])]
        for i,(s,p) in enumerate([('0.12 0.01 0.05','0 0.06 0.025'),('0.12 0.01 0.05','0 -0.06 0.025'),
                                  ('0.01 0.12 0.05','0.06 0 0.025'),('0.01 0.12 0.05','-0.06 0 0.025')]):
            w.append(box(f'w{i}',s,p,COL[c]))
        out.append(model(f'bin_{c}',f'{x} {y} {TOP} 0 0 0',''.join(w)))
    out.append('<model name="overhead_camera"><static>true</static><pose>0.5 0.05 1.0 0 1.5708 0</pose>'
               '<link name="l"><sensor name="cam" type="camera"><always_on>1</always_on><update_rate>15</update_rate>'
               '<topic>camera/image_raw</topic><gz_frame_id>camera_optical_frame</gz_frame_id>'
               '<camera><horizontal_fov>1.047</horizontal_fov><image><width>640</width><height>480</height>'
               '<format>R8G8B8</format></image><clip><near>0.1</near><far>5</far></clip></camera></sensor></link></model>')
    while len(pts)<n:
        x,y = rnd.uniform(0.40,0.62),rnd.uniform(-0.25,0.20)
        if all((x-a)**2+(y-b)**2>0.09 **2 for a,b in pts):
            c,i=list(COL)[len(pts)%3],0.05*C*C/6
            ine =f'<inertial><mass>0.05</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>'
            mu = '<surface><friction><ode><mu>2</mu><mu2>2</mu2></ode></friction></surface>'
            out.append(model(f'cube{len(pts)}_{c}',f'{x:.3f} {y:.3f} {TOP+C/2+0.001} 0 0 {rnd.uniform(-.7,.7):.2f}',
                     box('c',f'{C} {C} {C}','0 0 0',COL[c],mu),False,ine))
            pts.append((x,y))
    plugins =''.join(f'<plugin filename="gz-sim-{p}-system" name="gz::sim::systems::{c}"/>' for p,c in 
                    [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster')])
    
    return (f'<?xml version="1.0"?><sdf version="1.9"><world name="w">{plugins}'
            '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
            '<scene><ambient>0.6 0.6 0.6 1</ambient><shadows>false</shadows></scene>'
            '<light type="directional" name="sun"><direction>-0.2 0.2 -1</direction><diffuse>0.9 0.9 0.9 1</diffuse></light>'
            +''.join(out)+'</world></sdf>')


