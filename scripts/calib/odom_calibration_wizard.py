#!/usr/bin/env python3
"""Interactive odometry calibration wizard for RDK X5 + Yahboom M1.
It does not drive the robot. You manually drive it and type commands:
  static 30
  start rot / done rot 360
  start line / done line 1.0
  save / q
"""
import argparse, csv, json, math, os, queue, threading, time
from datetime import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def yaw_q(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def adiff(a,b): return math.atan2(math.sin(a-b), math.cos(a-b))

def pabs(vals,p):
    if not vals: return 0.0
    a=sorted(abs(x) for x in vals); i=round((len(a)-1)*p)
    return a[max(0,min(int(i),len(a)-1))]

class Wizard(Node):
    def __init__(self,args):
        super().__init__('odom_calibration_wizard')
        self.args=args
        self.create_subscription(Odometry,args.odom_topic,self.odom_cb,50)
        self.create_subscription(Twist,args.cmd_topic,self.cmd_cb,20)
        self.create_subscription(Twist,args.cmd_sent_topic,self.sent_cb,20)
        self.create_subscription(String,args.state_topic,self.state_cb,20)
        self.odom=None; self.cmd=None; self.sent=None; self.state={}
        self.last_yaw=None; self.yaw=0.0; self.baseline=None; self.active=''; self.static_until=None; self.static_samples=[]; self.results=[]
        self.cmdq=queue.Queue()
        os.makedirs(args.out_dir,exist_ok=True)
        stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path=os.path.join(args.out_dir,f'odom_calib_records_{stamp}.csv')
        self.json_path=os.path.join(args.out_dir,f'odom_calib_results_{stamp}.json')
        self.csv=open(self.csv_path,'w',newline='',encoding='utf-8')
        self.writer=csv.DictWriter(self.csv,fieldnames=['time','event','test','odom_x','odom_y','yaw_unwrapped','odom_vx','odom_vy','odom_wz','cmd_vx','cmd_wz','cmd_sent_vx','cmd_sent_wz','pwm_1','pwm_2','pwm_3','pwm_4','vx_pwm','wz_pwm','last_sent_vx','last_sent_wz'])
        self.writer.writeheader()
        threading.Thread(target=self.input_loop,daemon=True).start()
        self.timer=self.create_timer(0.05,self.tick)
        self.help(); self.get_logger().info(f'CSV: {self.csv_path}')
    def help(self):
        print('\n========== ODOM CALIBRATION WIZARD ==========')
        print('static 30              stationary sample; recommend deadzones')
        print('start line             baseline before straight line')
        print('done line 1.0          after real 1.0m; compute VX_SCALE')
        print('start rot              baseline before rotation')
        print('done rot 360           after real 360deg; compute WZ_SCALE')
        print('mark note              record note')
        print('save                   save JSON')
        print('q                      save and quit')
        print('=============================================\n')
    def input_loop(self):
        while True:
            try: self.cmdq.put(input('calib> ').strip())
            except EOFError: return
    def odom_cb(self,msg):
        self.odom=msg; y=yaw_q(msg.pose.pose.orientation)
        if self.last_yaw is None: self.last_yaw=y; self.yaw=y; return
        self.yaw += adiff(y,self.last_yaw); self.last_yaw=y
    def cmd_cb(self,msg): self.cmd=msg
    def sent_cb(self,msg): self.sent=msg
    def state_cb(self,msg):
        try: self.state=json.loads(msg.data)
        except Exception: self.state={}
    def snap(self):
        if self.odom is None: return None
        o=self.odom
        return {'time':time.time(),'x':o.pose.pose.position.x,'y':o.pose.pose.position.y,'yaw':self.yaw,'vx':o.twist.twist.linear.x,'vy':o.twist.twist.linear.y,'wz':o.twist.twist.angular.z}
    def process(self,text):
        if not text: return
        parts=text.split(); c=parts[0].lower()
        if c in ('q','quit','exit'):
            self.save(); self.csv.close(); rclpy.shutdown(); return
        if c=='help': self.help(); return
        if c=='save': self.save(); return
        if c=='mark':
            note=text[5:].strip() if len(text)>5 else ''
            self.results.append({'type':'mark','time':time.time(),'note':note}); print('[MARK]',note); return
        if c=='static':
            sec=float(parts[1]) if len(parts)>1 else 30.0
            self.static_until=time.time()+sec; self.static_samples=[]; self.active='static'
            print(f'[STATIC] sampling {sec:.1f}s. Do not touch robot.'); return
        if c=='start':
            if len(parts)<2 or parts[1] not in ('line','rot'):
                print('Usage: start line OR start rot'); return
            s=self.snap()
            if s is None: print('[ERROR] no /odom yet'); return
            self.baseline=s; self.active=parts[1]
            print(f'[START {self.active}] x={s["x"]:.4f}, y={s["y"]:.4f}, yaw={s["yaw"]:.4f}'); return
        if c=='done':
            if len(parts)<3: print('Usage: done line 1.0 OR done rot 360'); return
            if self.baseline is None: print('[ERROR] run start line/start rot first'); return
            s=self.snap()
            if s is None: print('[ERROR] no /odom yet'); return
            if parts[1]=='line': self.finish_line(float(parts[2]),s)
            elif parts[1]=='rot': self.finish_rot(float(parts[2]),s)
            else: print('kind must be line or rot')
            return
        print('Unknown command. Type help.')
    def finish_line(self,actual,s):
        b=self.baseline; dx=s['x']-b['x']; dy=s['y']-b['y']; dyaw=s['yaw']-b['yaw']; yaw0=b['yaw']
        forward=dx*math.cos(yaw0)+dy*math.sin(yaw0); lateral=-dx*math.sin(yaw0)+dy*math.cos(yaw0); disp=math.hypot(dx,dy)
        scale=actual/forward if abs(forward)>1e-6 else None
        r={'type':'line','actual_m':actual,'odom_forward_m':forward,'odom_lateral_m':lateral,'odom_displacement_m':disp,'yaw_drift_rad':dyaw,'duration_s':s['time']-b['time'],'recommended_CHASSIS_ODOM_VX_SCALE':scale}
        self.results.append(r)
        print('\n========== LINE RESULT ==========')
        print(f'real distance: {actual:.4f} m')
        print(f'odom forward:  {forward:.4f} m')
        print(f'odom lateral:  {lateral:.4f} m')
        print(f'odom total:    {disp:.4f} m')
        print(f'yaw drift:     {dyaw:.4f} rad')
        if scale is not None: print(f'recommended CHASSIS_ODOM_VX_SCALE={scale:.4f}')
        print('=================================\n')
    def finish_rot(self,actual_deg,s):
        b=self.baseline; dx=s['x']-b['x']; dy=s['y']-b['y']; dyaw=s['yaw']-b['yaw']; actual=math.radians(actual_deg)
        scale_abs=abs(actual)/abs(dyaw) if abs(dyaw)>1e-6 else None
        scale_signed=actual/dyaw if abs(dyaw)>1e-6 else None
        r={'type':'rot','actual_deg':actual_deg,'actual_rad':actual,'odom_yaw_delta_rad':dyaw,'translation_drift_m':math.hypot(dx,dy),'dx_m':dx,'dy_m':dy,'duration_s':s['time']-b['time'],'recommended_CHASSIS_ODOM_WZ_SCALE_ABS':scale_abs,'signed_scale_check':scale_signed}
        self.results.append(r)
        print('\n========== ROTATION RESULT ==========')
        print(f'real angle:     {actual_deg:.2f} deg = {actual:.4f} rad')
        print(f'odom yaw delta: {dyaw:.4f} rad')
        print(f'translation drift during rotation: {math.hypot(dx,dy):.4f} m')
        if scale_abs is not None:
            print(f'recommended CHASSIS_ODOM_WZ_SCALE={scale_abs:.4f}')
            if scale_signed is not None and scale_signed<0: print('[WARNING] sign mismatch: check angular.z direction/motor_signs/yaw sign.')
        print('=====================================\n')
    def finish_static(self):
        ss=self.static_samples; self.static_until=None; self.active=''
        vx=[x['vx'] for x in ss]; vy=[x['vy'] for x in ss]; wz=[x['wz'] for x in ss]
        p95x,p95y,p95w=pabs(vx,0.95),pabs(vy,0.95),pabs(wz,0.95)
        rec_vxy=max(0.003,1.5*max(p95x,p95y)); rec_wz=max(0.015,1.5*p95w)
        r={'type':'static','samples':len(ss),'p95_abs_vx':p95x,'p95_abs_vy':p95y,'p95_abs_wz':p95w,'max_abs_vx':max([abs(x) for x in vx],default=0.0),'max_abs_vy':max([abs(x) for x in vy],default=0.0),'max_abs_wz':max([abs(x) for x in wz],default=0.0),'recommended_CHASSIS_ODOM_VXY_DEADZONE':rec_vxy,'recommended_CHASSIS_ODOM_WZ_DEADZONE':rec_wz}
        self.results.append(r)
        print('\n========== STATIC RESULT ==========')
        print(f'samples: {len(ss)}')
        print(f'p95 |vx|={p95x:.6f}, |vy|={p95y:.6f}, |wz|={p95w:.6f}')
        print(f'recommended CHASSIS_ODOM_VXY_DEADZONE={rec_vxy:.6f}')
        print(f'recommended CHASSIS_ODOM_WZ_DEADZONE={rec_wz:.6f}')
        print('===================================\n')
    def tick(self):
        while not self.cmdq.empty(): self.process(self.cmdq.get())
        s=self.snap()
        if s:
            if self.static_until is not None:
                self.static_samples.append(s)
                if time.time()>=self.static_until: self.finish_static()
            self.write_row('sample')
    def write_row(self,event):
        if not self.odom: return
        o=self.odom; st=self.state or {}; cmd=self.cmd; sent=self.sent
        row={'time':time.time(),'event':event,'test':self.active,'odom_x':o.pose.pose.position.x,'odom_y':o.pose.pose.position.y,'yaw_unwrapped':self.yaw,'odom_vx':o.twist.twist.linear.x,'odom_vy':o.twist.twist.linear.y,'odom_wz':o.twist.twist.angular.z,'cmd_vx':cmd.linear.x if cmd else '','cmd_wz':cmd.angular.z if cmd else '','cmd_sent_vx':sent.linear.x if sent else '','cmd_sent_wz':sent.angular.z if sent else '','pwm_1':st.get('pwm_1',''),'pwm_2':st.get('pwm_2',''),'pwm_3':st.get('pwm_3',''),'pwm_4':st.get('pwm_4',''),'vx_pwm':st.get('vx_pwm',''),'wz_pwm':st.get('wz_pwm',''),'last_sent_vx':st.get('last_sent_vx',''),'last_sent_wz':st.get('last_sent_wz','')}
        self.writer.writerow(row); self.csv.flush()
    def save(self):
        with open(self.json_path,'w',encoding='utf-8') as f:
            json.dump({'created_at':datetime.now().isoformat(),'csv_path':self.csv_path,'results':self.results},f,indent=2,ensure_ascii=False)
        print(f'[SAVE] results saved: {self.json_path}')

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--odom-topic',default='/odom'); p.add_argument('--cmd-topic',default='/cmd_vel'); p.add_argument('--cmd-sent-topic',default='/cmd_vel_sent'); p.add_argument('--state-topic',default='/chassis_bridge_state'); p.add_argument('--out-dir',default='logs/calib')
    args=p.parse_args(); rclpy.init(); node=Wizard(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.save()
    finally:
        try: node.csv.close()
        except Exception: pass
if __name__=='__main__': main()
