#!/usr/bin/env python3
"""Rotation sweep tester for RDK X5 + Yahboom M1.
Publishes increasing /cmd_vel angular.z, logs /odom and /chassis_bridge_state.
Commands while running: ok, n, p, q.
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

def adiff(a,b):
    return math.atan2(math.sin(a-b), math.cos(a-b))

class RotateSweep(Node):
    def __init__(self,args):
        super().__init__('rotate_sweep_test')
        self.args=args
        self.pub=self.create_publisher(Twist,args.cmd_topic,10)
        self.create_subscription(Odometry,args.odom_topic,self.odom_cb,30)
        self.create_subscription(String,args.state_topic,self.state_cb,30)
        self.create_subscription(Twist,args.cmd_sent_topic,self.sent_cb,20)
        self.odom=None; self.state={}; self.sent=None
        self.last_yaw=None; self.yaw=0.0; self.stage_start_yaw=0.0; self.stage_start_time=time.time()
        self.cmdq=queue.Queue(); self.stop=False; self.pause=False; self.stage=0; self.ok_marks=[]
        self.wzs=self.make_wzs(args)
        os.makedirs(args.out_dir,exist_ok=True)
        stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path=os.path.join(args.out_dir,f'rotate_sweep_{stamp}.csv')
        self.json_path=os.path.join(args.out_dir,f'rotate_sweep_{stamp}.json')
        self.csv=open(self.csv_path,'w',newline='',encoding='utf-8')
        self.writer=csv.DictWriter(self.csv,fieldnames=['time','stage','target_wz','mode','yaw_unwrapped','stage_yaw_delta','odom_x','odom_y','odom_vx','odom_vy','odom_wz','cmd_sent_vx','cmd_sent_wz','last_sent_vx','last_sent_wz','vx_pwm','wz_pwm','pwm_1','pwm_2','pwm_3','pwm_4','wheel_layout','motor_signs','wz_pwm_deadband','wz_pwm_gain','pwm_max'])
        self.writer.writeheader()
        threading.Thread(target=self.input_loop,daemon=True).start()
        self.timer=self.create_timer(1.0/args.rate_hz,self.tick)
        self.get_logger().info(f'wz stages: {self.wzs}')
        self.get_logger().info('Type ok=mark good, n=next, p=pause, q=quit')
        self.get_logger().info(f'CSV: {self.csv_path}')
    def make_wzs(self,args):
        if args.wz_list:
            return [float(x.strip()) for x in args.wz_list.split(',') if x.strip()]
        out=[]; v=args.wz_start
        while v<=args.wz_end+1e-9:
            out.append(round(v,4)); v+=args.wz_step
        return out
    def input_loop(self):
        while True:
            try: self.cmdq.put(input().strip().lower())
            except EOFError: return
    def odom_cb(self,msg):
        self.odom=msg; y=yaw_q(msg.pose.pose.orientation)
        if self.last_yaw is None:
            self.last_yaw=y; self.yaw=y; self.stage_start_yaw=y; return
        self.yaw += adiff(y,self.last_yaw); self.last_yaw=y
    def state_cb(self,msg):
        try: self.state=json.loads(msg.data)
        except Exception: self.state={}
    def sent_cb(self,msg): self.sent=msg
    def cur_wz(self): return self.wzs[self.stage] if self.stage < len(self.wzs) else 0.0
    def next_stage(self,why):
        old=self.stage; self.stage+=1; self.stage_start_yaw=self.yaw; self.stage_start_time=time.time()
        self.get_logger().info(f'stage {old}->{self.stage}, {why}')
        if self.stage>=len(self.wzs): self.stop=True
    def process_cmds(self):
        while not self.cmdq.empty():
            c=self.cmdq.get()
            if c in ('q','quit','stop','s'): self.stop=True
            elif c=='ok':
                item={'time':time.time(),'stage':self.stage,'target_wz':self.cur_wz(),'yaw_unwrapped':self.yaw,'state':self.state}
                self.ok_marks.append(item); self.get_logger().info(f'OK marked at wz={self.cur_wz():.3f}')
            elif c=='n': self.next_stage('manual')
            elif c=='p': self.pause=not self.pause; self.get_logger().info(f'pause={self.pause}')
            elif c: self.get_logger().info('commands: ok, n, p, q')
    def publish_stop(self):
        m=Twist(); self.pub.publish(m)
    def tick(self):
        self.process_cmds()
        if self.stop:
            self.publish_stop(); self.save(); self.csv.close(); rclpy.shutdown(); return
        if self.stage>=len(self.wzs): self.publish_stop(); return
        dyaw=abs(self.yaw-self.stage_start_yaw); dt=time.time()-self.stage_start_time
        if self.args.stage_mode=='turns' and dyaw>=abs(self.args.turns_per_stage)*2*math.pi: self.next_stage('turns')
        if self.args.stage_mode=='seconds' and dt>=self.args.seconds_per_stage: self.next_stage('seconds')
        if self.pause: self.publish_stop()
        else:
            m=Twist(); m.angular.z=self.cur_wz(); self.pub.publish(m)
        self.write_row(dyaw)
    def write_row(self,dyaw):
        st=self.state or {}; o=self.odom; sent=self.sent
        row={'time':time.time(),'stage':self.stage,'target_wz':self.cur_wz(),'mode':'paused' if self.pause else 'running','yaw_unwrapped':self.yaw,'stage_yaw_delta':dyaw,'odom_x':'','odom_y':'','odom_vx':'','odom_vy':'','odom_wz':'','cmd_sent_vx':sent.linear.x if sent else '','cmd_sent_wz':sent.angular.z if sent else '','last_sent_vx':st.get('last_sent_vx',''),'last_sent_wz':st.get('last_sent_wz',''),'vx_pwm':st.get('vx_pwm',''),'wz_pwm':st.get('wz_pwm',''),'pwm_1':st.get('pwm_1',''),'pwm_2':st.get('pwm_2',''),'pwm_3':st.get('pwm_3',''),'pwm_4':st.get('pwm_4',''),'wheel_layout':st.get('wheel_layout',''),'motor_signs':st.get('motor_signs',''),'wz_pwm_deadband':st.get('wz_pwm_deadband',''),'wz_pwm_gain':st.get('wz_pwm_gain',''),'pwm_max':st.get('pwm_max','')}
        if o:
            row.update(odom_x=o.pose.pose.position.x,odom_y=o.pose.pose.position.y,odom_vx=o.twist.twist.linear.x,odom_vy=o.twist.twist.linear.y,odom_wz=o.twist.twist.angular.z)
        self.writer.writerow(row); self.csv.flush()
    def save(self):
        rec=None
        if self.ok_marks: rec={'first_ok_target_wz':self.ok_marks[0]['target_wz'],'note':'Use near this target_wz as reliable rotation region; then run 360 degree WZ_SCALE calibration.'}
        with open(self.json_path,'w',encoding='utf-8') as f:
            json.dump({'created_at':datetime.now().isoformat(),'csv_path':self.csv_path,'wz_values':self.wzs,'ok_marks':self.ok_marks,'recommendation':rec},f,indent=2,ensure_ascii=False)
        self.get_logger().info(f'Saved: {self.json_path}')

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--cmd-topic',default='/cmd_vel'); p.add_argument('--odom-topic',default='/odom'); p.add_argument('--cmd-sent-topic',default='/cmd_vel_sent'); p.add_argument('--state-topic',default='/chassis_bridge_state'); p.add_argument('--out-dir',default='logs/calib')
    p.add_argument('--wz-list',default=''); p.add_argument('--wz-start',type=float,default=0.06); p.add_argument('--wz-end',type=float,default=0.18); p.add_argument('--wz-step',type=float,default=0.02)
    p.add_argument('--stage-mode',choices=['seconds','turns'],default='seconds'); p.add_argument('--seconds-per-stage',type=float,default=8.0); p.add_argument('--turns-per-stage',type=float,default=2.0); p.add_argument('--rate-hz',type=float,default=10.0)
    args=p.parse_args(); rclpy.init(); node=RotateSweep(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.publish_stop(); node.save()
    finally:
        try: node.publish_stop(); node.csv.close()
        except Exception: pass
if __name__=='__main__': main()
