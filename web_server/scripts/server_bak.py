
import re
import math
import shutil
import os
import sys
import os.path as osp
import urllib
import mimetypes
import numpy as np
import Image as PILImage
from BaseHTTPServer import *
import socket
import subprocess

NODE_NAME = 'webServer'
import roslib; roslib.load_manifest(NODE_NAME)
import rospy
import yaml
import tf
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from tf.msg import *


SERVER_ADDRESS = ('', 8888)
ROOT_PATH = osp.dirname(osp.realpath(__file__))
STATIC_PATH = osp.join(ROOT_PATH, "static")
DEFAULT_HOME = "home_scout.html"

DEFAULT_MAP = "src/webServer/scripts/idmindfloor2.yaml"
#DEFAULT_MAPFILE = os.path.join(roslib.packages.get_pkg_dir('maps'), #DEFAULT_MAP+".yaml")
#DEFAULT_MAPFILE = []
#DEFAULT_MAP = []

RRAD  = 0.41/2

STDEV_XY = 0.2
STDEV_TH = np.deg2rad(30)

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
COV0 = [STDEV_XY**2, 0, 0, 0, 0, 0,
        0, STDEV_XY**2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, STDEV_TH**2]


class Server(BaseHTTPRequestHandler):

    def log_request(self, *args):
        pass
    
    def do_GET(self):
        self.serve('GET')
        
    def do_HEAD(self):
        self.serve('HEAD')

    def serve(self, method):
        for (pat, func) in self.URLS:
            m = re.match(pat, self.path)
            if m is not None:
                func(self, method, m)
                return
        self.send_error(404, "Not found: %s"%(self.path))

    def handle_home(self, method, match):
        self.send_response(301)
        self.send_header("Location", "/static/"+DEFAULT_HOME)
        self.end_headers()

    def handle_static(self, method, match):
        fn = osp.normpath(osp.join(STATIC_PATH, match.group(1)))
        if fn.startswith(STATIC_PATH):
            try:
                ctype = mimetypes.guess_type(fn)
                stat = os.stat(fn)
                with open(fn) as fh:
                    self.send_response(200)
                    self.send_header("Content-type", ctype)
                    self.send_header("Content-Length", str(stat[6]))
                    self.end_headers()
                    if method=='GET':
                        shutil.copyfileobj(fh, self.wfile)
            except (IOError, OSError):
                self.send_error(404, "Unknown URL: %s"%(match.group()))
        else:
            self.send_error(404, "Unauthorized")

    def handle_do(self, method, match):
        reqs = match.group(1).split('?', 1)
        cmd  = reqs[0]
        args = reqs[1].split('&') if len(reqs)>1 else []
        for (c, f) in self.CMDS:
            if cmd==c:
                f(self, method, args)
                return
        self.send_error(404, "Unrecognized request")

    def header(self, type):
        self.send_response(200)
        self.send_header("Content-type", type)
        self.end_headers()

    def handle_map(self, method, args):
        self.header("text/xml")
        if method=='GET':
            self.wfile.write(proxy.get_map())

    def handle_map_img(self, method, args):
        self.header("image/png")
        if method=='GET':
            fn = proxy.get_map_img_path(args[0])
            im = PILImage.open(fn)
            im.save(self.wfile, 'png')

    def handle_set_location(self, method, args):
        if len(args)==3:
            (x, y, t) = map(float, args)
            self.header("text/plain")
            (suc, res) = proxy.set_pose(x, y, t)
            if method=='GET':
                print >>self.wfile, res

    def handle_set_dct_goal(self, method, args):
        if len(args)==3:
            (x, y, t) = map(float, args)
            self.header("text/plain")
            (suc, res) = proxy.set_dct_goal(x, y, t)
            if method=='GET':
                print >>self.wfile, res

    def handle_set_route_goal(self, method, args):
        if len(args)==3:
            (x, y, t) = map(float, args)
            self.header("text/plain")
            (suc, res) = proxy.set_route_goal(x, y, t)
            if method=='GET':
                print >>self.wfile, res

    #def handle_teleop(self, method, args):
    #    self.header("text/plain")
    #    (suc, res) = proxy.send_teleop(args)
    #    if method=='GET':
    #        print >>self.wfile, res



    # Keep this to the end of class definition
    URLS = (('^/$', handle_home),
            ('^/static/(.*)$', handle_static),
            ('^/do/(.*)$', handle_do))

    CMDS = (('map', handle_map),
            ('map_img', handle_map_img),
            ('set_location', handle_set_location),
            ('set_dct_goal', handle_set_dct_goal),
            ('set_route_goal', handle_set_route_goal))#,
            #('teleop', handle_teleop))




class Proxy:
    pose  = None
    lidar = {}
    particlecloud = None
   
    
    def __init__(self):
        rospy.init_node(NODE_NAME, argv=sys.argv)

	goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        global DEFAULT_MAPFILE
        global DEFAULT_MAP
        global DEFAULT_HOME
        global DEFAULT_CAMERA
        DEFAULT_HOME = rospy.get_param("~home", DEFAULT_HOME)
        rospy.loginfo("home: %s"%(DEFAULT_HOME))
        
        #DEFAULT_MAPFILE = rospy.get_param("map")
        #rospy.loginfo("map: %s"%(DEFAULT_MAPFILE))
        #DEFAULT_MAP = os.path.splitext(os.path.basename(DEFAULT_MAPFILE))[0]

	 # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")

        self.load_map(DEFAULT_MAP)
        self.tf = tf.TransformListener()
        self.pose_msg = PoseWithCovarianceStamped(header=Header(frame_id="/map"),
                                                  pose=PoseWithCovariance(pose=Pose(position=Point(0, 0, 0),
                                                                                    orientation=Quaternion(0, 0, 0, 1)),
                                                                          covariance=COV0))
        self.init_pub   = rospy.Publisher("initialpose", PoseWithCovarianceStamped)
        self.guid_srv   = rospy.Publisher("/move_base_simple/goal", geometry_msgs.msg.PoseStamped)
        #self.teleop_srv = rospy.ServiceProxy("/scout/teleop", TeleOpSrv)
       
        #rospy.Subscriber("/tf", tfMessage, self.tf_handler, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_handler, queue_size=1)
        rospy.Subscriber("/particlecloud", PoseArray, self.particlecloud_handler, queue_size=1)
        
        rospy.loginfo("Proxy ready")

    def p2w(self, (px, py)):
        x = self.x0 + self.scale*px
        y = self.y0 + self.scale*(self.H-py-1)
        return (x, y)

    def w2p(self, (x, y)):
        px = int(round((x - self.x0) / self.scale))
        py = int(self.H - round((y - self.y0) / self.scale) - 1)
        return (px, py)

    def np_w2p(self, (x, y)):
        px = np.rint((x - self.x0) / self.scale).astype('int')
        py = (self.H - np.rint((y - self.y0) / self.scale) - 1).astype('int')
        return (px, py)

    def load_map(self, mapfile):
        # Load raw data
        with open(mapfile) as fh:
            self.meta = yaml.load(fh)
        #
        pgm_file = os.path.join(os.path.dirname(mapfile), self.meta['image'])
        img = PILImage.open(pgm_file)
        (self.x0, self.y0, self.z0) = self.meta['origin']
        (self.W, self.H) = img.size
        self.scale = self.meta['resolution']

    def set_pose(self, xp, yp, tp):
        (x, y) = self.p2w((xp, yp))
        t = -tp
        self.pose_msg.header.stamp = rospy.get_rostime()
        self.pose_msg.pose.pose.position.x = x
        self.pose_msg.pose.pose.position.y = y
        self.pose_msg.pose.pose.orientation.z = math.sin(t/2.0)
        self.pose_msg.pose.pose.orientation.w = math.cos(t/2.0)
        self.init_pub.publish(self.pose_msg)
        return (True, "Location set to (%.2f,%.2f,%.2f)"%(x,y,t))

    def set_dct_goal(self, xp, yp, tp):
        (x, y) = self.p2w((xp, yp))
        t = -tp
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(0.000, 0.000, math.sin(t/2.0), math.cos(t/2.0)))
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # Let the user know where the robot is going next
        
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        return (True, "Goal sent")

    def set_route_goal(self, xp, yp, tp):
        (x, y) = self.p2w((xp, yp))
	t = -tp
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose = Pose(Point(x, y, 0.000), Quaternion(0.000, 0.000, math.sin(t/2.0), math.cos(t/2.0)))
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # Let the user know where the robot is going next
        
        # Start the robot toward the next location
        self.move_base.send_goal(self.goal)
        return (True, "Goal sent")

 
               
    def lidar_handler(self, scan):
        frame = scan.header.frame_id
        try:
            position, quaternion = self.tf.lookupTransform("/map", frame, rospy.Time())
            pose = (position[0], position[1], 2*math.atan2(quaternion[2], quaternion[3]))
        except tf.Exception:
            return
        (xr, yr, tr) = pose
        z = np.array(scan.ranges)
        a = tr + np.arange(scan.angle_min, scan.angle_min+scan.angle_increment*len(z), scan.angle_increment)
        #a = tr + np.arange(scan.angle_min, scan.angle_max+scan.angle_increment, scan.angle_increment)
        assert len(z)==len(a), "Range and angle arrays do not match"
        valid = (z>scan.range_min)&(z<scan.range_max)
        if valid.any():
            # project LIDAR pointcloud into map
            z, a = z[valid], a[valid]
            xw = xr + z*np.cos(a)
            yw = yr + z*np.sin(a)
            self.lidar[frame] = self.np_w2p((xw, yw))

    def particlecloud_handler(self, msg):
        n = len(msg.poses)
        poses = np.empty((n, 3))
        for i in xrange(n):
            p = msg.poses[i]
            poses[i] = (p.position.x, p.position.y, 2*math.atan2(p.orientation.z, p.orientation.w))
        (xw, yw) = self.np_w2p((poses[:,0], poses[:,1]))
        tw = -poses[:,2]
        self.particlecloud = (xw, yw, tw)

   

    def available_maps(self):
        res = []
        path = "src/webServer/scripts"
        for fn in os.listdir(path):
            if fn.endswith(".yaml"):
                res.append(fn[:-5])
        return res
        
    def get_map(self):
        map_url = '/do/map_img?' + DEFAULT_MAP
        xml = '<?xml version="1.0"?>\n<map>\n<img>%s</img>\n'%(map_url)
        try:
            position, quaternion = self.tf.lookupTransform("/map", "/base_link", rospy.Time())
            pose = (position[0], position[1], 2*math.atan2(quaternion[2], quaternion[3]))
            (x, y, t) = pose
            (px, py)  = self.w2p((x, y))
            pr        = RRAD / self.scale
            xml += '<robot x="%d" y="%d" t="%.3f" r="%d"/>\n'%(px,py,t,pr)
        except tf.Exception:
            rospy.logwarn("[webconsole] Failed to get robot pose -- IGNORING")
        if len(self.lidar)>0:
            xml += '<lidar>\n'
            for (frame,(xl,yl)) in self.lidar.iteritems():
                xml += ''.join(['<p x="%d" y="%d"/>\n'%(xl[i], yl[i]) for i in xrange(len(xl))])
            xml += '</lidar>\n'
        if self.particlecloud is not None:
            (xl, yl, tl) = self.particlecloud
            xml += '<particlecloud>\n'
            xml += ''.join(['<p x="%d" y="%d" t="%.3f"/>\n'%(xl[i], yl[i], tl[i]) for i in xrange(len(xl))])
            xml += '</particlecloud>\n'
       
        maps = self.available_maps()
        xml += '<maps current="%s">%s</maps>\n'%(DEFAULT_MAP, ''.join(['<m id="%s">%s</m>'%(m,m) for m in maps]))
        xml += '</map>\n'
        return xml

    def get_map_img_path(self, name):
        yfn = osp.join(roslib.packages.get_pkg_dir('maps'), name+".yaml")
        with open(yfn) as yfh:
            meta = yaml.load(yfh)
        return osp.join(osp.dirname(yfn), meta['image'])

def main():
    global proxy
    proxy = Proxy()
    try:
        httpd = HTTPServer(SERVER_ADDRESS, Server)
    except socket.error:
        rospy.logerr("Can't open web socket")
        return
    httpd.serve_forever()

if __name__=="__main__":
    main()

# EOF
