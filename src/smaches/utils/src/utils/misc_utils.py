# -*- coding: utf-8 -*-
import imports

#Class to get XTION camera info (head)
class RGBD():
    def __init__(self):
        self._br = tf.TransformBroadcaster()
        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._points_data = None
        self._image_data = None
        self._h_image = None
        self._region = None
        self._h_min = 0
        self._h_max = 0
        self._xyz = [0, 0, 0]
        self._frame_name = None

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = \
        self._points_data['rgb'].view((np.uint8, 4))[..., [2, 1, 0]]
        hsv_image = cv2.cvtColor(self._image_data, cv2.COLOR_RGB2HSV_FULL)
        self._h_image = hsv_image[..., 0]
        self._region = \
        (self._h_image > self._h_min) & (self._h_image < self._h_max)
        if not np.any(self._region):
            return
            
        (y_idx, x_idx) = np.where(self._region)
        x = np.average(self._points_data['x'][y_idx, x_idx])
        y = np.average(self._points_data['y'][y_idx, x_idx])
        z = np.average(self._points_data['z'][y_idx, x_idx])
        self._xyz = [y, x, z]
        if self._frame_name is None:
            return

        self._br.sendTransform(
        (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
        rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
        self._frame_name,
        msg.header.frame_id)

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

    def get_h_image(self):
        return self._h_image

    def get_region(self):
        return self._region

    def get_xyz(self):
        return self._xyz

    def set_h(self, h_min, h_max):
        self._h_min = h_min
        self._h_max = h_max

    def set_coordinate_name(self, name):
        self._frame_name = name
        
#Color segmentator
    def color_segmentator(self, color = "orange"):
        image = self.get_image()
        if(color == "blue"):
            lower_threshold = (100,120,100)
            upper_threshold = (150,220,240)
        else:
            lower_threshold = (102,95,97)
            upper_threshold = (115,255,255)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img_hsv, lower_threshold, upper_threshold)
        res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
        pos = []
        pixels = cv2.findNonZero(mask)
        pixels = list(cv2.mean(pixels))
        pos.append(pixels[:2])
        return pos

#Class to get hand camera images(RGB)
class HAND_RGB():
    def __init__(self):
        self.cam_sub = rospy.Subscriber(
            '/hsrb/hand_camera/image_raw',
            ImageMsg, self._callback)
        self._points_data = None
        self._image_data = None
        
    def _callback(self, msg):
        self._image_data = ros_numpy.numpify(msg)
        
    def get_image(self):
        image = self._image_data
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
    
#Color segmentator
    def color_segmentator(self, color = "orange"):
        image = self.get_image()
        if(color == "blue"):
            lower_threshold = (100,120,100)
            upper_threshold = (150,220,240)
        else:
            # lower_threshold = (102,95,97)
            # upper_threshold = (115,255,255)
            lower_threshold = (105,130,100)
            upper_threshold = (115,225,255)
        img_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img_hsv, lower_threshold, upper_threshold)
        res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
        pos = []
        pixels = cv2.findNonZero(mask)
        pixels = list(cv2.mean(pixels))
        pos.append(pixels[:2])
        return pos
class TF_MANAGER():
    def __init__(self):
        self._tfbuff = tf2.Buffer()
        self._lis = tf2.TransformListener(self._tfbuff)
        self._tf_static_broad = tf2.StaticTransformBroadcaster()
        self._broad = tf2.TransformBroadcaster()

    def _fillMsg(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = ref
        TS.child_frame_id = point_name
        TS.transform.translation = Point(pos)
        TS.transform.rotation = Quaternion(rot)
        return TS

    def pub_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        dinamic_ts = self._fillMsg(pos, rot, point_name, ref)
        self._broad.sendTransform(dinamic_ts)

    def pub_static_tf(self, pos = [0,0,0], rot = [0,0,0,1] ,point_name ='', ref="map"):
        static_ts = self._fillMsg(pos, rot, point_name, ref)
        self._tf_static_broad.sendTransform(static_ts)

    def change_ref_frame_tf(self, point_name = '', new_frame = 'map'):
        try:
            traf = self._tfbuff.lookup_transform(new_frame, point_name, rospy.Time(0))
            translation, rotational = self.tf2_obj_2_arr(traf)
            self.pub_static_tf(pos = translation, rot = rotational, point_name = point_name, ref = new_frame)
            return True
        except:
            return False

    def getTF(self, target_frame='', ref_frame='map'):
        try:
            tf = self._tfbuff.lookup_transform(ref_frame, target_frame, rospy.Time(0))
            return self.tf2_obj_2_arr(tf)
        except:
            return [False,False]

    def tf2_obj_2_arr(self, transf):
        pos = []
        pos.append(transf.transform.translation.x)
        pos.append(transf.transform.translation.y)
        pos.append(transf.transform.translation.z)
    
        rot = []
        rot.append(transf.transform.rotation.x)
        rot.append(transf.transform.rotation.y)
        rot.append(transf.transform.rotation.z)
        rot.append(transf.transform.rotation.w)

        return [pos, rot]


class LineDetector():
    def __init__(self):
        self.scan_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, self._scan_callback)
        # self.scan_sub = rospy.Subscriber("/hsrb/base_scan/fix", LaserScan, self._scan_callback)
        self._result = False
    def _scan_callback(self, scan_msg):
        # Obtener las lecturas de los puntos del LIDAR
        ranges = scan_msg.ranges

         # Tomar solo las muestras centrales
        num_samples = len(ranges)
        num_central_samples = int(num_samples * 0.05) # valor ajustable
        start_index = int((num_samples - num_central_samples) / 2)
        central_ranges = ranges[start_index : start_index + num_central_samples]

        # Calcular la media de las lecturas
        mean = sum(central_ranges) / len(central_ranges)

        # Calcular la desviación estándar de las lecturas
        variance = sum((x - mean)**2 for x in central_ranges) / len(central_ranges)
        std_dev = variance**0.5

        # Verificar si las lecturas se aproximan a ser una línea
        if std_dev < 0.5: # valor ajustable
            # rospy.loginfo("Posible línea enfrente")
            self._result = True
        else:
            # rospy.loginfo("No se encontró una línea")
            self._result = False
    def line_found(self):
        return self._result

talk_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)   ### PUMAS NAV ACTION LIB

def talk(msg, time_out = 5):
    # talker = rospy.Publisher('/talk_request_action/goal', TalkRequestActionGoal, queue_size=10)
    voice = TalkRequestActionGoal()
    voice.goal.data.interrupting = True
    voice.goal.data.queueing = True
    voice.goal.data.language = 1
    voice.goal.data.sentence = msg
    talk_client.send_goal(voice.goal)
    talk_client.wait_for_result(timeout=rospy.Duration(time_out))