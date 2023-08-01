;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::GPSData)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'GPSData (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::GPSDATA")
  (make-package "PRONTO_MSGS::GPSDATA"))

(in-package "ROS")
;;//! \htmlinclude GPSData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::GPSData
  :super ros::object
  :slots (_header _utime _gps_lock _longitude _latitude _elev _horizontal_accuracy _vertical_accuracy _num_satellites _speed _heading _xyz_pos _gps_time ))

(defmethod pronto_msgs::GPSData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:utime __utime) 0)
    ((:gps_lock __gps_lock) 0)
    ((:longitude __longitude) 0.0)
    ((:latitude __latitude) 0.0)
    ((:elev __elev) 0.0)
    ((:horizontal_accuracy __horizontal_accuracy) 0.0)
    ((:vertical_accuracy __vertical_accuracy) 0.0)
    ((:num_satellites __num_satellites) 0)
    ((:speed __speed) 0.0)
    ((:heading __heading) 0.0)
    ((:xyz_pos __xyz_pos) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:gps_time __gps_time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _utime (round __utime))
   (setq _gps_lock (round __gps_lock))
   (setq _longitude (float __longitude))
   (setq _latitude (float __latitude))
   (setq _elev (float __elev))
   (setq _horizontal_accuracy (float __horizontal_accuracy))
   (setq _vertical_accuracy (float __vertical_accuracy))
   (setq _num_satellites (round __num_satellites))
   (setq _speed (float __speed))
   (setq _heading (float __heading))
   (setq _xyz_pos __xyz_pos)
   (setq _gps_time (float __gps_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:utime
   (&optional __utime)
   (if __utime (setq _utime __utime)) _utime)
  (:gps_lock
   (&optional __gps_lock)
   (if __gps_lock (setq _gps_lock __gps_lock)) _gps_lock)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:elev
   (&optional __elev)
   (if __elev (setq _elev __elev)) _elev)
  (:horizontal_accuracy
   (&optional __horizontal_accuracy)
   (if __horizontal_accuracy (setq _horizontal_accuracy __horizontal_accuracy)) _horizontal_accuracy)
  (:vertical_accuracy
   (&optional __vertical_accuracy)
   (if __vertical_accuracy (setq _vertical_accuracy __vertical_accuracy)) _vertical_accuracy)
  (:num_satellites
   (&optional __num_satellites)
   (if __num_satellites (setq _num_satellites __num_satellites)) _num_satellites)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:xyz_pos
   (&optional __xyz_pos)
   (if __xyz_pos (setq _xyz_pos __xyz_pos)) _xyz_pos)
  (:gps_time
   (&optional __gps_time)
   (if __gps_time (setq _gps_time __gps_time)) _gps_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _utime
    8
    ;; int32 _gps_lock
    4
    ;; float64 _longitude
    8
    ;; float64 _latitude
    8
    ;; float64 _elev
    8
    ;; float64 _horizontal_accuracy
    8
    ;; float64 _vertical_accuracy
    8
    ;; uint32 _num_satellites
    4
    ;; float64 _speed
    8
    ;; float64 _heading
    8
    ;; float64[3] _xyz_pos
    (* 8    3)
    ;; float64 _gps_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint64 _utime
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _utime (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _utime) (= (length (_utime . bv)) 2)) ;; bignum
              (write-long (ash (elt (_utime . bv) 0) 0) s)
              (write-long (ash (elt (_utime . bv) 1) -1) s))
             ((and (class _utime) (= (length (_utime . bv)) 1)) ;; big1
              (write-long (elt (_utime . bv) 0) s)
              (write-long (if (>= _utime 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _utime s)(write-long (if (>= _utime 0) 0 #xffffffff) s)))
     ;; int32 _gps_lock
       (write-long _gps_lock s)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _elev
       (sys::poke _elev (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _horizontal_accuracy
       (sys::poke _horizontal_accuracy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vertical_accuracy
       (sys::poke _vertical_accuracy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint32 _num_satellites
       (write-long _num_satellites s)
     ;; float64 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _xyz_pos
     (dotimes (i 3)
       (sys::poke (elt _xyz_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _gps_time
       (sys::poke _gps_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint64 _utime
     
#+(or :alpha :irix6 :x86_64)
      (setf _utime (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _utime (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int32 _gps_lock
     (setq _gps_lock (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _elev
     (setq _elev (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _horizontal_accuracy
     (setq _horizontal_accuracy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vertical_accuracy
     (setq _vertical_accuracy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint32 _num_satellites
     (setq _num_satellites (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _speed
     (setq _speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _xyz_pos
   (dotimes (i (length _xyz_pos))
     (setf (elt _xyz_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _gps_time
     (setq _gps_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pronto_msgs::GPSData :md5sum-) "79674b2d1f8526159c0504accff3f259")
(setf (get pronto_msgs::GPSData :datatype-) "pronto_msgs/GPSData")
(setf (get pronto_msgs::GPSData :definition-)
      "Header header
uint64 utime
int32 gps_lock
float64 longitude
float64 latitude
float64 elev
float64 horizontal_accuracy
float64 vertical_accuracy
uint32 num_satellites
float64 speed
float64 heading
float64[3] xyz_pos
float64 gps_time


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :pronto_msgs/GPSData "79674b2d1f8526159c0504accff3f259")


