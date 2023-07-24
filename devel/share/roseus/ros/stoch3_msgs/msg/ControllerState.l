;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::ControllerState)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'ControllerState (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::CONTROLLERSTATE")
  (make-package "STOCH3_MSGS::CONTROLLERSTATE"))

(in-package "ROS")
;;//! \htmlinclude ControllerState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::ControllerState
  :super ros::object
  :slots (_header _joint_names _act_position _cmd_position _position_error _shifts ))

(defmethod stoch3_msgs::ControllerState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:act_position __act_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmd_position __cmd_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:position_error __position_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:shifts __shifts) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _act_position __act_position)
   (setq _cmd_position __cmd_position)
   (setq _position_error __position_error)
   (setq _shifts __shifts)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:act_position
   (&optional __act_position)
   (if __act_position (setq _act_position __act_position)) _act_position)
  (:cmd_position
   (&optional __cmd_position)
   (if __cmd_position (setq _cmd_position __cmd_position)) _cmd_position)
  (:position_error
   (&optional __position_error)
   (if __position_error (setq _position_error __position_error)) _position_error)
  (:shifts
   (&optional __shifts)
   (if __shifts (setq _shifts __shifts)) _shifts)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; float64[] _act_position
    (* 8    (length _act_position)) 4
    ;; float64[] _cmd_position
    (* 8    (length _cmd_position)) 4
    ;; float64[] _position_error
    (* 8    (length _position_error)) 4
    ;; float64[] _shifts
    (* 8    (length _shifts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _act_position
     (write-long (length _act_position) s)
     (dotimes (i (length _act_position))
       (sys::poke (elt _act_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmd_position
     (write-long (length _cmd_position) s)
     (dotimes (i (length _cmd_position))
       (sys::poke (elt _cmd_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _position_error
     (write-long (length _position_error) s)
     (dotimes (i (length _position_error))
       (sys::poke (elt _position_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _shifts
     (write-long (length _shifts) s)
     (dotimes (i (length _shifts))
       (sys::poke (elt _shifts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _act_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _act_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _act_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmd_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _position_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _shifts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _shifts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _shifts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get stoch3_msgs::ControllerState :md5sum-) "d913fd7c30d2891f22fc139634144c49")
(setf (get stoch3_msgs::ControllerState :datatype-) "stoch3_msgs/ControllerState")
(setf (get stoch3_msgs::ControllerState :definition-)
      "std_msgs/Header header
string[] joint_names
float64[] act_position # Actual position
float64[] cmd_position # Commanded position
float64[] position_error # Position error
float64[] shifts # leg shifts from linear policy

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



(provide :stoch3_msgs/ControllerState "d913fd7c30d2891f22fc139634144c49")


