;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::QuadrupedStance)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'QuadrupedStance (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::QUADRUPEDSTANCE")
  (make-package "PRONTO_MSGS::QUADRUPEDSTANCE"))

(in-package "ROS")
;;//! \htmlinclude QuadrupedStance.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::QuadrupedStance
  :super ros::object
  :slots (_header _lf _rf _lh _rh ))

(defmethod pronto_msgs::QuadrupedStance
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lf __lf) 0.0)
    ((:rf __rf) 0.0)
    ((:lh __lh) 0.0)
    ((:rh __rh) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _lf (float __lf))
   (setq _rf (float __rf))
   (setq _lh (float __lh))
   (setq _rh (float __rh))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lf
   (&optional __lf)
   (if __lf (setq _lf __lf)) _lf)
  (:rf
   (&optional __rf)
   (if __rf (setq _rf __rf)) _rf)
  (:lh
   (&optional __lh)
   (if __lh (setq _lh __lh)) _lh)
  (:rh
   (&optional __rh)
   (if __rh (setq _rh __rh)) _rh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _lf
    4
    ;; float32 _rf
    4
    ;; float32 _lh
    4
    ;; float32 _rh
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _lf
       (sys::poke _lf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rf
       (sys::poke _rf (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _lh
       (sys::poke _lh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rh
       (sys::poke _rh (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _lf
     (setq _lf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rf
     (setq _rf (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _lh
     (setq _lh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rh
     (setq _rh (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pronto_msgs::QuadrupedStance :md5sum-) "6e165cd35d6f3eecca48a2a9f715fd5f")
(setf (get pronto_msgs::QuadrupedStance :datatype-) "pronto_msgs/QuadrupedStance")
(setf (get pronto_msgs::QuadrupedStance :definition-)
      "Header header
float32 lf
float32 rf
float32 lh
float32 rh


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



(provide :pronto_msgs/QuadrupedStance "6e165cd35d6f3eecca48a2a9f715fd5f")


