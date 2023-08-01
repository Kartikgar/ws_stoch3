;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::ControllerFootContact)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'ControllerFootContact (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::CONTROLLERFOOTCONTACT")
  (make-package "PRONTO_MSGS::CONTROLLERFOOTCONTACT"))

(in-package "ROS")
;;//! \htmlinclude ControllerFootContact.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::ControllerFootContact
  :super ros::object
  :slots (_header _num_right_foot_contacts _right_foot_contacts _num_left_foot_contacts _left_foot_contacts ))

(defmethod pronto_msgs::ControllerFootContact
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_right_foot_contacts __num_right_foot_contacts) 0)
    ((:right_foot_contacts __right_foot_contacts) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:num_left_foot_contacts __num_left_foot_contacts) 0)
    ((:left_foot_contacts __left_foot_contacts) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_right_foot_contacts (round __num_right_foot_contacts))
   (setq _right_foot_contacts __right_foot_contacts)
   (setq _num_left_foot_contacts (round __num_left_foot_contacts))
   (setq _left_foot_contacts __left_foot_contacts)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_right_foot_contacts
   (&optional __num_right_foot_contacts)
   (if __num_right_foot_contacts (setq _num_right_foot_contacts __num_right_foot_contacts)) _num_right_foot_contacts)
  (:right_foot_contacts
   (&optional __right_foot_contacts)
   (if __right_foot_contacts (setq _right_foot_contacts __right_foot_contacts)) _right_foot_contacts)
  (:num_left_foot_contacts
   (&optional __num_left_foot_contacts)
   (if __num_left_foot_contacts (setq _num_left_foot_contacts __num_left_foot_contacts)) _num_left_foot_contacts)
  (:left_foot_contacts
   (&optional __left_foot_contacts)
   (if __left_foot_contacts (setq _left_foot_contacts __left_foot_contacts)) _left_foot_contacts)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _num_right_foot_contacts
    4
    ;; float64[] _right_foot_contacts
    (* 8    (length _right_foot_contacts)) 4
    ;; int32 _num_left_foot_contacts
    4
    ;; float64[] _left_foot_contacts
    (* 8    (length _left_foot_contacts)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _num_right_foot_contacts
       (write-long _num_right_foot_contacts s)
     ;; float64[] _right_foot_contacts
     (write-long (length _right_foot_contacts) s)
     (dotimes (i (length _right_foot_contacts))
       (sys::poke (elt _right_foot_contacts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _num_left_foot_contacts
       (write-long _num_left_foot_contacts s)
     ;; float64[] _left_foot_contacts
     (write-long (length _left_foot_contacts) s)
     (dotimes (i (length _left_foot_contacts))
       (sys::poke (elt _left_foot_contacts i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _num_right_foot_contacts
     (setq _num_right_foot_contacts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _right_foot_contacts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _right_foot_contacts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _right_foot_contacts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _num_left_foot_contacts
     (setq _num_left_foot_contacts (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64[] _left_foot_contacts
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _left_foot_contacts (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _left_foot_contacts i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get pronto_msgs::ControllerFootContact :md5sum-) "ac4086dd5e9ba23a62d9963e591cc896")
(setf (get pronto_msgs::ControllerFootContact :datatype-) "pronto_msgs/ControllerFootContact")
(setf (get pronto_msgs::ControllerFootContact :definition-)
      "Header header
int32 num_right_foot_contacts

# Porting from LCM definition:
# double right_foot_contacts[3][num_right_foot_contacts]
float64[] right_foot_contacts

int32 num_left_foot_contacts

# Porting from LCM definition:
# double left_foot_contacts[3][num_left_foot_contacts];
float64[] left_foot_contacts

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



(provide :pronto_msgs/ControllerFootContact "ac4086dd5e9ba23a62d9963e591cc896")


