;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::QuadrupedLegFeedback)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'QuadrupedLegFeedback (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::QUADRUPEDLEGFEEDBACK")
  (make-package "STOCH3_MSGS::QUADRUPEDLEGFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude QuadrupedLegFeedback.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::QuadrupedLegFeedback
  :super ros::object
  :slots (_header _name _foot_position _foot_velocity _foot_force _foot_pos_error _error_status ))

(defmethod stoch3_msgs::QuadrupedLegFeedback
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:foot_position __foot_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:foot_velocity __foot_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:foot_force __foot_force) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:foot_pos_error __foot_pos_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:error_status __error_status) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _foot_position __foot_position)
   (setq _foot_velocity __foot_velocity)
   (setq _foot_force __foot_force)
   (setq _foot_pos_error __foot_pos_error)
   (setq _error_status __error_status)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:foot_position
   (&optional __foot_position)
   (if __foot_position (setq _foot_position __foot_position)) _foot_position)
  (:foot_velocity
   (&optional __foot_velocity)
   (if __foot_velocity (setq _foot_velocity __foot_velocity)) _foot_velocity)
  (:foot_force
   (&optional __foot_force)
   (if __foot_force (setq _foot_force __foot_force)) _foot_force)
  (:foot_pos_error
   (&optional __foot_pos_error)
   (if __foot_pos_error (setq _foot_pos_error __foot_pos_error)) _foot_pos_error)
  (:error_status
   (&optional __error_status)
   (if __error_status (setq _error_status __error_status)) _error_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; float64[] _foot_position
    (* 8    (length _foot_position)) 4
    ;; float64[] _foot_velocity
    (* 8    (length _foot_velocity)) 4
    ;; float64[] _foot_force
    (* 8    (length _foot_force)) 4
    ;; float64[] _foot_pos_error
    (* 8    (length _foot_pos_error)) 4
    ;; bool _error_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _foot_position
     (write-long (length _foot_position) s)
     (dotimes (i (length _foot_position))
       (sys::poke (elt _foot_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _foot_velocity
     (write-long (length _foot_velocity) s)
     (dotimes (i (length _foot_velocity))
       (sys::poke (elt _foot_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _foot_force
     (write-long (length _foot_force) s)
     (dotimes (i (length _foot_force))
       (sys::poke (elt _foot_force i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _foot_pos_error
     (write-long (length _foot_pos_error) s)
     (dotimes (i (length _foot_pos_error))
       (sys::poke (elt _foot_pos_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _error_status
       (if _error_status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _foot_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _foot_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _foot_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _foot_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _foot_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _foot_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _foot_force
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _foot_force (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _foot_force i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _foot_pos_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _foot_pos_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _foot_pos_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _error_status
     (setq _error_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get stoch3_msgs::QuadrupedLegFeedback :md5sum-) "7475d2278216c40ff50e977798003eb3")
(setf (get stoch3_msgs::QuadrupedLegFeedback :datatype-) "stoch3_msgs/QuadrupedLegFeedback")
(setf (get stoch3_msgs::QuadrupedLegFeedback :definition-)
      "Header    header
string[]  name
float64[] foot_position
float64[] foot_velocity
float64[] foot_force
float64[] foot_pos_error
bool error_status


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



(provide :stoch3_msgs/QuadrupedLegFeedback "7475d2278216c40ff50e977798003eb3")


