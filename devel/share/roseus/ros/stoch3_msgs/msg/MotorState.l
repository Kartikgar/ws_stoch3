;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::MotorState)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'MotorState (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::MOTORSTATE")
  (make-package "STOCH3_MSGS::MOTORSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass stoch3_msgs::MotorState
  :super ros::object
  :slots (_header _name _mode _fault _temperature _voltage ))

(defmethod stoch3_msgs::MotorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:mode __mode) (make-array 0 :initial-element 0 :element-type :integer))
    ((:fault __fault) (make-array 0 :initial-element 0 :element-type :integer))
    ((:temperature __temperature) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:voltage __voltage) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name __name)
   (setq _mode __mode)
   (setq _fault __fault)
   (setq _temperature __temperature)
   (setq _voltage __voltage)
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
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:fault
   (&optional __fault)
   (if __fault (setq _fault __fault)) _fault)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:voltage
   (&optional __voltage)
   (if __voltage (setq _voltage __voltage)) _voltage)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; int16[] _mode
    (* 2    (length _mode)) 4
    ;; int8[] _fault
    (* 1    (length _fault)) 4
    ;; float32[] _temperature
    (* 4    (length _temperature)) 4
    ;; float32[] _voltage
    (* 4    (length _voltage)) 4
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
     ;; int16[] _mode
     (write-long (length _mode) s)
     (dotimes (i (length _mode))
       (write-word (elt _mode i) s)
       )
     ;; int8[] _fault
     (write-long (length _fault) s)
     (dotimes (i (length _fault))
       (write-byte (elt _fault i) s)
       )
     ;; float32[] _temperature
     (write-long (length _temperature) s)
     (dotimes (i (length _temperature))
       (sys::poke (elt _temperature i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _voltage
     (write-long (length _voltage) s)
     (dotimes (i (length _voltage))
       (sys::poke (elt _voltage i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
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
   ;; int16[] _mode
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _mode (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _mode i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int8[] _fault
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _fault (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _fault i) (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> (elt _fault i) 127) (setf (elt _fault i) (- (elt _fault i) 256)))
     ))
   ;; float32[] _temperature
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temperature (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temperature i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _voltage
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _voltage (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _voltage i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get stoch3_msgs::MotorState :md5sum-) "2b6a11e26d15449b995d7f315380d304")
(setf (get stoch3_msgs::MotorState :datatype-) "stoch3_msgs/MotorState")
(setf (get stoch3_msgs::MotorState :definition-)
      "std_msgs/Header header
string[] name
int16[] mode
int8[] fault
float32[] temperature
float32[] voltage

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



(provide :stoch3_msgs/MotorState "2b6a11e26d15449b995d7f315380d304")


