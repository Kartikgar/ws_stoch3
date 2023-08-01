;; Auto-generated. Do not edit!


(when (boundp 'pronto_msgs::IndexedMeasurement)
  (if (not (find-package "PRONTO_MSGS"))
    (make-package "PRONTO_MSGS"))
  (shadow 'IndexedMeasurement (find-package "PRONTO_MSGS")))
(unless (find-package "PRONTO_MSGS::INDEXEDMEASUREMENT")
  (make-package "PRONTO_MSGS::INDEXEDMEASUREMENT"))

(in-package "ROS")
;;//! \htmlinclude IndexedMeasurement.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pronto_msgs::IndexedMeasurement
  :super ros::object
  :slots (_header _utime _state_utime _z_effective _z_indices _R_effective ))

(defmethod pronto_msgs::IndexedMeasurement
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:utime __utime) 0)
    ((:state_utime __state_utime) 0)
    ((:z_effective __z_effective) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:z_indices __z_indices) (make-array 0 :initial-element 0 :element-type :integer))
    ((:R_effective __R_effective) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _utime (round __utime))
   (setq _state_utime (round __state_utime))
   (setq _z_effective __z_effective)
   (setq _z_indices __z_indices)
   (setq _R_effective __R_effective)
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
  (:state_utime
   (&optional __state_utime)
   (if __state_utime (setq _state_utime __state_utime)) _state_utime)
  (:z_effective
   (&optional __z_effective)
   (if __z_effective (setq _z_effective __z_effective)) _z_effective)
  (:z_indices
   (&optional __z_indices)
   (if __z_indices (setq _z_indices __z_indices)) _z_indices)
  (:R_effective
   (&optional __R_effective)
   (if __R_effective (setq _R_effective __R_effective)) _R_effective)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint64 _utime
    8
    ;; uint64 _state_utime
    8
    ;; float64[] _z_effective
    (* 8    (length _z_effective)) 4
    ;; int32[] _z_indices
    (* 4    (length _z_indices)) 4
    ;; float64[] _R_effective
    (* 8    (length _R_effective)) 4
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
     ;; uint64 _state_utime
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _state_utime (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _state_utime) (= (length (_state_utime . bv)) 2)) ;; bignum
              (write-long (ash (elt (_state_utime . bv) 0) 0) s)
              (write-long (ash (elt (_state_utime . bv) 1) -1) s))
             ((and (class _state_utime) (= (length (_state_utime . bv)) 1)) ;; big1
              (write-long (elt (_state_utime . bv) 0) s)
              (write-long (if (>= _state_utime 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _state_utime s)(write-long (if (>= _state_utime 0) 0 #xffffffff) s)))
     ;; float64[] _z_effective
     (write-long (length _z_effective) s)
     (dotimes (i (length _z_effective))
       (sys::poke (elt _z_effective i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32[] _z_indices
     (write-long (length _z_indices) s)
     (dotimes (i (length _z_indices))
       (write-long (elt _z_indices i) s)
       )
     ;; float64[] _R_effective
     (write-long (length _R_effective) s)
     (dotimes (i (length _R_effective))
       (sys::poke (elt _R_effective i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
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
   ;; uint64 _state_utime
     
#+(or :alpha :irix6 :x86_64)
      (setf _state_utime (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _state_utime (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64[] _z_effective
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z_effective (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _z_effective i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32[] _z_indices
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _z_indices (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _z_indices i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; float64[] _R_effective
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _R_effective (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _R_effective i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get pronto_msgs::IndexedMeasurement :md5sum-) "5a9a7b4bda2dae60a94faa7c688f6e20")
(setf (get pronto_msgs::IndexedMeasurement :datatype-) "pronto_msgs/IndexedMeasurement")
(setf (get pronto_msgs::IndexedMeasurement :definition-)
      "Header header
uint64 utime
uint64 state_utime
float64[] z_effective
int32[] z_indices
float64[] R_effective


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



(provide :pronto_msgs/IndexedMeasurement "5a9a7b4bda2dae60a94faa7c688f6e20")


