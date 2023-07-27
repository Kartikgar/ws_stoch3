;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::ControllerSupervisorState)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'ControllerSupervisorState (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATE")
  (make-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATE"))
(unless (find-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATEREQUEST")
  (make-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATEREQUEST"))
(unless (find-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATERESPONSE")
  (make-package "STOCH3_MSGS::CONTROLLERSUPERVISORSTATERESPONSE"))

(in-package "ROS")





(defclass stoch3_msgs::ControllerSupervisorStateRequest
  :super ros::object
  :slots ())

(defmethod stoch3_msgs::ControllerSupervisorStateRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass stoch3_msgs::ControllerSupervisorStateResponse
  :super ros::object
  :slots (_controller_name _seq_num ))

(defmethod stoch3_msgs::ControllerSupervisorStateResponse
  (:init
   (&key
    ((:controller_name __controller_name) "")
    ((:seq_num __seq_num) 0)
    )
   (send-super :init)
   (setq _controller_name (string __controller_name))
   (setq _seq_num (round __seq_num))
   self)
  (:controller_name
   (&optional __controller_name)
   (if __controller_name (setq _controller_name __controller_name)) _controller_name)
  (:seq_num
   (&optional __seq_num)
   (if __seq_num (setq _seq_num __seq_num)) _seq_num)
  (:serialization-length
   ()
   (+
    ;; string _controller_name
    4 (length _controller_name)
    ;; int64 _seq_num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _controller_name
       (write-long (length _controller_name) s) (princ _controller_name s)
     ;; int64 _seq_num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _seq_num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _seq_num) (= (length (_seq_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_seq_num . bv) 0) 0) s)
              (write-long (ash (elt (_seq_num . bv) 1) -1) s))
             ((and (class _seq_num) (= (length (_seq_num . bv)) 1)) ;; big1
              (write-long (elt (_seq_num . bv) 0) s)
              (write-long (if (>= _seq_num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _seq_num s)(write-long (if (>= _seq_num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _controller_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _seq_num
#+(or :alpha :irix6 :x86_64)
      (setf _seq_num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _seq_num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass stoch3_msgs::ControllerSupervisorState
  :super ros::object
  :slots ())

(setf (get stoch3_msgs::ControllerSupervisorState :md5sum-) "61f9a8ff34ea0f4986898104d7e20d25")
(setf (get stoch3_msgs::ControllerSupervisorState :datatype-) "stoch3_msgs/ControllerSupervisorState")
(setf (get stoch3_msgs::ControllerSupervisorState :request) stoch3_msgs::ControllerSupervisorStateRequest)
(setf (get stoch3_msgs::ControllerSupervisorState :response) stoch3_msgs::ControllerSupervisorStateResponse)

(defmethod stoch3_msgs::ControllerSupervisorStateRequest
  (:response () (instance stoch3_msgs::ControllerSupervisorStateResponse :init)))

(setf (get stoch3_msgs::ControllerSupervisorStateRequest :md5sum-) "61f9a8ff34ea0f4986898104d7e20d25")
(setf (get stoch3_msgs::ControllerSupervisorStateRequest :datatype-) "stoch3_msgs/ControllerSupervisorStateRequest")
(setf (get stoch3_msgs::ControllerSupervisorStateRequest :definition-)
      "---
string controller_name # Name of the current active controller
int64  seq_num         # Sequence numer of the current active controller
                       # (i.e, number of times the controller update function called since it became active)

")

(setf (get stoch3_msgs::ControllerSupervisorStateResponse :md5sum-) "61f9a8ff34ea0f4986898104d7e20d25")
(setf (get stoch3_msgs::ControllerSupervisorStateResponse :datatype-) "stoch3_msgs/ControllerSupervisorStateResponse")
(setf (get stoch3_msgs::ControllerSupervisorStateResponse :definition-)
      "---
string controller_name # Name of the current active controller
int64  seq_num         # Sequence numer of the current active controller
                       # (i.e, number of times the controller update function called since it became active)

")



(provide :stoch3_msgs/ControllerSupervisorState "61f9a8ff34ea0f4986898104d7e20d25")


