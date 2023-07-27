;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::SwitchController)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'SwitchController (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::SWITCHCONTROLLER")
  (make-package "STOCH3_MSGS::SWITCHCONTROLLER"))
(unless (find-package "STOCH3_MSGS::SWITCHCONTROLLERREQUEST")
  (make-package "STOCH3_MSGS::SWITCHCONTROLLERREQUEST"))
(unless (find-package "STOCH3_MSGS::SWITCHCONTROLLERRESPONSE")
  (make-package "STOCH3_MSGS::SWITCHCONTROLLERRESPONSE"))

(in-package "ROS")





(defclass stoch3_msgs::SwitchControllerRequest
  :super ros::object
  :slots (_name ))

(defmethod stoch3_msgs::SwitchControllerRequest
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass stoch3_msgs::SwitchControllerResponse
  :super ros::object
  :slots (_ok ))

(defmethod stoch3_msgs::SwitchControllerResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional __ok)
   (if __ok (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass stoch3_msgs::SwitchController
  :super ros::object
  :slots ())

(setf (get stoch3_msgs::SwitchController :md5sum-) "647e5c54b8d6468952d8d31f1efe96c0")
(setf (get stoch3_msgs::SwitchController :datatype-) "stoch3_msgs/SwitchController")
(setf (get stoch3_msgs::SwitchController :request) stoch3_msgs::SwitchControllerRequest)
(setf (get stoch3_msgs::SwitchController :response) stoch3_msgs::SwitchControllerResponse)

(defmethod stoch3_msgs::SwitchControllerRequest
  (:response () (instance stoch3_msgs::SwitchControllerResponse :init)))

(setf (get stoch3_msgs::SwitchControllerRequest :md5sum-) "647e5c54b8d6468952d8d31f1efe96c0")
(setf (get stoch3_msgs::SwitchControllerRequest :datatype-) "stoch3_msgs/SwitchControllerRequest")
(setf (get stoch3_msgs::SwitchControllerRequest :definition-)
      "string name
---
bool ok

")

(setf (get stoch3_msgs::SwitchControllerResponse :md5sum-) "647e5c54b8d6468952d8d31f1efe96c0")
(setf (get stoch3_msgs::SwitchControllerResponse :datatype-) "stoch3_msgs/SwitchControllerResponse")
(setf (get stoch3_msgs::SwitchControllerResponse :definition-)
      "string name
---
bool ok

")



(provide :stoch3_msgs/SwitchController "647e5c54b8d6468952d8d31f1efe96c0")


