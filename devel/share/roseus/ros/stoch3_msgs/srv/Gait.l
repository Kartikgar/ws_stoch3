;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::Gait)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'Gait (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::GAIT")
  (make-package "STOCH3_MSGS::GAIT"))
(unless (find-package "STOCH3_MSGS::GAITREQUEST")
  (make-package "STOCH3_MSGS::GAITREQUEST"))
(unless (find-package "STOCH3_MSGS::GAITRESPONSE")
  (make-package "STOCH3_MSGS::GAITRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass stoch3_msgs::GaitRequest
  :super ros::object
  :slots (_step_frequency _swing_height _stance_height _max_vel ))

(defmethod stoch3_msgs::GaitRequest
  (:init
   (&key
    ((:step_frequency __step_frequency) 0.0)
    ((:swing_height __swing_height) 0.0)
    ((:stance_height __stance_height) 0.0)
    ((:max_vel __max_vel) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _step_frequency (float __step_frequency))
   (setq _swing_height (float __swing_height))
   (setq _stance_height (float __stance_height))
   (setq _max_vel __max_vel)
   self)
  (:step_frequency
   (&optional __step_frequency)
   (if __step_frequency (setq _step_frequency __step_frequency)) _step_frequency)
  (:swing_height
   (&optional __swing_height)
   (if __swing_height (setq _swing_height __swing_height)) _swing_height)
  (:stance_height
   (&optional __stance_height)
   (if __stance_height (setq _stance_height __stance_height)) _stance_height)
  (:max_vel
   (&rest __max_vel)
   (if (keywordp (car __max_vel))
       (send* _max_vel __max_vel)
     (progn
       (if __max_vel (setq _max_vel (car __max_vel)))
       _max_vel)))
  (:serialization-length
   ()
   (+
    ;; float64 _step_frequency
    8
    ;; float64 _swing_height
    8
    ;; float64 _stance_height
    8
    ;; geometry_msgs/Twist _max_vel
    (send _max_vel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _step_frequency
       (sys::poke _step_frequency (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _swing_height
       (sys::poke _swing_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _stance_height
       (sys::poke _stance_height (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Twist _max_vel
       (send _max_vel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _step_frequency
     (setq _step_frequency (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _swing_height
     (setq _swing_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _stance_height
     (setq _stance_height (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Twist _max_vel
     (send _max_vel :deserialize buf ptr-) (incf ptr- (send _max_vel :serialization-length))
   ;;
   self)
  )

(defclass stoch3_msgs::GaitResponse
  :super ros::object
  :slots (_ok ))

(defmethod stoch3_msgs::GaitResponse
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

(defclass stoch3_msgs::Gait
  :super ros::object
  :slots ())

(setf (get stoch3_msgs::Gait :md5sum-) "e17f1b3fe4a600b5b71bcddef43f4905")
(setf (get stoch3_msgs::Gait :datatype-) "stoch3_msgs/Gait")
(setf (get stoch3_msgs::Gait :request) stoch3_msgs::GaitRequest)
(setf (get stoch3_msgs::Gait :response) stoch3_msgs::GaitResponse)

(defmethod stoch3_msgs::GaitRequest
  (:response () (instance stoch3_msgs::GaitResponse :init)))

(setf (get stoch3_msgs::GaitRequest :md5sum-) "e17f1b3fe4a600b5b71bcddef43f4905")
(setf (get stoch3_msgs::GaitRequest :datatype-) "stoch3_msgs/GaitRequest")
(setf (get stoch3_msgs::GaitRequest :definition-)
      "float64 step_frequency
float64 swing_height
float64 stance_height
geometry_msgs/Twist max_vel

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool ok

")

(setf (get stoch3_msgs::GaitResponse :md5sum-) "e17f1b3fe4a600b5b71bcddef43f4905")
(setf (get stoch3_msgs::GaitResponse :datatype-) "stoch3_msgs/GaitResponse")
(setf (get stoch3_msgs::GaitResponse :definition-)
      "float64 step_frequency
float64 swing_height
float64 stance_height
geometry_msgs/Twist max_vel

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
---
bool ok

")



(provide :stoch3_msgs/Gait "e17f1b3fe4a600b5b71bcddef43f4905")


