;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::LegCommand)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'LegCommand (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::LEGCOMMAND")
  (make-package "STOCH3_MSGS::LEGCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude LegCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass stoch3_msgs::LegCommand
  :super ros::object
  :slots (_position _velocity _force _kp_scale _kd_scale ))

(defmethod stoch3_msgs::LegCommand
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:force __force) (instance geometry_msgs::Vector3 :init))
    ((:kp_scale __kp_scale) 0.0)
    ((:kd_scale __kd_scale) 0.0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _force __force)
   (setq _kp_scale (float __kp_scale))
   (setq _kd_scale (float __kd_scale))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:force
   (&rest __force)
   (if (keywordp (car __force))
       (send* _force __force)
     (progn
       (if __force (setq _force (car __force)))
       _force)))
  (:kp_scale
   (&optional __kp_scale)
   (if __kp_scale (setq _kp_scale __kp_scale)) _kp_scale)
  (:kd_scale
   (&optional __kd_scale)
   (if __kd_scale (setq _kd_scale __kd_scale)) _kd_scale)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _force
    (send _force :serialization-length)
    ;; float64 _kp_scale
    8
    ;; float64 _kd_scale
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _force
       (send _force :serialize s)
     ;; float64 _kp_scale
       (sys::poke _kp_scale (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _kd_scale
       (sys::poke _kd_scale (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;; float64 _kp_scale
     (setq _kp_scale (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _kd_scale
     (setq _kd_scale (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get stoch3_msgs::LegCommand :md5sum-) "ecee7b804ccb7dfdc74daea16cd92c3f")
(setf (get stoch3_msgs::LegCommand :datatype-) "stoch3_msgs/LegCommand")
(setf (get stoch3_msgs::LegCommand :definition-)
      "geometry_msgs/Vector3 position   # Position of foot
geometry_msgs/Vector3 velocity   # Velocity of foot
geometry_msgs/Vector3 force      # Force exerted by foot on environment
float64 kp_scale
float64 kd_scale

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
float64 z
")



(provide :stoch3_msgs/LegCommand "ecee7b804ccb7dfdc74daea16cd92c3f")


