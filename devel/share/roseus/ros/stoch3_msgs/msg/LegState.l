;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::LegState)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'LegState (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::LEGSTATE")
  (make-package "STOCH3_MSGS::LEGSTATE"))

(in-package "ROS")
;;//! \htmlinclude LegState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass stoch3_msgs::LegState
  :super ros::object
  :slots (_name _position _velocity _force _support_probability ))

(defmethod stoch3_msgs::LegState
  (:init
   (&key
    ((:name __name) "")
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:force __force) (instance geometry_msgs::Vector3 :init))
    ((:support_probability __support_probability) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _force __force)
   (setq _support_probability (float __support_probability))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
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
  (:support_probability
   (&optional __support_probability)
   (if __support_probability (setq _support_probability __support_probability)) _support_probability)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _force
    (send _force :serialization-length)
    ;; float64 _support_probability
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _force
       (send _force :serialize s)
     ;; float64 _support_probability
       (sys::poke _support_probability (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;; float64 _support_probability
     (setq _support_probability (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get stoch3_msgs::LegState :md5sum-) "796e284058360fe14a241d0db59e274d")
(setf (get stoch3_msgs::LegState :datatype-) "stoch3_msgs/LegState")
(setf (get stoch3_msgs::LegState :definition-)
      "string name
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 force
float64 support_probability        # [0, 1] , probability that the leg is a support leg.
                                   # Limit the value to range [0, 1] if it is outside the range.

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



(provide :stoch3_msgs/LegState "796e284058360fe14a241d0db59e274d")


