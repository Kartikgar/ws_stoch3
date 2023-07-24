;; Auto-generated. Do not edit!


(when (boundp 'stoch3_msgs::Command)
  (if (not (find-package "STOCH3_MSGS"))
    (make-package "STOCH3_MSGS"))
  (shadow 'Command (find-package "STOCH3_MSGS")))
(unless (find-package "STOCH3_MSGS::COMMAND")
  (make-package "STOCH3_MSGS::COMMAND"))
(unless (find-package "STOCH3_MSGS::COMMANDREQUEST")
  (make-package "STOCH3_MSGS::COMMANDREQUEST"))
(unless (find-package "STOCH3_MSGS::COMMANDRESPONSE")
  (make-package "STOCH3_MSGS::COMMANDRESPONSE"))

(in-package "ROS")





(defclass stoch3_msgs::CommandRequest
  :super ros::object
  :slots (_command ))

(defmethod stoch3_msgs::CommandRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass stoch3_msgs::CommandResponse
  :super ros::object
  :slots (_ok ))

(defmethod stoch3_msgs::CommandResponse
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

(defclass stoch3_msgs::Command
  :super ros::object
  :slots ())

(setf (get stoch3_msgs::Command :md5sum-) "8d2fe1ada9945234c69964dd9f502541")
(setf (get stoch3_msgs::Command :datatype-) "stoch3_msgs/Command")
(setf (get stoch3_msgs::Command :request) stoch3_msgs::CommandRequest)
(setf (get stoch3_msgs::Command :response) stoch3_msgs::CommandResponse)

(defmethod stoch3_msgs::CommandRequest
  (:response () (instance stoch3_msgs::CommandResponse :init)))

(setf (get stoch3_msgs::CommandRequest :md5sum-) "8d2fe1ada9945234c69964dd9f502541")
(setf (get stoch3_msgs::CommandRequest :datatype-) "stoch3_msgs/CommandRequest")
(setf (get stoch3_msgs::CommandRequest :definition-)
      "string command
---
bool ok

")

(setf (get stoch3_msgs::CommandResponse :md5sum-) "8d2fe1ada9945234c69964dd9f502541")
(setf (get stoch3_msgs::CommandResponse :datatype-) "stoch3_msgs/CommandResponse")
(setf (get stoch3_msgs::CommandResponse :definition-)
      "string command
---
bool ok

")



(provide :stoch3_msgs/Command "8d2fe1ada9945234c69964dd9f502541")


