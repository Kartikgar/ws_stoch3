;; Auto-generated. Do not edit!


(when (boundp 'fovis_msgs::Stats)
  (if (not (find-package "FOVIS_MSGS"))
    (make-package "FOVIS_MSGS"))
  (shadow 'Stats (find-package "FOVIS_MSGS")))
(unless (find-package "FOVIS_MSGS::STATS")
  (make-package "FOVIS_MSGS::STATS"))

(in-package "ROS")
;;//! \htmlinclude Stats.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATUS_CODE_NO_DATA*" (find-package "FOVIS_MSGS::STATS"))
(shadow '*STATUS_CODE_NO_DATA* (find-package "FOVIS_MSGS::STATS"))
(defconstant fovis_msgs::Stats::*STATUS_CODE_NO_DATA* 0)
(intern "*STATUS_CODE_SUCCESS*" (find-package "FOVIS_MSGS::STATS"))
(shadow '*STATUS_CODE_SUCCESS* (find-package "FOVIS_MSGS::STATS"))
(defconstant fovis_msgs::Stats::*STATUS_CODE_SUCCESS* 1)
(intern "*STATUS_CODE_INSUFFICIENT_INLIERS*" (find-package "FOVIS_MSGS::STATS"))
(shadow '*STATUS_CODE_INSUFFICIENT_INLIERS* (find-package "FOVIS_MSGS::STATS"))
(defconstant fovis_msgs::Stats::*STATUS_CODE_INSUFFICIENT_INLIERS* 2)
(intern "*STATUS_CODE_OPTIMIZATION_FAILURE*" (find-package "FOVIS_MSGS::STATS"))
(shadow '*STATUS_CODE_OPTIMIZATION_FAILURE* (find-package "FOVIS_MSGS::STATS"))
(defconstant fovis_msgs::Stats::*STATUS_CODE_OPTIMIZATION_FAILURE* 3)
(intern "*STATUS_CODE_REPROJECTION_ERROR*" (find-package "FOVIS_MSGS::STATS"))
(shadow '*STATUS_CODE_REPROJECTION_ERROR* (find-package "FOVIS_MSGS::STATS"))
(defconstant fovis_msgs::Stats::*STATUS_CODE_REPROJECTION_ERROR* 4)
(defclass fovis_msgs::Stats
  :super ros::object
  :slots (_header _num_matches _num_inliers _num_detected_keypoints _num_reprojection_failures _num_keypoints _mean_reprojection_error _fast_threshold _status ))

(defmethod fovis_msgs::Stats
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:num_matches __num_matches) 0)
    ((:num_inliers __num_inliers) 0)
    ((:num_detected_keypoints __num_detected_keypoints) 0)
    ((:num_reprojection_failures __num_reprojection_failures) 0)
    ((:num_keypoints __num_keypoints) 0)
    ((:mean_reprojection_error __mean_reprojection_error) 0.0)
    ((:fast_threshold __fast_threshold) 0.0)
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _num_matches (round __num_matches))
   (setq _num_inliers (round __num_inliers))
   (setq _num_detected_keypoints (round __num_detected_keypoints))
   (setq _num_reprojection_failures (round __num_reprojection_failures))
   (setq _num_keypoints (round __num_keypoints))
   (setq _mean_reprojection_error (float __mean_reprojection_error))
   (setq _fast_threshold (float __fast_threshold))
   (setq _status (round __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:num_matches
   (&optional __num_matches)
   (if __num_matches (setq _num_matches __num_matches)) _num_matches)
  (:num_inliers
   (&optional __num_inliers)
   (if __num_inliers (setq _num_inliers __num_inliers)) _num_inliers)
  (:num_detected_keypoints
   (&optional __num_detected_keypoints)
   (if __num_detected_keypoints (setq _num_detected_keypoints __num_detected_keypoints)) _num_detected_keypoints)
  (:num_reprojection_failures
   (&optional __num_reprojection_failures)
   (if __num_reprojection_failures (setq _num_reprojection_failures __num_reprojection_failures)) _num_reprojection_failures)
  (:num_keypoints
   (&optional __num_keypoints)
   (if __num_keypoints (setq _num_keypoints __num_keypoints)) _num_keypoints)
  (:mean_reprojection_error
   (&optional __mean_reprojection_error)
   (if __mean_reprojection_error (setq _mean_reprojection_error __mean_reprojection_error)) _mean_reprojection_error)
  (:fast_threshold
   (&optional __fast_threshold)
   (if __fast_threshold (setq _fast_threshold __fast_threshold)) _fast_threshold)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _num_matches
    4
    ;; int32 _num_inliers
    4
    ;; int32 _num_detected_keypoints
    4
    ;; int32 _num_reprojection_failures
    4
    ;; int32 _num_keypoints
    4
    ;; float64 _mean_reprojection_error
    8
    ;; float64 _fast_threshold
    8
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _num_matches
       (write-long _num_matches s)
     ;; int32 _num_inliers
       (write-long _num_inliers s)
     ;; int32 _num_detected_keypoints
       (write-long _num_detected_keypoints s)
     ;; int32 _num_reprojection_failures
       (write-long _num_reprojection_failures s)
     ;; int32 _num_keypoints
       (write-long _num_keypoints s)
     ;; float64 _mean_reprojection_error
       (sys::poke _mean_reprojection_error (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _fast_threshold
       (sys::poke _fast_threshold (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _num_matches
     (setq _num_matches (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_inliers
     (setq _num_inliers (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_detected_keypoints
     (setq _num_detected_keypoints (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_reprojection_failures
     (setq _num_reprojection_failures (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_keypoints
     (setq _num_keypoints (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _mean_reprojection_error
     (setq _mean_reprojection_error (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _fast_threshold
     (setq _fast_threshold (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fovis_msgs::Stats :md5sum-) "dbaf83c8b3afd2954f468abc4e36c223")
(setf (get fovis_msgs::Stats :datatype-) "fovis_msgs/Stats")
(setf (get fovis_msgs::Stats :definition-)
      "# state of fovis
# same as fovis::stats_t lcm message
Header header

int32 num_matches
int32 num_inliers
int32 num_detected_keypoints
int32 num_reprojection_failures

# keypoints after pruning
int32 num_keypoints

float64 mean_reprojection_error
float64 fast_threshold

uint8 STATUS_CODE_NO_DATA = 0
uint8 STATUS_CODE_SUCCESS = 1
uint8 STATUS_CODE_INSUFFICIENT_INLIERS = 2
uint8 STATUS_CODE_OPTIMIZATION_FAILURE = 3
uint8 STATUS_CODE_REPROJECTION_ERROR = 4

uint8 status

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



(provide :fovis_msgs/Stats "dbaf83c8b3afd2954f468abc4e36c223")


