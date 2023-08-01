
(cl:in-package :asdf)

(defsystem "fovis_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Stats" :depends-on ("_package_Stats"))
    (:file "_package_Stats" :depends-on ("_package"))
    (:file "VisualOdometryUpdate" :depends-on ("_package_VisualOdometryUpdate"))
    (:file "_package_VisualOdometryUpdate" :depends-on ("_package"))
  ))