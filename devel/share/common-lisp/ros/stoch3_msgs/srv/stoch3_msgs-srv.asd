
(cl:in-package :asdf)

(defsystem "stoch3_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "ControllerSupervisorState" :depends-on ("_package_ControllerSupervisorState"))
    (:file "_package_ControllerSupervisorState" :depends-on ("_package"))
    (:file "Gait" :depends-on ("_package_Gait"))
    (:file "_package_Gait" :depends-on ("_package"))
    (:file "SwitchController" :depends-on ("_package_SwitchController"))
    (:file "_package_SwitchController" :depends-on ("_package"))
  ))