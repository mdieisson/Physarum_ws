
(cl:in-package :asdf)

(defsystem "stage_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetModelPose" :depends-on ("_package_GetModelPose"))
    (:file "_package_GetModelPose" :depends-on ("_package"))
    (:file "SetModelPose" :depends-on ("_package_SetModelPose"))
    (:file "_package_SetModelPose" :depends-on ("_package"))
  ))