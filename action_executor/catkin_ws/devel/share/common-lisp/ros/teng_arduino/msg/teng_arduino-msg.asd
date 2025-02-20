
(cl:in-package :asdf)

(defsystem "teng_arduino-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ObsCache" :depends-on ("_package_ObsCache"))
    (:file "_package_ObsCache" :depends-on ("_package"))
    (:file "PoseAction" :depends-on ("_package_PoseAction"))
    (:file "_package_PoseAction" :depends-on ("_package"))
    (:file "TwistGripper" :depends-on ("_package_TwistGripper"))
    (:file "_package_TwistGripper" :depends-on ("_package"))
    (:file "channel" :depends-on ("_package_channel"))
    (:file "_package_channel" :depends-on ("_package"))
    (:file "channels" :depends-on ("_package_channels"))
    (:file "_package_channels" :depends-on ("_package"))
  ))