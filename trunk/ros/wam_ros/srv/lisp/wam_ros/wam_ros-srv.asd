
(in-package :asdf)

(defsystem "wam_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "WamCommands" :depends-on ("_package"))
    (:file "_package_WamCommands" :depends-on ("_package"))
    ))
