
(in-package :asdf)

(defsystem "wam_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Joints" :depends-on ("_package"))
    (:file "_package_Joints" :depends-on ("_package"))
    ))
