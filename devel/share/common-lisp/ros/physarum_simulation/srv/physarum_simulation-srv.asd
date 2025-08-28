
(cl:in-package :asdf)

(defsystem "physarum_simulation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LockTask" :depends-on ("_package_LockTask"))
    (:file "_package_LockTask" :depends-on ("_package"))
  ))