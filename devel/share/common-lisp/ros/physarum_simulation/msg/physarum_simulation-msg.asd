
(cl:in-package :asdf)

(defsystem "physarum_simulation-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ColetaEvent" :depends-on ("_package_ColetaEvent"))
    (:file "_package_ColetaEvent" :depends-on ("_package"))
    (:file "ContainerTask" :depends-on ("_package_ContainerTask"))
    (:file "_package_ContainerTask" :depends-on ("_package"))
  ))