;; Auto-generated. Do not edit!


(when (boundp 'master_thesis_kremmel::MoveRobot)
  (if (not (find-package "MASTER_THESIS_KREMMEL"))
    (make-package "MASTER_THESIS_KREMMEL"))
  (shadow 'MoveRobot (find-package "MASTER_THESIS_KREMMEL")))
(unless (find-package "MASTER_THESIS_KREMMEL::MOVEROBOT")
  (make-package "MASTER_THESIS_KREMMEL::MOVEROBOT"))
(unless (find-package "MASTER_THESIS_KREMMEL::MOVEROBOTREQUEST")
  (make-package "MASTER_THESIS_KREMMEL::MOVEROBOTREQUEST"))
(unless (find-package "MASTER_THESIS_KREMMEL::MOVEROBOTRESPONSE")
  (make-package "MASTER_THESIS_KREMMEL::MOVEROBOTRESPONSE"))

(in-package "ROS")





(defclass master_thesis_kremmel::MoveRobotRequest
  :super ros::object
  :slots (_x _y _t ))

(defmethod master_thesis_kremmel::MoveRobotRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:t __t) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _t (float __t))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _t
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _t
       (sys::poke _t (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _t
     (setq _t (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass master_thesis_kremmel::MoveRobotResponse
  :super ros::object
  :slots (_success ))

(defmethod master_thesis_kremmel::MoveRobotResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass master_thesis_kremmel::MoveRobot
  :super ros::object
  :slots ())

(setf (get master_thesis_kremmel::MoveRobot :md5sum-) "164463c3fac04b29fa9ab47a504dfc26")
(setf (get master_thesis_kremmel::MoveRobot :datatype-) "master_thesis_kremmel/MoveRobot")
(setf (get master_thesis_kremmel::MoveRobot :request) master_thesis_kremmel::MoveRobotRequest)
(setf (get master_thesis_kremmel::MoveRobot :response) master_thesis_kremmel::MoveRobotResponse)

(defmethod master_thesis_kremmel::MoveRobotRequest
  (:response () (instance master_thesis_kremmel::MoveRobotResponse :init)))

(setf (get master_thesis_kremmel::MoveRobotRequest :md5sum-) "164463c3fac04b29fa9ab47a504dfc26")
(setf (get master_thesis_kremmel::MoveRobotRequest :datatype-) "master_thesis_kremmel/MoveRobotRequest")
(setf (get master_thesis_kremmel::MoveRobotRequest :definition-)
      "float64 x
float64 y
float64 t
---
bool success

")

(setf (get master_thesis_kremmel::MoveRobotResponse :md5sum-) "164463c3fac04b29fa9ab47a504dfc26")
(setf (get master_thesis_kremmel::MoveRobotResponse :datatype-) "master_thesis_kremmel/MoveRobotResponse")
(setf (get master_thesis_kremmel::MoveRobotResponse :definition-)
      "float64 x
float64 y
float64 t
---
bool success

")



(provide :master_thesis_kremmel/MoveRobot "164463c3fac04b29fa9ab47a504dfc26")


