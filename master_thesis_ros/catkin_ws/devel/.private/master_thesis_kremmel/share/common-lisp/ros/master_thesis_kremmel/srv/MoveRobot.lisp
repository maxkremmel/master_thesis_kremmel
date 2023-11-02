; Auto-generated. Do not edit!


(cl:in-package master_thesis_kremmel-srv)


;//! \htmlinclude MoveRobot-request.msg.html

(cl:defclass <MoveRobot-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0))
)

(cl:defclass MoveRobot-request (<MoveRobot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRobot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRobot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name master_thesis_kremmel-srv:<MoveRobot-request> is deprecated: use master_thesis_kremmel-srv:MoveRobot-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <MoveRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:x-val is deprecated.  Use master_thesis_kremmel-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <MoveRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:y-val is deprecated.  Use master_thesis_kremmel-srv:y instead.")
  (y m))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <MoveRobot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:t-val is deprecated.  Use master_thesis_kremmel-srv:t instead.")
  (t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRobot-request>) ostream)
  "Serializes a message object of type '<MoveRobot-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRobot-request>) istream)
  "Deserializes a message object of type '<MoveRobot-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRobot-request>)))
  "Returns string type for a service object of type '<MoveRobot-request>"
  "master_thesis_kremmel/MoveRobotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot-request)))
  "Returns string type for a service object of type 'MoveRobot-request"
  "master_thesis_kremmel/MoveRobotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRobot-request>)))
  "Returns md5sum for a message object of type '<MoveRobot-request>"
  "4a28bd10bf2ad79a2c715f25c5a7ebd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRobot-request)))
  "Returns md5sum for a message object of type 'MoveRobot-request"
  "4a28bd10bf2ad79a2c715f25c5a7ebd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRobot-request>)))
  "Returns full string definition for message of type '<MoveRobot-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRobot-request)))
  "Returns full string definition for message of type 'MoveRobot-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 t~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRobot-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRobot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRobot-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':t (t msg))
))
;//! \htmlinclude MoveRobot-response.msg.html

(cl:defclass <MoveRobot-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MoveRobot-response (<MoveRobot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveRobot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveRobot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name master_thesis_kremmel-srv:<MoveRobot-response> is deprecated: use master_thesis_kremmel-srv:MoveRobot-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveRobot-response>) ostream)
  "Serializes a message object of type '<MoveRobot-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveRobot-response>) istream)
  "Deserializes a message object of type '<MoveRobot-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveRobot-response>)))
  "Returns string type for a service object of type '<MoveRobot-response>"
  "master_thesis_kremmel/MoveRobotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot-response)))
  "Returns string type for a service object of type 'MoveRobot-response"
  "master_thesis_kremmel/MoveRobotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveRobot-response>)))
  "Returns md5sum for a message object of type '<MoveRobot-response>"
  "4a28bd10bf2ad79a2c715f25c5a7ebd3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveRobot-response)))
  "Returns md5sum for a message object of type 'MoveRobot-response"
  "4a28bd10bf2ad79a2c715f25c5a7ebd3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveRobot-response>)))
  "Returns full string definition for message of type '<MoveRobot-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveRobot-response)))
  "Returns full string definition for message of type 'MoveRobot-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveRobot-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveRobot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveRobot-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MoveRobot)))
  'MoveRobot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MoveRobot)))
  'MoveRobot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveRobot)))
  "Returns string type for a service object of type '<MoveRobot>"
  "master_thesis_kremmel/MoveRobot")