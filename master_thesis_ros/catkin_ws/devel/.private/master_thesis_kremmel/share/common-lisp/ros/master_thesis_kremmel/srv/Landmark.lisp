; Auto-generated. Do not edit!


(cl:in-package master_thesis_kremmel-srv)


;//! \htmlinclude Landmark-request.msg.html

(cl:defclass <Landmark-request> (roslisp-msg-protocol:ros-message)
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
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (points
    :reader points
    :initarg :points
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass Landmark-request (<Landmark-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmark-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmark-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name master_thesis_kremmel-srv:<Landmark-request> is deprecated: use master_thesis_kremmel-srv:Landmark-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Landmark-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:x-val is deprecated.  Use master_thesis_kremmel-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Landmark-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:y-val is deprecated.  Use master_thesis_kremmel-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Landmark-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:z-val is deprecated.  Use master_thesis_kremmel-srv:z instead.")
  (z m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <Landmark-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:points-val is deprecated.  Use master_thesis_kremmel-srv:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmark-request>) ostream)
  "Serializes a message object of type '<Landmark-request>"
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
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'points) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmark-request>) istream)
  "Deserializes a message object of type '<Landmark-request>"
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
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'points) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmark-request>)))
  "Returns string type for a service object of type '<Landmark-request>"
  "master_thesis_kremmel/LandmarkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark-request)))
  "Returns string type for a service object of type 'Landmark-request"
  "master_thesis_kremmel/LandmarkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmark-request>)))
  "Returns md5sum for a message object of type '<Landmark-request>"
  "660e396741d234fed172fa08d86c0fb0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmark-request)))
  "Returns md5sum for a message object of type 'Landmark-request"
  "660e396741d234fed172fa08d86c0fb0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmark-request>)))
  "Returns full string definition for message of type '<Landmark-request>"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%sensor_msgs/PointCloud2 points~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmark-request)))
  "Returns full string definition for message of type 'Landmark-request"
  (cl:format cl:nil "float64 x~%float64 y~%float64 z~%sensor_msgs/PointCloud2 points~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmark-request>))
  (cl:+ 0
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'points))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmark-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmark-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':points (points msg))
))
;//! \htmlinclude Landmark-response.msg.html

(cl:defclass <Landmark-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Landmark-response (<Landmark-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Landmark-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Landmark-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name master_thesis_kremmel-srv:<Landmark-response> is deprecated: use master_thesis_kremmel-srv:Landmark-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Landmark-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader master_thesis_kremmel-srv:success-val is deprecated.  Use master_thesis_kremmel-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Landmark-response>) ostream)
  "Serializes a message object of type '<Landmark-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Landmark-response>) istream)
  "Deserializes a message object of type '<Landmark-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Landmark-response>)))
  "Returns string type for a service object of type '<Landmark-response>"
  "master_thesis_kremmel/LandmarkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark-response)))
  "Returns string type for a service object of type 'Landmark-response"
  "master_thesis_kremmel/LandmarkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Landmark-response>)))
  "Returns md5sum for a message object of type '<Landmark-response>"
  "660e396741d234fed172fa08d86c0fb0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Landmark-response)))
  "Returns md5sum for a message object of type 'Landmark-response"
  "660e396741d234fed172fa08d86c0fb0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Landmark-response>)))
  "Returns full string definition for message of type '<Landmark-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Landmark-response)))
  "Returns full string definition for message of type 'Landmark-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Landmark-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Landmark-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Landmark-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Landmark)))
  'Landmark-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Landmark)))
  'Landmark-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Landmark)))
  "Returns string type for a service object of type '<Landmark>"
  "master_thesis_kremmel/Landmark")