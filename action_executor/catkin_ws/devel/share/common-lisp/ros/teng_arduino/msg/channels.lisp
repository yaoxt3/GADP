; Auto-generated. Do not edit!


(cl:in-package teng_arduino-msg)


;//! \htmlinclude channels.msg.html

(cl:defclass <channels> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (c
    :reader c
    :initarg :c
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass channels (<channels>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <channels>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'channels)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teng_arduino-msg:<channels> is deprecated: use teng_arduino-msg:channels instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <channels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:stamp-val is deprecated.  Use teng_arduino-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <channels>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:c-val is deprecated.  Use teng_arduino-msg:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <channels>) ostream)
  "Serializes a message object of type '<channels>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'c))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <channels>) istream)
  "Deserializes a message object of type '<channels>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'c) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'c)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<channels>)))
  "Returns string type for a message object of type '<channels>"
  "teng_arduino/channels")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'channels)))
  "Returns string type for a message object of type 'channels"
  "teng_arduino/channels")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<channels>)))
  "Returns md5sum for a message object of type '<channels>"
  "78b27486b5f7984dbb5e15d9126d42d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'channels)))
  "Returns md5sum for a message object of type 'channels"
  "78b27486b5f7984dbb5e15d9126d42d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<channels>)))
  "Returns full string definition for message of type '<channels>"
  (cl:format cl:nil "time stamp~%float32[] c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'channels)))
  "Returns full string definition for message of type 'channels"
  (cl:format cl:nil "time stamp~%float32[] c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <channels>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'c) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <channels>))
  "Converts a ROS message object to a list"
  (cl:list 'channels
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':c (c msg))
))
