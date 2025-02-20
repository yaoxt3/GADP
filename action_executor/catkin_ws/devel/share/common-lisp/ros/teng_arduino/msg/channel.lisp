; Auto-generated. Do not edit!


(cl:in-package teng_arduino-msg)


;//! \htmlinclude channel.msg.html

(cl:defclass <channel> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (c
    :reader c
    :initarg :c
    :type cl:float
    :initform 0.0))
)

(cl:defclass channel (<channel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <channel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'channel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teng_arduino-msg:<channel> is deprecated: use teng_arduino-msg:channel instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <channel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:stamp-val is deprecated.  Use teng_arduino-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'c-val :lambda-list '(m))
(cl:defmethod c-val ((m <channel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:c-val is deprecated.  Use teng_arduino-msg:c instead.")
  (c m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <channel>) ostream)
  "Serializes a message object of type '<channel>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'c))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <channel>) istream)
  "Deserializes a message object of type '<channel>"
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
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'c) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<channel>)))
  "Returns string type for a message object of type '<channel>"
  "teng_arduino/channel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'channel)))
  "Returns string type for a message object of type 'channel"
  "teng_arduino/channel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<channel>)))
  "Returns md5sum for a message object of type '<channel>"
  "3a18997610954b0a6cd4a6ffda94b3bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'channel)))
  "Returns md5sum for a message object of type 'channel"
  "3a18997610954b0a6cd4a6ffda94b3bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<channel>)))
  "Returns full string definition for message of type '<channel>"
  (cl:format cl:nil "time stamp~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'channel)))
  "Returns full string definition for message of type 'channel"
  (cl:format cl:nil "time stamp~%float32 c~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <channel>))
  (cl:+ 0
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <channel>))
  "Converts a ROS message object to a list"
  (cl:list 'channel
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':c (c msg))
))
