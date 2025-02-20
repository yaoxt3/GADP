; Auto-generated. Do not edit!


(cl:in-package action_infer-msg)


;//! \htmlinclude TwistGripper.msg.html

(cl:defclass <TwistGripper> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (twist
    :reader twist
    :initarg :twist
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:float
    :initform 0.0))
)

(cl:defclass TwistGripper (<TwistGripper>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TwistGripper>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TwistGripper)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name action_infer-msg:<TwistGripper> is deprecated: use action_infer-msg:TwistGripper instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <TwistGripper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_infer-msg:stamp-val is deprecated.  Use action_infer-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'twist-val :lambda-list '(m))
(cl:defmethod twist-val ((m <TwistGripper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_infer-msg:twist-val is deprecated.  Use action_infer-msg:twist instead.")
  (twist m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <TwistGripper>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader action_infer-msg:gripper-val is deprecated.  Use action_infer-msg:gripper instead.")
  (gripper m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TwistGripper>) ostream)
  "Serializes a message object of type '<TwistGripper>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'twist) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TwistGripper>) istream)
  "Deserializes a message object of type '<TwistGripper>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'twist) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TwistGripper>)))
  "Returns string type for a message object of type '<TwistGripper>"
  "action_infer/TwistGripper")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TwistGripper)))
  "Returns string type for a message object of type 'TwistGripper"
  "action_infer/TwistGripper")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TwistGripper>)))
  "Returns md5sum for a message object of type '<TwistGripper>"
  "ee0372fbde743581b8670dce4da4b75c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TwistGripper)))
  "Returns md5sum for a message object of type 'TwistGripper"
  "ee0372fbde743581b8670dce4da4b75c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TwistGripper>)))
  "Returns full string definition for message of type '<TwistGripper>"
  (cl:format cl:nil "time stamp~%geometry_msgs/Twist twist~%float32 gripper~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TwistGripper)))
  "Returns full string definition for message of type 'TwistGripper"
  (cl:format cl:nil "time stamp~%geometry_msgs/Twist twist~%float32 gripper~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TwistGripper>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'twist))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TwistGripper>))
  "Converts a ROS message object to a list"
  (cl:list 'TwistGripper
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':twist (twist msg))
    (cl:cons ':gripper (gripper msg))
))
