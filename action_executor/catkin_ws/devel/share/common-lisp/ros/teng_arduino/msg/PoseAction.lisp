; Auto-generated. Do not edit!


(cl:in-package teng_arduino-msg)


;//! \htmlinclude PoseAction.msg.html

(cl:defclass <PoseAction> (roslisp-msg-protocol:ros-message)
  ((pose_data
    :reader pose_data
    :initarg :pose_data
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (action_data
    :reader action_data
    :initarg :action_data
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray)))
)

(cl:defclass PoseAction (<PoseAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PoseAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PoseAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name teng_arduino-msg:<PoseAction> is deprecated: use teng_arduino-msg:PoseAction instead.")))

(cl:ensure-generic-function 'pose_data-val :lambda-list '(m))
(cl:defmethod pose_data-val ((m <PoseAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:pose_data-val is deprecated.  Use teng_arduino-msg:pose_data instead.")
  (pose_data m))

(cl:ensure-generic-function 'action_data-val :lambda-list '(m))
(cl:defmethod action_data-val ((m <PoseAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader teng_arduino-msg:action_data-val is deprecated.  Use teng_arduino-msg:action_data instead.")
  (action_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PoseAction>) ostream)
  "Serializes a message object of type '<PoseAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_data) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PoseAction>) istream)
  "Deserializes a message object of type '<PoseAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_data) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PoseAction>)))
  "Returns string type for a message object of type '<PoseAction>"
  "teng_arduino/PoseAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PoseAction)))
  "Returns string type for a message object of type 'PoseAction"
  "teng_arduino/PoseAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PoseAction>)))
  "Returns md5sum for a message object of type '<PoseAction>"
  "c51c985cde497333deb62314cbacaf1b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PoseAction)))
  "Returns md5sum for a message object of type 'PoseAction"
  "c51c985cde497333deb62314cbacaf1b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PoseAction>)))
  "Returns full string definition for message of type '<PoseAction>"
  (cl:format cl:nil "std_msgs/Float32MultiArray pose_data~%std_msgs/Float32MultiArray action_data~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PoseAction)))
  "Returns full string definition for message of type 'PoseAction"
  (cl:format cl:nil "std_msgs/Float32MultiArray pose_data~%std_msgs/Float32MultiArray action_data~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PoseAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_data))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PoseAction>))
  "Converts a ROS message object to a list"
  (cl:list 'PoseAction
    (cl:cons ':pose_data (pose_data msg))
    (cl:cons ':action_data (action_data msg))
))
