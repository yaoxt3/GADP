;; Auto-generated. Do not edit!


(when (boundp 'action_infer::PoseAction)
  (if (not (find-package "ACTION_INFER"))
    (make-package "ACTION_INFER"))
  (shadow 'PoseAction (find-package "ACTION_INFER")))
(unless (find-package "ACTION_INFER::POSEACTION")
  (make-package "ACTION_INFER::POSEACTION"))

(in-package "ROS")
;;//! \htmlinclude PoseAction.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass action_infer::PoseAction
  :super ros::object
  :slots (_pose_data _action_data ))

(defmethod action_infer::PoseAction
  (:init
   (&key
    ((:pose_data __pose_data) (instance std_msgs::Float32MultiArray :init))
    ((:action_data __action_data) (instance std_msgs::Float32MultiArray :init))
    )
   (send-super :init)
   (setq _pose_data __pose_data)
   (setq _action_data __action_data)
   self)
  (:pose_data
   (&rest __pose_data)
   (if (keywordp (car __pose_data))
       (send* _pose_data __pose_data)
     (progn
       (if __pose_data (setq _pose_data (car __pose_data)))
       _pose_data)))
  (:action_data
   (&rest __action_data)
   (if (keywordp (car __action_data))
       (send* _action_data __action_data)
     (progn
       (if __action_data (setq _action_data (car __action_data)))
       _action_data)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32MultiArray _pose_data
    (send _pose_data :serialization-length)
    ;; std_msgs/Float32MultiArray _action_data
    (send _action_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32MultiArray _pose_data
       (send _pose_data :serialize s)
     ;; std_msgs/Float32MultiArray _action_data
       (send _action_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32MultiArray _pose_data
     (send _pose_data :deserialize buf ptr-) (incf ptr- (send _pose_data :serialization-length))
   ;; std_msgs/Float32MultiArray _action_data
     (send _action_data :deserialize buf ptr-) (incf ptr- (send _action_data :serialization-length))
   ;;
   self)
  )

(setf (get action_infer::PoseAction :md5sum-) "c51c985cde497333deb62314cbacaf1b")
(setf (get action_infer::PoseAction :datatype-) "action_infer/PoseAction")
(setf (get action_infer::PoseAction :definition-)
      "std_msgs/Float32MultiArray pose_data
std_msgs/Float32MultiArray action_data

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
")



(provide :action_infer/PoseAction "c51c985cde497333deb62314cbacaf1b")


