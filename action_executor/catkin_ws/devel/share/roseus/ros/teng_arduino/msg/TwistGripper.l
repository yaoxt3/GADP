;; Auto-generated. Do not edit!


(when (boundp 'teng_arduino::TwistGripper)
  (if (not (find-package "TENG_ARDUINO"))
    (make-package "TENG_ARDUINO"))
  (shadow 'TwistGripper (find-package "TENG_ARDUINO")))
(unless (find-package "TENG_ARDUINO::TWISTGRIPPER")
  (make-package "TENG_ARDUINO::TWISTGRIPPER"))

(in-package "ROS")
;;//! \htmlinclude TwistGripper.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass teng_arduino::TwistGripper
  :super ros::object
  :slots (_stamp _twist _gripper ))

(defmethod teng_arduino::TwistGripper
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:twist __twist) (instance geometry_msgs::Twist :init))
    ((:gripper __gripper) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _twist __twist)
   (setq _gripper (float __gripper))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:twist
   (&rest __twist)
   (if (keywordp (car __twist))
       (send* _twist __twist)
     (progn
       (if __twist (setq _twist (car __twist)))
       _twist)))
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; geometry_msgs/Twist _twist
    (send _twist :serialization-length)
    ;; float32 _gripper
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; geometry_msgs/Twist _twist
       (send _twist :serialize s)
     ;; float32 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Twist _twist
     (send _twist :deserialize buf ptr-) (incf ptr- (send _twist :serialization-length))
   ;; float32 _gripper
     (setq _gripper (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get teng_arduino::TwistGripper :md5sum-) "ee0372fbde743581b8670dce4da4b75c")
(setf (get teng_arduino::TwistGripper :datatype-) "teng_arduino/TwistGripper")
(setf (get teng_arduino::TwistGripper :definition-)
      "time stamp
geometry_msgs/Twist twist
float32 gripper

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :teng_arduino/TwistGripper "ee0372fbde743581b8670dce4da4b75c")


