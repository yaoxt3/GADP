;; Auto-generated. Do not edit!


(when (boundp 'teng_arduino::channel)
  (if (not (find-package "TENG_ARDUINO"))
    (make-package "TENG_ARDUINO"))
  (shadow 'channel (find-package "TENG_ARDUINO")))
(unless (find-package "TENG_ARDUINO::CHANNEL")
  (make-package "TENG_ARDUINO::CHANNEL"))

(in-package "ROS")
;;//! \htmlinclude channel.msg.html


(defclass teng_arduino::channel
  :super ros::object
  :slots (_stamp _c ))

(defmethod teng_arduino::channel
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:c __c) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _c (float __c))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:c
   (&optional __c)
   (if __c (setq _c __c)) _c)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32 _c
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _c
       (sys::poke _c (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _c
     (setq _c (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get teng_arduino::channel :md5sum-) "3a18997610954b0a6cd4a6ffda94b3bf")
(setf (get teng_arduino::channel :datatype-) "teng_arduino/channel")
(setf (get teng_arduino::channel :definition-)
      "time stamp
float32 c

")



(provide :teng_arduino/channel "3a18997610954b0a6cd4a6ffda94b3bf")


