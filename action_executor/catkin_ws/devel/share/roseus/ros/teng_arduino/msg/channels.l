;; Auto-generated. Do not edit!


(when (boundp 'teng_arduino::channels)
  (if (not (find-package "TENG_ARDUINO"))
    (make-package "TENG_ARDUINO"))
  (shadow 'channels (find-package "TENG_ARDUINO")))
(unless (find-package "TENG_ARDUINO::CHANNELS")
  (make-package "TENG_ARDUINO::CHANNELS"))

(in-package "ROS")
;;//! \htmlinclude channels.msg.html


(defclass teng_arduino::channels
  :super ros::object
  :slots (_stamp _c ))

(defmethod teng_arduino::channels
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:c __c) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _c __c)
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
    ;; float32[] _c
    (* 4    (length _c)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32[] _c
     (write-long (length _c) s)
     (dotimes (i (length _c))
       (sys::poke (elt _c i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _c
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _c (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _c i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get teng_arduino::channels :md5sum-) "78b27486b5f7984dbb5e15d9126d42d7")
(setf (get teng_arduino::channels :datatype-) "teng_arduino/channels")
(setf (get teng_arduino::channels :definition-)
      "time stamp
float32[] c

")



(provide :teng_arduino/channels "78b27486b5f7984dbb5e15d9126d42d7")


