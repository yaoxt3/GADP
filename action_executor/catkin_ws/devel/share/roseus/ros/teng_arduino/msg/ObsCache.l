;; Auto-generated. Do not edit!


(when (boundp 'teng_arduino::ObsCache)
  (if (not (find-package "TENG_ARDUINO"))
    (make-package "TENG_ARDUINO"))
  (shadow 'ObsCache (find-package "TENG_ARDUINO")))
(unless (find-package "TENG_ARDUINO::OBSCACHE")
  (make-package "TENG_ARDUINO::OBSCACHE"))

(in-package "ROS")
;;//! \htmlinclude ObsCache.msg.html


(defclass teng_arduino::ObsCache
  :super ros::object
  :slots (_stamp _pose_1 _pose_2 _point_cloud_1 _point_cloud_2 ))

(defmethod teng_arduino::ObsCache
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:pose_1 __pose_1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:pose_2 __pose_2) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:point_cloud_1 __point_cloud_1) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:point_cloud_2 __point_cloud_2) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _pose_1 __pose_1)
   (setq _pose_2 __pose_2)
   (setq _point_cloud_1 __point_cloud_1)
   (setq _point_cloud_2 __point_cloud_2)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:pose_1
   (&optional __pose_1)
   (if __pose_1 (setq _pose_1 __pose_1)) _pose_1)
  (:pose_2
   (&optional __pose_2)
   (if __pose_2 (setq _pose_2 __pose_2)) _pose_2)
  (:point_cloud_1
   (&optional __point_cloud_1)
   (if __point_cloud_1 (setq _point_cloud_1 __point_cloud_1)) _point_cloud_1)
  (:point_cloud_2
   (&optional __point_cloud_2)
   (if __point_cloud_2 (setq _point_cloud_2 __point_cloud_2)) _point_cloud_2)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32[] _pose_1
    (* 4    (length _pose_1)) 4
    ;; float32[] _pose_2
    (* 4    (length _pose_2)) 4
    ;; float32[] _point_cloud_1
    (* 4    (length _point_cloud_1)) 4
    ;; float32[] _point_cloud_2
    (* 4    (length _point_cloud_2)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32[] _pose_1
     (write-long (length _pose_1) s)
     (dotimes (i (length _pose_1))
       (sys::poke (elt _pose_1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _pose_2
     (write-long (length _pose_2) s)
     (dotimes (i (length _pose_2))
       (sys::poke (elt _pose_2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _point_cloud_1
     (write-long (length _point_cloud_1) s)
     (dotimes (i (length _point_cloud_1))
       (sys::poke (elt _point_cloud_1 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _point_cloud_2
     (write-long (length _point_cloud_2) s)
     (dotimes (i (length _point_cloud_2))
       (sys::poke (elt _point_cloud_2 i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _pose_1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pose_1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _pose_2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pose_2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _pose_2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _point_cloud_1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point_cloud_1 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point_cloud_1 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _point_cloud_2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point_cloud_2 (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _point_cloud_2 i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get teng_arduino::ObsCache :md5sum-) "0afcc9dc37d685388aeb48cae3618cb4")
(setf (get teng_arduino::ObsCache :datatype-) "teng_arduino/ObsCache")
(setf (get teng_arduino::ObsCache :definition-)
      "time stamp
float32[] pose_1
float32[] pose_2
float32[] point_cloud_1
float32[] point_cloud_2

")



(provide :teng_arduino/ObsCache "0afcc9dc37d685388aeb48cae3618cb4")


