#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_pcl_ros")

(ros::roseus "listener")
(defclass string-cb-class
  :super propertied-object
  :slots ())
(defmethod string-cb-class
  (:init () (ros::subscribe "num_topic" jsk_pcl_ros::Int32Stamped #'send self :string-cb))
  (:string-cb (msg) (print(format nil "sec [~A] nsec [~A] msg [~A]" (send (send msg :header :stamp) :sec) (send (send msg :header :stamp) :nsec) (send msg :data)))))

(setq m (instance string-cb-class :init))
(do-until-key
 (ros::spin-once)
)
