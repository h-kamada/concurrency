#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "jsk_pcl_ros")

(ros::roseus "talker")
(ros::advertise "num_topic" jsk_pcl_ros::Int32Stamped 1)
(ros::rate 100)
(while (ros::ok)
  (setq msg (instance jsk_pcl_ros::Int32Stamped :init))
  (send msg :data 1)
  (send msg :header :stamp (ros::time-now))
  (ros::ros-info "msg [~A]" (send msg :data))
  (ros::publish "num_topic" msg)
  (ros::sleep)
  )
(ros::roseus "shutdown")
(exit)

