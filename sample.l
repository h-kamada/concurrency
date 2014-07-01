
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(pr2-init)
(defclass queue
  :super propertied-object
  :slots (qlist))
(defmethod queue
  (:init ())
  (:list () qlist)
  (:push (obj) (setq qlist (concatenate cons qlist (cons obj nil))))
  (:pop () (pop qlist))
  (:next () (car qlist))
  (:clear () (setq qlist nil)))

;; sending angle-vector separately for each action controllers
(setf (get *ri* :angle-vector-queue-table) (make-hash-table))
(setf (get *ri* :av-tmp-table) (make-hash-table))
(setf (get *ri* :tm-tmp-table) (make-hash-table))
(setf (get *ri* :v-pop-table) (make-hash-table))
(setf (get *ri* :grip-state-table) (make-hash-table))
(setf (get *ri* :count-table) (make-hash-table))
;;(:default-controller :torso-controller :head-controller :rarm-controller :larm-controller)それぞれに対して
(dolist (ctype (send (pr2-interface-controller-table *ri*) :list-keys))
  (setf (gethash ctype (get *ri* :angle-vector-queue-table)) (instance queue :init))
  (setf (gethash ctype (get *ri* :av-tmp-table)) (instance queue :init))
  (setf (gethash ctype (get *ri* :tm-tmp-table)) (instance queue :init))
  (setf (gethash ctype (get *ri* :v-pop-table)) (instance queue :init))
  (setf (gethash ctype (get *ri* :grip-state-table)) (instance queue :init))
  (setf (gethash ctype (get *ri* :count-table)) 0)) ;;counter 全て0に設定しておく

(defmethod pr2-interface
  (:angle-vector-sequence-parallel
   (avs tms ctype)
   (send (gethash ctype (get self :angle-vector-queue-table)) :push (cons avs tms)))
  ;;(send (gethash :av-list (get self :av-tm-queue-table)) :push (list avs))
  ;;(send (gethash :tm-list (get self :av-tm-queue-table)):push (list tms)))
  (:angle-vector-parallel
   (av &optional (tm 2000) (ctype :default-controller))
   (if (equal ctype :default-controller)
       (progn
         (send self :angle-vector-sequence-parallel (list av) (list tm) :rarm-controller)
         (send self :angle-vector-sequence-parallel (list av) (list tm) :larm-controller)
         (send self :angle-vector-sequence-parallel (list av) (list tm) :torso-controller)
         (send self :angle-vector-sequence-parallel (list av) (list tm) :head-controller))
     (send self :angle-vector-sequence-parallel (list av) (list tm) ctype)))
  (:dequeue-seq
   ()
   (let ((qtable (get self :angle-vector-queue-table)))
     (mapcar #'(lambda (k v)
                 (let ((v-pop (send v :pop));;  ・・・(list av tm)
                       (av-tmp (instance queue :init));; ...(list av av av)
                       (tm-tmp (instance queue :init)));;... (list tm tm tm)
                       (while (null (null v-pop))
                         (ros::ros-info "~A" v-pop)
                         (cond ((equal (car (car v-pop)) :start-grasp)
                                (ros::ros-info "start")
                                (ros::ros-info "~A~A~A" (send av-tmp :list) (send tm-tmp :list) k)
                                (send *ri* :angle-vector-sequence (send av-tmp :list) (send tm-tmp :list) k)
                                (ros::ros-info "~A" (send-all (gethash k controller-table) :get-state))
                                (while (not (find actionlib_msgs::GoalStatus::*SUCCEEDED*
                                                  (send-all (gethash k controller-table) :get-state)))
                                  (ros::spin-once)
                                  (send *ri* :spin-once)
                                  (ros::ros-info "~A ~A" actionlib_msgs::GoalStatus::*SUCCEEDED* (send-all (gethash k controller-table) :get-state))
                                  )
                                (send *ri* :start-grasp (car (cdr v-pop)))
                                (send av-tmp :clear)
                                (send tm-tmp :clear)
                                (setf v-pop (send v :pop)))
                               ((equal (car (car v-pop)) :stop-grasp)
                                (ros::ros-info "stop")
                                (send *ri* :angle-vector-sequence (send av-tmp :list) (send tm-tmp :list) k)
                                (while (not (find actionlib_msgs::GoalStatus::*SUCCEEDED*
                                                  (send-all (gethash k controller-table) :get-state)))
                                  (ros::spin-once)
                                  (send *ri* :spin-once)
                                  )
                                (send *ri* :stop-grasp (car (cdr v-pop)) :wait t)
                                (send av-tmp :clear)
                                (send tm-tmp :clear)
                                (setf v-pop (send v :pop)))
                               (t
                                (ros::ros-info "t")
                                (send av-tmp :push (car (car v-pop)))
                                (send tm-tmp :push (car (cdr v-pop)))
                                (setf v-pop (send v :pop))
                                )))
                       (send *ri* :angle-vector-sequence (send av-tmp :list) (send tm-tmp :list) k)
                       ))
             (send qtable :list-keys) (send qtable :list-values))))
;;            :defa :rarm とか５つ      "<queue # ...> が５つ
  (:dequeue-seq2
   ()
   (ros::rate 10);;[hz]
   (dolist (ctype (send (pr2-interface-controller-table *ri*) :list-keys))
     (setf (gethash ctype (get *ri* :count-table)) 0)) ;;counter 全て0に設定しておく
   (let ((qtable (get self :angle-vector-queue-table)))
     (while (not (equal 4 (+ (gethash :torso-controller (get *ri* :count-table))
                             (gethash :rarm-controller (get *ri* :count-table))
                             (gethash :larm-controller (get *ri* :count-table))
                             (gethash :head-controller (get *ri* :count-table)))))
       (dolist (ctype (send (pr2-interface-controller-table *ri*) :list-keys))
         (warn "[debug0] dolist print is here~%")
         (ros::ros-info "~A"(send-all (gethash ctype controller-table) :get-state)))
       (unix:sleep 1)
       (ros::spin-once)
       (send *ri* :spin-once)
       (mapcar #'(lambda (k v)
                   (ros::spin-once)
                   (send *ri* :spin-once)
                   (ros::ros-info "~A" k)
                   (let ((v-pop (car (send v :list))) ;;(av tm)のcar部(つまりpop部) popは後で
                         (av-tmp (send (gethash k (get self :av-tmp-table)) :list))
                         (tm-tmp (send (gethash k (get self :tm-tmp-table)) :list))
                         (grip-state (send (gethash k (get self :grip-state-table)) :pop)))
                     (warn "[debug11] ~a ~a ~%" k (gethash k (get self :grip-state-table)))
                     (if (null v-pop)
                         (if (not (null (car av-tmp)))
                             (progn
                               (warn "[debug1] ~a is nil1~%" k)
                               (send *ri* :angle-vector-sequence av-tmp tm-tmp k);;取り出したので
                               (send (gethash k (get self :av-tmp-table)) :clear);;(av av av )clear
                               (send (gethash k (get self :tm-tmp-table)) :clear))
                           (progn
                             (warn "[debug1] ~a is nil2~%" k)
                             (setf (gethash k (get *ri* :count-table)) 1) ;;counter to 1
                             (ros::ros-info "null")
                             (send (gethash k (get self :av-tmp-table)) :clear)
                             (send (gethash k (get self :tm-tmp-table)) :clear)))
                       (progn
                         (warn "[debug2] ~a is not nil~%" k)
                         (ros::ros-info "v: ~A" (send v :list))
                         (ros::ros-info "grip-state ~A" grip-state)
                         (ros::spin-once)
                         (send *ri* :spin-once)
                         (warn "[debug3] start1 cont ~a, state ~a~%" k (send-all (gethash k controller-table) :get-state))
                         (cond ((equal grip-state :start)
                                (warn "[debug3] grip-start1 cont ~a, state ~a~%" k (send-all (gethash k controller-table) :get-state))
                                (if (find actionlib_msgs::GoalStatus::*SUCCEEDED*
                                          (send-all (gethash k controller-table) :get-state))
                                    (progn
                                      (send *ri* :start-grasp (car (cdr v-pop)))
                                      (ros::ros-info "suc, state: ~A, ~A" actionlib_msgs::GoalStatus::*SUCCEEDED* (send-all (gethash k controller-table) :get-state))
                                      (send v :pop) ;; :start-graspだったlistの部分をpopして無くす
                                      )
                                  (ros::ros-info "suc, state: ~A, ~A" actionlib_msgs::GoalStatus::*SUCCEEDED* (send-all (gethash k controller-table) :get-state))))
                               ((equal grip-state :stop)
                                (warn "[debug3] grip-stop1 cont ~a, state ~a~%" k (send-all (gethash k controller-table) :get-state))
                                (if (find actionlib_msgs::GoalStatus::*SUCCEEDED*
                                          (send-all (gethash k controller-table) :get-state))
                                    (progn
                                      (send *ri* :stop-grasp (car (cdr v-pop)) :wait t)
                                      (ros::ros-info "~A~A" actionlib_msgs::GoalStatus::*SUCCEEDED* (send-all (gethash k controller-table) :get-state))
                                      (send v :pop)) ;; :start-graspだったlistの部分をpopして無くす
                                  (ros::ros-info "~A~A" actionlib_msgs::GoalStatus::*SUCCEEDED* (send-all (gethash k controller-table) :get-state))))
                               (t
                                (warn "[debug3] otherwise cont ~a, state ~a~%" k (send-all (gethash k controller-table) :get-state))
                                (cond ((equal (car (car v-pop)) :start-grasp)
                                       (ros::ros-info "start flag")
                                       (ros::ros-info "~A~A~A" av-tmp tm-tmp k)
                                       (warn "[debug10] ~a ~a ~a~%" av-tmp tm-tmp k)
                                       (send *ri* :angle-vector-sequence av-tmp tm-tmp k);;取り出したでの
                                       (send (gethash k (get self :av-tmp-table)) :clear);;(av av av )らを初期化
                                       (send (gethash k (get self :tm-tmp-table)) :clear)
                                       (send (gethash k (get self :grip-state-table)) :push :start)
                                       (warn "[debug12] ~a ~a ~%" k (gethash k (get self :grip-state-table))))
                                      ((equal (car (car v-pop)) :stop-grasp)
                                       (ros::ros-info "stop")
                                       (ros::ros-info "av-tmp ~A" av-tmp)
                                       (send *ri* :angle-vector-sequence av-tmp tm-tmp k)
                                       (send (gethash k (get self :av-tmp-table)) :clear)
                                       (send (gethash k (get self :tm-tmp-table)) :clear)
                                       (send (gethash k (get self :grip-state-table)) :push :stop))
                                      (t
                                       (ros::ros-info "t")
                                       (send (gethash k (get self :av-tmp-table)) :push (car (car v-pop)))
                                       (send (gethash k (get self :tm-tmp-table)) :push (car (cdr v-pop)))
                                       (send v :pop)
                                       ))
                                ))
                         ))))
                   (send qtable :list-keys) (send qtable :list-values)))))
   ;;           :defa :rarm とか５つ      "<queue # ...> が５つ
   (:dequeue
    ()
    (let ((qtable (get self :angle-vector-queue-table)))
      (mapcar #'(lambda (k v)
                  (unless (equal k :default-controller)
                    (ros::ros-info "~A" (send-all (gethash k controller-table) :get-state))
                    (when (not (find actionlib_msgs::GoalStatus::*ACTIVE*
                                     (send-all (gethash k controller-table) :get-state)))
                      (let ((avs-tms (send v :pop)))
                        (unless (null (car(car avs-tms)))
                          (unless (null (car(cdr avs-tms)))
                            (send self :angle-vector (car(car avs-tms)) (car(cdr avs-tms)) k)))))))
              (send qtable :list-keys) (send qtable :list-values)))))

;;(pr2) ;;(setq *pr2* (instance pr2-robot :init)) pr2.l
(pr2-init);;(setq *ri* (instance pr2-interface :init))etc...


(defun manip (&key (a 1))
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000)
  ;;  (send *pr2* :rarm :end-coords)
  ;; (let ((c (send *pr2* :rarm :end-coords :copy-worldcoords))) copyしないと、手先の目印自体を動かしてしまうことになって良くないので、手先の目印のコピーを作る
  ;;   (send c :translate (float-vector 100 0 0) :world)　それを１０cm動かして
  ;;   (send *pr2* :rarm :inverse-kinematics c :rotation-axis nil)　inverse-linematicsを解く
  ;;   )
  ;;  (send (send *pr2* :rarm :end-coords) :copy-worldcoords)
  (send *pr2* :rarm :move-end-pos (float-vector 100 0 0) :world :rotation-axis nil) ;;それらとほぼ同義なのがmove-end-pos
  ;;基本的に#fは使わず、float-vectorを使う方がいいそうだ
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000)
  (send *ri* :angle-vector-parallel a)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000)
  (send *pr2* :larm :move-end-pos #f(100 0 0) :world :rotation-axis nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000)
  (send *pr2* :larm :move-end-pos #f(-100 0 0) :world :rotation-axis nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000)
  ;;(send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 -300 0) :world))
  ;;(send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :rotate (deg2rad 10) :z :world))
  ;;make-irtviewer
  ;;send *irtviewer* :draw-object
  )

(defun rarm-manip(&key (a 1))
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
;;  (send *ri* :stop-grasp :rarm)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :rotate (deg2rad -40) :z :world) :use-torso nil)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :rotate (deg2rad 10) :y :world) :use-torso nil)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :rotate (deg2rad 30) :x :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 200 0 -100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :rarm-controller)

  (send *ri* :angle-vector-parallel :start-grasp :rarm :rarm-controller) ;1000 is dummy

  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector -100 0 100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 -200 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 -100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :rarm-controller)

  (send *ri* :angle-vector-parallel :stop-grasp :rarm :rarm-controller) ;1000 is dummy

  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  )
(defun larm-manip(&key (a 1))
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :stop-grasp :larm)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :rotate (deg2rad 40) :z :world) :use-torso nil :use-torso nil)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :rotate (deg2rad 10) :y :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 300 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :angle-vector-parallel :start-grasp :larm :larm-controller) ;1000 is dummy
;;  (send *ri* :start-grasp :larm)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector -100 0 100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 0 200 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 -100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :angle-vector-parallel :stop-grasp :larm :larm-controller) ;1000 is dummy
;;  (send *ri* :stop-grasp :larm)
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
  )

(defun rarm (&key (sp 0)(sr 0))
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *pr2* :rarm :shoulder-p :joint-angle 70);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle -74);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *pr2* :rarm :shoulder-p :joint-angle 70);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle -74);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *pr2* :rarm :shoulder-p :joint-angle 70);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle -74);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
  )

(defun larm (&key (sp 0)(sr 0))
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向 74
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向 70
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *pr2* :larm :shoulder-p :joint-angle 74);;肩上げる方向 74
  (send *pr2* :larm :shoulder-r :joint-angle 70);;;回外回内方向 70
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *pr2* :larm :shoulder-p :joint-angle 74);;肩上げる方向 74
  (send *pr2* :larm :shoulder-r :joint-angle 70);;;回外回内方向 70
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *pr2* :larm :shoulder-p :joint-angle 74);;肩上げる方向 74
  (send *pr2* :larm :shoulder-r :joint-angle 70);;;回外回内方向 70
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
  )

(defun torso (&key (wz 50))
  (send *pr2* :torso :waist-z :joint-angle 200)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  (send *pr2* :torso :waist-z :joint-angle 50)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  (send *pr2* :torso :waist-z :joint-angle 200)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  (send *pr2* :torso :waist-z :joint-angle 50)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  (send *pr2* :torso :waist-z :joint-angle 200)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  (send *pr2* :torso :waist-z :joint-angle 50)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 3000 :torso-controller)
  )
;;(send *pr2* :head :look-at (apply 'midpoint 0.5 (send *pr2* :arms :end-coords :worldpos)))
;;(send *pr2* :head :look-at (send *pr2* :rarm :end-coords :worldpos))
(defun head (&key (np 0))
  (send *pr2* :head :neck-p :joint-angle 40);; 0
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle 20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  (send *pr2* :head :neck-y :joint-angle -20);; 0
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 8000 :head-controller)
  )

(defun resetpose (&key (arg 1))
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :default-controller)
  )

(defun catch (&key (arg 1))
  ;;  (send *ri* :stop-grasp :larm)
  (send *pr2* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector #f(49.9893 -0.184653 -15.2376 113.545 -92.2444 165.71 -58.6879 -100.272 -59.9531 74.0963 -69.9794 -119.891 -19.9977 -29.9993 180.003 -0.006113 -2.37))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :angle-vector #f(49.9893 -0.184653 -15.2376 123.545 -82.2444 165.71 -58.6879 -100.272 -59.9531 74.0963 -69.9794 -119.891 -19.9977 -29.9993 180.003 -0.006113 -2.37))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  ;;  (send *ri* :start-grasp :larm)
  (send *pr2* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)

  )

(defun move(&key (arg 1))
  (send *ri* :go-pos 0 3 0)
  (send *ri* :go-pos 0 -3 0)
  )

(defun nagiharai (&key (arg 1))
  (send *ri* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382) 1000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9893 -0.184653 -15.2376 113.545 -92.2444 165.71 -58.6879 -19.272 -59.9531 74.0963 -69.9794 -119.891 -19.9977 -29.9993 180.003 -0.006113 -2.37) 1000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9892 23.4666 -16.7838 112.121 -91.473 158.839 -61.0013 -17.532 -59.9484 74.1011 -69.9794 -119.891 -19.9977 -29.9993 180.003 0.005887 -2.346) 1000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382) 1000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 1000 :larm-controller)
  )

(defun rarm-nagi(&key (a 1))
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 0) :world) :use-torso nil)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :rotate (deg2rad 20) :y :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 0 0 -100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)

  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector -200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)

  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector 200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :rarm :end-coords) :copy-worldcoords) :translate (float-vector -200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :rarm-controller)
  )


(defun larm-nagi(&key (a 1))
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 100 0 0) :world) :use-torso nil);; :use-torso nil ってするとたぶんtorsoを使わずに済む
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :rotate (deg2rad 20) :y :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 0 0 -100) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)

  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector -200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)

  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector 200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)
  (send *pr2* :inverse-kinematics (send  (send (send *pr2* :larm :end-coords) :copy-worldcoords) :translate (float-vector -200 0 0) :world) :use-torso nil)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 1000 :larm-controller)
  )


(defun demo()
  (progn
    (send *pr2* :reset-manip-pose)
    (send *pr2* :torso :waist-z :joint-angle 380)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (head )

    (rarm-nagi)
    (rarm )
    (rarm-manip)

    (larm )
    (larm-manip)
    (larm-nagi)

    (ros::rate 10);;[hz]
    (while (ros::ok)
      ;;      (print 1)
      (ros::spin-once)
      (send *ri* :spin-once)
      (send *ri* :dequeue)
      (ros::sleep)
      )))

;;  (torso 150)))
;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :rarm-controller)
;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)
;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :head-controller)
;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :torso-controller)
;;これらをmapcarで順にやらせているのが通常の:angle-vector





