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

;(:default-controller :torso-controller :head-controller :rarm-controller :larm-controller)それぞれに対して
(dolist (ctype (send (pr2-interface-controller-table *ri*) :list-keys))
  (setf (gethash ctype (get *ri* :angle-vector-queue-table)) (instance queue :init)))

(defmethod pr2-interface
  (:angle-vector-sequence-parallel
   (avs tms ctype)
   (send (gethash ctype (get self :angle-vector-queue-table)) :push (cons avs tms)))
  (:angle-vector-parallel
   (av tm ctype)
   (send self :angle-vector-sequence-parallel (list av) (list tm) ctype))
  (:dequeue-with-checking-results
   ();;   ((a (send (get self :angle-vector-queue-table) :list-keys)))
   (let ((qtable (get self :angle-vector-queue-table)))
     (mapcar #'(lambda (k v)
                 (when (not (find actionlib_msgs::GoalStatus::*ACTIVE*
                                  (send-all (gethash k controller-table) :get-state)))
                   (let ((avs-tms (send v :pop)))
                     (send self :angle-vector-sequence (car avs-tms) (cdr avs-tms) k))))
             (send qtable :list-keys) (send qtable :list-values))))
;;           :defa :rarm とか５つ      "<queue # ...> が５つ
  )


;;(pr2) ;;(setq *pr2* (instance pr2-robot :init)) pr2.l
(pr2-init);;(setq *ri* (instance pr2-interface :init))etc...


(defun rarm (&key (sp 0)(sr 0))
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
;;  (send *ri* :dequeue-with-checking-results)
;;  (unix:sleep 1)
  (send *pr2* :rarm :shoulder-p :joint-angle 20);;肩上げる方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
;;  (send *ri* :dequeue-with-checking-results)
;;  (unix:sleep 1)
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
;;  (send *ri* :dequeue-with-checking-results)
;;  (unix:sleep 1)
  (send *pr2* :rarm :shoulder-p :joint-angle 20);;肩上げる方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :rarm-controller)
;;  (send *ri* :dequeue-with-checking-results)
)

(defun larm (&key (sp 0)(sr 0))
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
;;  (send *ri* :dequeue-with-checking-results)
  ;;(unix:sleep 1)
  (send *pr2* :larm :shoulder-p :joint-angle 20);;肩上げる方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
;;  (send *ri* :dequeue-with-checking-results)
;;  (unix:sleep 1)
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :larm-controller)
;;  (send *ri* :dequeue-with-checking-results)
  ;;(unix:sleep 1)
  (send *pr2* :larm :shoulder-p :joint-angle 30);;肩上げる方向
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :larm-controller)
;;  (send *ri* :dequeue-with-checking-results)
  ;;(unix:sleep 1)
)

(defun torso (&key (wz 50))
  (send *pr2* :torso :waist-z :joint-angle wz)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector ) 2000 :torso-controller))

(defun head (&key (np 0))
  (send *pr2* :head :neck-p :joint-angle np)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :head-controller)
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector-parallel (send *pr2* :angle-vector) 2000 :head-controller)
)

(defun reset (&key (arg 1))
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
  (send *ri* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9893 -0.184653 -15.2376 113.545 -92.2444 165.71 -58.6879 -19.272 -59.9531 74.0963 -69.9794 -119.891 -19.9977 -29.9993 180.003 -0.006113 -2.37) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9892 23.4666 -16.7838 112.121 -91.473 158.839 -61.0013 -17.532 -59.9484 74.1011 -69.9794 -119.891 -19.9977 -29.9993 180.003 0.005887 -2.346) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector #f(49.9892 25.5092 -3.7892 36.3598 -114.458 162.83 -42.9229 -88.245 -59.9531 74.0963 -69.9794 -119.899 -19.9977 -29.9993 180.003 -0.000113 -2.382) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)
  )

(defun demo()
  (progn
    (rarm :sp 30 :sr 30)
    (larm :sp 30 :sr -30)
;;    (torso 200)
    (head )
    (reset)
    (rarm )
    (larm )
    (catch)
    (move)
    (rarm)
    (nagiharai)
    (reset)
    (head )
    (rarm )
    (larm )))
  ;;  (torso 150)))
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :rarm-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :head-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :torso-controller)
  ;;これらをmapcarで順にやらせているのが通常の:angle-vector