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
   ()
   (let ((qtable (get self :angle-vector-queue-table)))
     (mapcar #'(lambda (k v)
                 (when (not (find actionlib_msgs::GoalStatus::*ACTIVE*
                                  (send-all (gethash k controller-table) :get-state)))
                   (let ((avs-tms (send v :pop)))
                     (send self :angle-vector-sequence (car avs-tms) (cdr avs-tms) k))))
             (send qtable :list-keys) (send qtable :list-values))))
  )
