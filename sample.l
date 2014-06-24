(load "package://pr2eus/pr2-interface.l")
(pr2) ;;(setq *pr2* (instance pr2-robot :init)) pr2.l
(pr2-init);;(setq *ri* (instance pr2-interface :init))etc...
(defun rarm (&key (sp 0)(sr 0))
  (send *pr2* :rarm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :rarm :shoulder-r :joint-angle sr);;回外回内方向
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :rarm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :rarm :shoulder-p :joint-angle 20);;肩上げる方向
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :rarm-controller)
)

(defun larm (&key (sp 0)(sr 0))
  (send *pr2* :larm :shoulder-p :joint-angle sp);;肩上げる方向
  (send *pr2* :larm :shoulder-r :joint-angle sr);;;回外回内方向
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :shoulder-p :joint-angle 20);;肩上げる方向
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :larm-controller)
)

(defun torso (&key (wz 50))
  (send *pr2* :torso :waist-z :joint-angle wz)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :torso-controller))

(defun head (&key (np 0))
  (send *pr2* :head :neck-p :joint-angle np)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)
  (send *pr2* :head :neck-p :joint-angle 50)
  (send *ri* :wait-interpolation)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000 :head-controller)
)

(defun reset (&key (arg 1))
  (send *pr2* :reset-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector ) 2000)
)

(defun demo()
  (progn
    (rarm :sp 30 :sr 30)
    (larm :sp 30 :sr -30)
;;    (torso 200)
    (head 60)
    (reset)
    (rarm )
    (larm )
    (reset)
    (head -30)
    (rarm )
    (larm )
  ;;  (torso 150)))
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :rarm-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :larm-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :head-controller)
  ;;(send *ri* :angle-vector (send *pr2* :angle-vector ) 2000 :torso-controller)
  ;;これらをmapcarで順にやらせているのが通常の:angle-vector