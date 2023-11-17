
;************************************************
;*                                              *
;*      ros_clips.clp                           *
;*                                              *
;*      Jesus Savage                            *
;*                                              *
;*              Bio-Robotics Laboratory         *
;*                                              *
;*                                              *
;************************************************


(defrule clips-alive
        ?f <- (alive clips)
        =>
        (retract ?f)
        (printout t "ROS clips alive ROS")
)

(defrule send-ros
	(declare (salience 100))
	?f <-  (step ?n)
	?f1 <- (send-ROS ?system ?action ?command ?t ?num)
	=>
        (retract ?f ?f1);saca de la lista de echos porque ya pasó
        (printout t "ROS " ?system " " ?action " " ?command " " ?t " " ?num " ROS")
)

(defrule step-ros
	?f <- (step ?num);Si ocurre un paso x
	=>
	(retract ?f);Sacalo
	(printout t "ROS step " ?num " ROS")
)
