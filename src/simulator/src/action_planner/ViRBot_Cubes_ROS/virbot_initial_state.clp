
;************************************************
;*						*
;*	Initial state 				*
;*						*
;*                      J.Savage, UNAM          *
;*						*
;*                      1/5/20                  *
;*                                              *
;************************************************




(deffacts Initial-state-objects-rooms-zones-actors

; Rooms definitions
	( Room (name kitchen) (zone fridge_zone)							(zones dummy1 frontexit frontentrance fridge_zone dummy2)(center 1.70 1.60))
	( Room (name service) (zone s_table_zone)			(zones dummy1 frontexit frontentrance s_table_zone dummy2)(center 1.65 0.55))
	( Room (name corridor)(zone entry)							(zones dummy1 frontexit frontentrance entry dummy2)(center 0.5 1.05))
	( Room (name bedrom)(zone bedrom_zone)							(zones dummy1 frontexit frontentrance bedrom_zone dummy2)(center 1 0.6))
	( Room (name studio)(zone studio_zone)							(zones dummy1 frontexit frontentrance studio_zone dummy2)(center 0.4 0.4))

; Objects definitions

	( item (type Objects) (name Appl)	(room corridor)(zone entry)	(image Appl)	(attributes pick)(pose 0.46 1.12 0.0))
	( item (type Objects) (name Sush)	(room corridor)(zone entry)	(image Sush)	(attributes pick)(pose 0.32 1.12 0.0))
	( item (type Objects) (name Milk)	(room corridor)(zone entry)	(image Milk)	(attributes pick)(pose 0.18 1.12 0.0))
	( item (type Objects) (name Soap)	(room corridor)(zone entry)	(image Soap)	(attributes pick)(pose 0.18 0.97 0.0))
	( item (type Objects) (name Perf)	(room corridor)(zone entry)	(image Perf)	(attributes pick)(pose 0.32 0.97 0.0))
	( item (type Objects) (name Sham)	(room corridor)(zone entry)	(image Sham)	(attributes pick)(pose 0.46 0.97 0.0))
	( item (type Objects) (name Book)	(room corridor)(zone entry)	(image Book)	(attributes pick)(pose 0.60 0.97 0.0))
	( item (type Objects) (name Hamm)	(room corridor)(zone entry)	(image Hamm)	(attributes pick)(pose 0.74 0.97 0.0))
	( item (type Objects) (name freespace)(room any)(zone any)(image none)(attributes none)(pose 0.0 0.0 0.0))

	; Robots definitions
	( item (type Robot) (name robot)(zone frontexit)(pose 2.05 2.05 0.0))

	; Furniture definitions
	( item (type Furniture) (name fridge)					(room kitchen)	(zone fridge_zone)	(image fridge)	( attributes no-pick brown)(pose 1.50 1.50 0.0))
	( item (type Furniture) (name service_table)	(room service)	(zone s_table_zone)	(image table)		( attributes no-pick brown)(pose 1.65 0.30 0.0))
	( item (type Furniture) (name bed)	(room bedrom)	(zone bedrom_zone)	(image bed)		( attributes no-pick brown)(pose 1 0.60 0.0))
	( item (type Furniture) (name desk)	(room studio)	(zone studio_zone)	(image desk)		( attributes no-pick brown)(pose 0.4 0.4 0.0))

	; Doors definitions
	( item (type Door) (name outsidedoor) (status closed) )
	( Arm (name left))

	;Posición de los objetos, el primero está en la cima
	(stack corridor entry entryObj Appl Sush Milk)
	(stack corridor entry entryObj Soap Perf Sham)
	; (stack corridor entry entryObj Hamm Book)

	(real-stack corridor entry entryObj Appl Sush Milk)
	(real-stack corridor entry entryObj Soap Perf Sham)
	(real-stack corridor entry entryObj Hamm Book)

	;A donde deben ser llevados, el último elemento es el tope de la pila (primero en llevar)
	(goal-stack 2 service s_table_zone service_table Soap Perf Sham)
	(goal-stack 1 kitchen fridge_zone fridge Sush Appl Milk)
	;(goal-stack 2 bedrom bedrom_zone bed Book)
	;(goal-stack 2 studio studio_zone desk Hamm)

	;No sé para qué, pero si no está no se mueve
	(plan (name cubes) (number 0)(duration 0))
)
