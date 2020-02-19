#lang racket

(require racket/dict
         posn
         lens
         "posn+.rkt")

(provide rb
         rb-hooks-lens
         rb-hooks
         rb-static

         hook
         hook-f-lens

         rb/hook-lens
         rb/damped-spring-force
         rb/find-hook-velocity
         rb/find-hook-position
         rb/integrate)

(struct/lens rb (id static
                 position position-velocity
                 rotation rotation-velocity
                 position-mass
                 rotation-mass
                 hooks)
  #:transparent)

(struct/lens hook (position f)
  #:transparent)

(define (rb/hook-lens key)
  (lens-compose (dict-ref-lens key)
                rb-hooks-lens))

; find hook position in global coordinates
(define (rb/find-hook-position an-rb key)
  (rb/transform-hook-position an-rb
                              (lens-view (rb/hook-lens key) an-rb)))

; find hook velocity in global coordinates
(define (rb/find-hook-velocity an-rb key)
  (rb/transform-hook-velocity an-rb
                              (lens-view (rb/hook-lens key) an-rb)))

(define (rb/transform-hook-position an-rb a-hook)
  (let* ([cm (rb-position an-rb)]
         [rot (rb-rotation an-rb)]
         [hook-position (hook-position a-hook)])
    (posn-add cm
              (posn-rot origin rot hook-position))))

(define (rb/transform-hook-velocity an-rb a-hook)
  (let* ([cm (rb-position an-rb)]
         [alpha (rb-rotation an-rb)]
         [rot-vel (rb-rotation-velocity an-rb)]
         [vec-rot (posn-rot origin (+ alpha 90) (posn 1 0))]
         [rb-vel (rb-position-velocity an-rb)])
    (posn-add (posn-scale rot-vel vec-rot)
              rb-vel)))

; hook->tip O----c
(define (rb/damped-spring-force a b a-velocity base-len k beta)
  (let* ([ab (posn-subtract b a)]
         [ab-len (posn-len ab)]
         [len-diff (- base-len ab-len)]
         [ab-norm (posn-normalize ab)]
         [a-velocity-len (posn-len a-velocity)]
         [a-velocity-proj (posn-dotp ab-norm a-velocity)]
         [k-part (- (* k len-diff))]
         ;[k-part-eff (* (sgn k-part) (sqr k-part) 0.00001)] ; Quadratic spring
         [beta-part (- (* beta a-velocity-proj))]
         [force-factor (+ k-part beta-part)])
    (posn-scale force-factor
                ab-norm)))

(define (rb/torque lever frce)
  (* (posn-len lever)
     (posn-len frce)
     (sin (degrees->radians (posn-angle-diff lever frce)))))

(define (rb/resultant-force an-rb)
  (sequence-fold (lambda (f-sum b-hook)
                   (posn-add f-sum (hook-f b-hook)))
                 (posn 0 0)
                 (in-dict-values (rb-hooks an-rb))))

(define (rb/resultant-torque an-rb)
  (sequence-fold (lambda (torque hook)
                   (let* ([cm (rb-position an-rb)]
                          [hook-posn (rb/transform-hook-position an-rb hook)]
                          [lever (posn-subtract hook-posn cm)])
                     (+ torque
                        (rb/torque lever (hook-f hook)))))
                 0
                 (in-dict-values (rb-hooks an-rb))))

(define (rb/integrate an-rb dt)
  (let* ([f (rb/resultant-force an-rb)]
         [a (posn-scale (/ 1 (rb-position-mass an-rb)) f)]
         [dv (posn-scale dt a)]
         [v (rb-position-velocity an-rb)]
         [v-new (posn-add v dv)]
         [dp (posn-scale dt v-new)]
         [p (rb-position an-rb)]
         [p-new (posn-add p dp)]

         [rot-v (rb-rotation-velocity an-rb)]
         [rot-f (rb/resultant-torque an-rb)]
         [rot-a (/ rot-f (rb-rotation-mass an-rb))]
         [rot-dv (* rot-a dt)]
         [rot-v-new (* 0.90 (+ rot-v rot-dv))]
         [rot-dp (* rot-v-new dt)]
         [rot-p (rb-rotation an-rb)]
         [rot-p-new (+ rot-p rot-dp)])
    (struct-copy rb an-rb
                 [position-velocity v-new]
                 [position p-new]
                 [rotation-velocity rot-v-new]
                 [rotation rot-p-new]
                 )))
