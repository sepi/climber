#lang racket

(require racket/dict
         rnrs/base-6
         posn)

(provide rb
         rb-rotation
         rb-position
         rb*
         rb-hook*
         rb/set-hook
         rb/damped-spring-force
         rb/set-hook-force
         rb/find-hook-velocity
         rb/find-hook-position
         rb/integrate)

(define-struct rb (position position-velocity
                   rotation rotation-velocity
                   position-mass
                   rotation-mass
                   hooks)
  #:transparent)

(define (rb* position rotation position-mass rotation-mass)
  (rb position (posn 0 0) rotation 0 position-mass rotation-mass '()))

(define-struct rb-hook (position f)
  #:transparent)

(define (rb-hook* position)
  (rb-hook position origin))

(define (rb/set-hook an-rb key hook)
  (struct-copy rb an-rb
               [hooks (dict-set (rb-hooks an-rb) key hook)]))

(define (rb/find-hook an-rb key)
  (dict-ref (rb-hooks an-rb) key))

(define (rb/rot anchor rotation point)
  (posn-rotate-ccw anchor (mod rotation 360) point))

; find hook position in global coordinates
(define (rb/find-hook-position an-rb key)
  (rb/transform-hook-position an-rb (rb/find-hook an-rb key)))

(define (rb/transform-hook-position an-rb a-hook)
  (let* ([cm (rb-position an-rb)]
         [rot (rb-rotation an-rb)]
         [hook-position (rb-hook-position a-hook)])
    (posn-add cm
              (rb/rot origin rot hook-position))))

(define (rb/transform-hook-velocity an-rb a-hook)
  (let* ([cm (rb-position an-rb)]
         [alpha (rb-rotation an-rb)]
         [rot-vel (rb-rotation-velocity an-rb)]
         [vec-rot (rb/rot origin (+ alpha 90) (posn 1 0))]
         [rb-vel (rb-position-velocity an-rb)])
    (posn-add (posn-scale rot-vel vec-rot)
              rb-vel)))

(define (rb/find-hook-force an-rb key)
  (rb-hook-f (rb/find-hook an-rb key)))

(define (rb/find-hook-velocity an-rb key)
  (rb/transform-hook-velocity an-rb (rb/find-hook an-rb key)))

(define (posn-dotp a b)
  (+ (* (posn-x a) (posn-x b))
     (* (posn-y a) (posn-y b))))

; hook->tip O----c
(define (rb/damped-spring-force a b a-velocity base-len k beta)
  (let* ([ab (posn-subtract b a)]
         [ab-len (posn-len ab)]
         [len-diff (- base-len ab-len)]
         [ab-norm (posn-normalize ab)]
         [a-velocity-len (posn-len a-velocity)]
         [a-velocity-proj (posn-dotp ab-norm a-velocity)]
         
         [force-factor (+ (- (* k len-diff))
                          (- (* beta a-velocity-proj)))])
    (posn-scale force-factor ab-norm)))

(define (rb/set-hook-force an-rb key frce)
  (rb/set-hook an-rb key
               (struct-copy rb-hook (rb/find-hook an-rb key)
                            [f frce])))

(define (rb/torque lever frce)
  (* (posn-len lever)
     (posn-len frce)
     (sin (to-rad (posn-angle-diff lever frce)))))

(define (rb/resultant-force an-rb)
  (sequence-fold (lambda (f-sum b-hook)
                   (posn-add f-sum (rb-hook-f b-hook)))
                 (posn 0 0)
                 (in-dict-values (rb-hooks an-rb))))

(define (rb/resultant-torque an-rb)
  (sequence-fold (lambda (torque hook)
                   (let* ([cm (rb-position an-rb)]
                          [hook-posn (rb/transform-hook-position an-rb hook)]
                          [lever (posn-subtract hook-posn cm)])
                     ;(display lever)
                     (+ torque
                        (rb/torque lever (rb-hook-f hook)))))
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
    ;(displayln rot-p-new)
    (struct-copy rb an-rb
                 [position-velocity v-new]
                 [position p-new]
                 [rotation-velocity rot-v-new]
                 [rotation rot-p-new]
                 )))

(define (posn-len a-posn)
  (sqrt (+ (* (posn-x a-posn) (posn-x a-posn))
           (* (posn-y a-posn) (posn-y a-posn)))))

(define (posn-normalize a-posn)
  (posn-scale (/ 1 (posn-len a-posn)) a-posn))

(define (posn-abs a-posn)
  (posn (abs (posn-x a-posn)) (abs (posn-y a-posn))))

(define (to-deg rad)
  (* 180 (/ 1 pi) rad))

(define (to-rad deg)
  (* pi (/ 1 180) deg))

(define (posn-angle a-posn)
  (if (equal? a-posn (posn 0 0))
      0
      (to-deg (atan (posn-y a-posn) (posn-x a-posn)))))

(define (posn-angle-diff a-posn b-posn)
  (- (posn-angle b-posn)
     (posn-angle a-posn)))
