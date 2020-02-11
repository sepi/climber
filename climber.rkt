#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         "rigid-body.rkt")

(define WIDTH 300)
(define HEIGHT 300)
(define SCENE (empty-scene WIDTH HEIGHT))

(define INITIAL-POS (posn (/ WIDTH 2)
                          (/ HEIGHT 2)))
(define INITIAL-ROT 0)
(define INITIAL-MASS 3)
(define INITIAL-ROT-MASS 900)
(define EMPTY-RB (rb* (posn-add (posn 17 0) INITIAL-POS) INITIAL-ROT INITIAL-MASS INITIAL-ROT-MASS))
(define DT (/ 1 60.0))

(define K 1400.0)
(define BETA 80.0)

(define LIMB-KEYS '(sh-l sh-r hi-l hi-r))
(define-struct climber-state (torso limbs time)
  #:transparent)

(define-struct limb (rest-length tip)
  #:transparent)

(define TIPS
  (map (lambda (key pos)
         (cons key (limb 20 (posn-add pos INITIAL-POS))))
       LIMB-KEYS
       (list (posn -40 -30)
             (posn 40 -30)
             (posn -40 30)
             (posn 40 30)
             )))

(define (initialize-rb an-rb)
  (let ([foo (lambda (x y)
               (let ([torso-pos (rb-position an-rb)])
                 (rb-hook* (posn x y))))])
    (rb/set-hook
     (rb/set-hook
      (rb/set-hook
       (rb/set-hook
         an-rb
        'sh-l (foo -10 -20))
       'sh-r (foo 10 -20))
      'hi-l (foo -7 20))
     'hi-r (foo 7 20))))

(define (initialize-state empty-rb)
  (climber-state
   (initialize-rb empty-rb)
   TIPS
   0))

(define (draw-state state)
  (let* ([an-rb (climber-state-torso state)]
         [hook-positions (map (λ (key) (rb/find-hook-position an-rb key))
                              LIMB-KEYS)]
         [image1 (foldl (λ (hook-posn image)
                          (place-image (circle 3 'outline "black")
                                       (posn-x hook-posn) (posn-y hook-posn)
                                       image))
                        SCENE
                        hook-positions)]
         [limb-tips (map (λ (key) (limb-tip (dict-ref (climber-state-limbs state) key)))
                         LIMB-KEYS)]
         [image2 (foldl (λ (limb-tip image)
                          (place-image (circle 4 'solid "blue")
                                       (posn-x limb-tip) (posn-y limb-tip)
                                       image))
                        image1
                        limb-tips)]
         [image3 (foldl (λ (limb-tip hook-posn image)
                          (scene+line image
                                      (posn-x limb-tip) (posn-y limb-tip)
                                      (posn-x hook-posn) (posn-y hook-posn)
                                      "black"))
                        image2
                        limb-tips
                        hook-positions)]
         [hook-vels (map (λ (key) (rb/find-hook-velocity an-rb key))
                         LIMB-KEYS)]
         [image4 (foldl (λ (limb-tip hook-posn hook-vel image)
                          (let* ([f (rb/damped-spring-force hook-posn limb-tip hook-vel 0 K BETA)]
                                 [hook+vel (posn-add hook-posn (posn-scale 10 hook-vel))]
                                 [hook+f (posn-add hook-posn (posn-scale 0.001 f))])
                            (place-image (circle 1 'solid "red")
                                         (posn-x hook+f) (posn-y hook+f)
                                         ;(posn-x hook+vel) (posn-y hook+vel)
                             
                             (scene+line image
                                        (posn-x hook-posn) (posn-y hook-posn)
                                        (posn-x hook+f) (posn-y hook+f)
                                        ;(posn-x hook+vel) (posn-y hook+vel)
                                        "green")
                            
                            )))
                        image3
                        limb-tips
                        hook-positions
                        hook-vels)])
    image3))

(define (tick-state state)
  (let* ([dt DT]
         [cur-time (climber-state-time state)]
         [torso-rb (climber-state-torso state)])
    (struct-copy climber-state
                 state
                 [time (+ cur-time dt)]
                 [torso (update-torso state dt)])))

(define (limb-force cs key)
  (let* ([torso (climber-state-torso cs)]
         [hook-posn (rb/find-hook-position torso key)]
         [hook-vel (rb/find-hook-velocity torso key)]
         [limbs (climber-state-limbs cs)]
         [limb-posn (limb-tip (dict-ref limbs key))]
         [limb-rest-len (limb-rest-length (dict-ref limbs key))]
         [force (rb/damped-spring-force hook-posn limb-posn
                                        hook-vel limb-rest-len
                                        K BETA)])
    force))

; actually update-state
(define (update-torso cs dt)
  (let* ([cs1 (foldl (lambda (key state)
                       (struct-copy climber-state state
                                    [torso
                                     (rb/set-hook-force
                                      (climber-state-torso state) key
                                      (limb-force state key))]))
                     cs
                     LIMB-KEYS)])
    ;(displayln cs1)
    (rb/integrate
     (climber-state-torso cs1) dt)))

(define (key-state state key)
  (cond [(key=? key "left")
         (struct-copy climber-state state
                      [torso (rb/set-hook-force
                              (climber-state-torso state) 'sh-l (posn -100 0))])]
        [(key=? key "right")
         (struct-copy climber-state state
                      [torso (rb/set-hook-force
                              (climber-state-torso state) 'sh-r (posn 100 0))])]
        [else
         (struct-copy climber-state state
                      [torso (rb/set-hook-force
                              (rb/set-hook-force
                               (climber-state-torso state) 'sh-l origin)
                              'sh-r origin)])
         ]))

(define (mouse-state state x y ev)
  (let* ([limbs (climber-state-limbs state)]
         [limb+ (dict-ref limbs 'sh-r)]
         [limb-new (struct-copy limb limb+
                                [tip (posn x y)])])
    (struct-copy climber-state state
                 [limbs (dict-set limbs 'sh-r limb-new)])))

(big-bang (initialize-state EMPTY-RB)
          [to-draw draw-state WIDTH HEIGHT]
          [on-tick tick-state DT]
          [on-key  key-state]
          [on-mouse mouse-state])
