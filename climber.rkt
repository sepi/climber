#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         rackunit
         "dualshock.rkt"
         "rigid-body.rkt"
         "posn+.rkt")

(define WIDTH 600)
(define HEIGHT 600)
(define SCENE (empty-scene WIDTH HEIGHT))

(define INITIAL-POSITION (posn (/ WIDTH 2)
                          (/ HEIGHT 2)))
(define INITIAL-ROTATION 0)
(define INITIAL-POSITION-MASS 3)
(define INITIAL-ROTATION-MASS 900)
(define EMPTY-RB
  (rb* INITIAL-POSITION INITIAL-ROTATION
       INITIAL-POSITION-MASS INITIAL-ROTATION-MASS))
(define DT (/ 1 60.0))

(define K 1400.0)
(define BETA 80.0)
(define GRAVITY-FORCE 50000)

(define LIMB-KEYS '(sh-l sh-r hi-l hi-r))
(define-struct climber-state (torso limbs time)
  #:transparent)

(define-struct limb (rest-length tip)
  #:transparent)

(define TIP-OFFSET-MAP
  `((sh-l . ,(posn -40 -30))
    (sh-r . ,(posn 40 -30))
    (hi-l . ,(posn -40 30))
    (hi-r . ,(posn 40 30))))

(define TIPS
  (map (lambda (key)
         (cons key (limb 20 (posn-add (dict-ref TIP-OFFSET-MAP key)
                                      INITIAL-POSITION))))
       LIMB-KEYS))

(define (initialize-rb an-rb)
  (let ([foo (lambda (x y)
               (let ([torso-pos (rb-position an-rb)])
                 (rb-hook* (posn x y))))])
    (rb/set-hook
     (rb/set-hook
      (rb/set-hook
       (rb/set-hook
        (rb/set-hook
         (rb/set-hook
          an-rb
          'sh-l (foo -10 -20))
         'sh-r (foo 10 -20))
        'hi-l (foo -7 20))
       'hi-r (foo 7 20))
      'cg (rb-hook (posn 0 0) (posn 0 GRAVITY-FORCE)))
     'head (foo 0 -35))))

(define (initialize-state empty-rb)
  (climber-state (initialize-rb empty-rb)
                 TIPS
                 0))

(define (find-limb state key)
  (dict-ref (climber-state-limbs state) key))

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
         [limb-tips (map (λ (key) (limb-tip (find-limb state key)))
                         LIMB-KEYS)]
         [image2 (foldl (λ (limb-tip image)
                          (place-image (circle 4 'solid "blue")
                                       (posn-x limb-tip) (posn-y limb-tip)
                                       image))
                        image1
                        limb-tips)]
         [hook-vels (map (λ (key) (rb/find-hook-velocity an-rb key))
                         LIMB-KEYS)]
         [image3 (foldl (λ (key limb-tip hook-posn image)
                          (render-limb image
                                       hook-posn limb-tip
                                       25 25
                                       (knee-direction key)))
                 image2
                 LIMB-KEYS
                 limb-tips
                 hook-positions)]
         [head-hook (rb/find-hook-position (climber-state-torso state)
                                           'head)]
         [image4 (place-image (circle 10 'outline "black")
                              (posn-x head-hook) (posn-y head-hook)
                              image3)])
    image4))

(define (pad-posns)
  (values (posn (pad-stick 'left 'x) (pad-stick 'left 'y))
          (posn (pad-stick 'right 'x) (pad-stick 'right 'y))))

(define (climber-state-set-tip-posn state key a-posn)
  (let* ([limbs (climber-state-limbs state)]
         [limb+ (find-limb state key)]
         [limb-tip-orig (posn-add INITIAL-POSITION (dict-ref TIP-OFFSET-MAP key))]
         [limb-new (struct-copy limb limb+
                                [tip (posn-add a-posn limb-tip-orig)])]
         )
    (struct-copy climber-state state
                 [limbs (dict-set limbs key limb-new)])))

(define (tick-state state)
  (let-values ([(stick-l stick-r) (pad-posns)])
    (let* ([dt DT]
           [cur-time (climber-state-time state)]
           [torso-rb (climber-state-torso state)])
      (climber-state-set-tip-posn
       (climber-state-set-tip-posn
        (struct-copy climber-state
                     state
                     [time (+ cur-time dt)]
                     [torso (update-torso state dt)])
        'sh-l stick-l)
       'sh-r stick-r))))

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
    (rb/integrate
     (climber-state-torso cs1) dt)))

(define (line canvas from to color)
  (scene+line canvas
               (posn-x from) (posn-y from)
               (posn-x to) (posn-y to)
               color))

(define (knee-direction key)
  (cond [(equal? key 'sh-l) -1]
        [(equal? key 'sh-r) 1]
        [(equal? key 'hi-l) 1]
        [(equal? key 'hi-r) -1]))

(define (knee-posn hook tip lower-limb-length upper-limb-length knee-dir)
  (let* ([arm (posn-subtract tip hook)]
         [arm-len (posn-len arm)]
         [arm-norm (posn-normalize arm)]
         [A upper-limb-length]
         [B arm-len]
         [C lower-limb-length]
         [alpha (acos (/ (+ (sqr B) (sqr C) (- (sqr A)))
                         (* 2 B C)))])
    (if (real? alpha)
        (posn-add hook
                  (posn-scale lower-limb-length
                              (posn-rot origin (radians->degrees (* knee-dir alpha)) arm-norm)))
        tip)))

(define (render-limb canvas hook tip lower-limb-length upper-limb-length knee-dir)
  (let ([knee (knee-posn hook tip lower-limb-length upper-limb-length knee-dir)])
    (line (line canvas hook knee "black")
          knee tip "black")))

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

(big-bang (initialize-state EMPTY-RB)
          [to-draw draw-state WIDTH HEIGHT]
          [on-tick tick-state DT]
          [on-key  key-state])
