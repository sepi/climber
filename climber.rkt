#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         rackunit
         threading
         "dualshock.rkt"
         "rigid-body.rkt"
         "posn+.rkt")

(define-struct climber-state (torso limbs time)
  #:transparent)

(define-struct limb (rest-length tip)
  #:transparent)

(define WIDTH 600)
(define HEIGHT 600)
(define SCENE (empty-scene WIDTH HEIGHT))

(define INITIAL-POSITION (posn (/ WIDTH 2)
                          (/ HEIGHT 2)))
(define INITIAL-ROTATION 0)
(define INITIAL-POSITION-MASS 3)
(define INITIAL-ROTATION-MASS 900)
(define INITIAL-LIMB-LENGTH 20)
(define EMPTY-RB
  (rb* INITIAL-POSITION INITIAL-ROTATION
       INITIAL-POSITION-MASS INITIAL-ROTATION-MASS))
(define DT (/ 1 60.0))

(define K 1400.0)
(define BETA 80.0)
(define GRAVITY-FORCE 50000)

(define LIMB-KEYS '(sh-l sh-r hi-l hi-r))

(define TIP-OFFSET-MAP
  `((sh-l . ,(posn -40 -30))
    (sh-r . ,(posn 40 -30))
    (hi-l . ,(posn -40 30))
    (hi-r . ,(posn 40 30))))

(define (knee-direction key)
  (dict-ref '((sh-l . -1)
              (sh-r . 1)
              (hi-l . 1)
              (hi-r . -1))
            key))

(define TIPS
  (map (lambda (key)
         (cons key (limb INITIAL-LIMB-LENGTH
                         (posn-add (dict-ref TIP-OFFSET-MAP key)
                                   INITIAL-POSITION))))
       LIMB-KEYS))

(define (initialize-rb an-rb)
  (define (make-hook-from-offset x y)
    (let ([torso-pos (rb-position an-rb)])
      (rb-hook* (posn x y))))
  (~> an-rb
      (rb/set-hook 'sh-l (make-hook-from-offset -10 -20))
      (rb/set-hook 'sh-r (make-hook-from-offset 10 -20))
      (rb/set-hook 'hi-l (make-hook-from-offset -10 20))
      (rb/set-hook 'hi-r (make-hook-from-offset 10 20))
      (rb/set-hook 'cg (make-hook-from-offset 0 0))
      (rb/set-hook 'head (make-hook-from-offset 0 -35))))

(define (line canvas from to color)
  (scene+line canvas
               (posn-x from) (posn-y from)
               (posn-x to) (posn-y to)
               color))

(define (place canvas an-image a-posn)
  (place-image an-image
               (posn-x a-posn) (posn-y a-posn)
               canvas))

(define (pad-stick-posns)
  (values (posn (pad-stick 'left 'x) (pad-stick 'left 'y))
          (posn (pad-stick 'right 'x) (pad-stick 'right 'y))))

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

(define (limb/draw canvas hook tip lower-limb-length upper-limb-length knee-dir)
  (let ([knee (knee-posn hook tip lower-limb-length upper-limb-length knee-dir)])
    (line (line canvas hook knee "black")
          knee tip "black")))

;; climber-state
(define (cs/init empty-rb)
  (climber-state (initialize-rb empty-rb)
                 TIPS
                 0))

(define (cs/find-limb cs key)
  (dict-ref (climber-state-limbs cs) key))

(define (cs/limb-force cs key)
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

(define (cs/draw cs)
  (let* ([an-rb (climber-state-torso cs)]
         [hook-posns (map (λ (key) (rb/find-hook-position an-rb key))
                          LIMB-KEYS)]
         [limb-tip-posns (map (λ (key) (limb-tip (cs/find-limb cs key)))
                              LIMB-KEYS)]
         [hook-vels (map (λ (key) (rb/find-hook-velocity an-rb key))
                         LIMB-KEYS)]
         [head-hook (rb/find-hook-position (climber-state-torso cs)
                                           'head)])
    (~> SCENE
        (foldl (λ (hook-posn image)
                 (place image (circle 3 'outline "black") hook-posn))
               _
               hook-posns)
        (foldl (λ (limb-tip image)
                 (place image (circle 4 'solid "blue") limb-tip))
               _
               limb-tip-posns)
        (foldl (λ (key limb-tip hook-posn image)
                 (limb/draw image
                            hook-posn limb-tip
                            25 25
                            (knee-direction key)))
               _
               LIMB-KEYS
               limb-tip-posns
               hook-posns)
        (place _ (circle 10 'outline "black") head-hook))))

(define (cs/set-limb-tip-posn cs key a-posn)
  (let* ([limbs (climber-state-limbs cs)]
         [limb+ (cs/find-limb cs key)]
         [limb-tip-orig (posn-add INITIAL-POSITION (dict-ref TIP-OFFSET-MAP key))]
         [limb-new (struct-copy limb limb+
                                [tip (posn-add a-posn limb-tip-orig)])])
    (struct-copy climber-state cs
                 [limbs (dict-set limbs key limb-new)])))

(define (cs/update-torso cs dt)
  (let* ([cs1 (foldl (lambda (key state)
                       (struct-copy climber-state state
                                    [torso
                                     (rb/set-hook-force
                                      (climber-state-torso state) key
                                      (cs/limb-force state key))]))
                     cs
                     LIMB-KEYS)])
    (rb/integrate
     (climber-state-torso cs1) dt)))

(define (cs/tick state)
  (let-values ([(stick-l stick-r) (pad-stick-posns)])
    (let* ([dt DT]
           [cur-time (climber-state-time state)]
           [torso-rb (climber-state-torso state)])
      (cs/set-limb-tip-posn
       (cs/set-limb-tip-posn
        (struct-copy climber-state
                     state
                     [time (+ cur-time dt)]
                     [torso (cs/update-torso state dt)])
        'sh-l stick-l)
       'sh-r stick-r))))

(define (cs/key state key)
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

(big-bang (cs/init EMPTY-RB)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT]
          [on-key  cs/key])
