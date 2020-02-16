#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         rackunit
         threading
         lens
         "dualshock.rkt"
         "rigid-body.rkt"
         "posn+.rkt")

(struct/lens climber-state (torso limbs time)
  #:transparent)

(struct/lens limb (rest-length tip)
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
      (lens-set (rb/hook-lens 'sh-l) _ (make-hook-from-offset -10 -20))
      (lens-set (rb/hook-lens 'sh-r) _ (make-hook-from-offset 10 -20))
      (lens-set (rb/hook-lens 'hi-l) _ (make-hook-from-offset -10 20))
      (lens-set (rb/hook-lens 'hi-r) _ (make-hook-from-offset 10 20))
      (lens-set (rb/hook-lens 'cg) _ (make-hook-from-offset 0 0))
      (lens-set (rb/hook-lens 'head) _ (make-hook-from-offset 0 -35))))

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

(define (cs/hook-limb-force hook-posn hook-vel limb)
  (rb/damped-spring-force hook-posn (limb-tip limb)
                          hook-vel (limb-rest-length limb)
                          K BETA))

(define (cs/draw cs)
  (let* ([an-rb (climber-state-torso cs)]
         [hook-posns (map (λ (key) (rb/find-hook-position an-rb key))
                          LIMB-KEYS)]
         [limb-tip-posns (map (λ (key) (limb-tip (cs/find-limb cs key)))
                              LIMB-KEYS)]
         [hook-vels (map (λ (key) (rb/find-hook-velocity an-rb key))
                         LIMB-KEYS)]
         [head-hook (rb/find-hook-position (climber-state-torso cs) 'head)])
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
  (let ([limb-tip-orig (posn-add INITIAL-POSITION (dict-ref TIP-OFFSET-MAP key))])
    (lens-transform (lens-compose limb-tip-lens
                                  (dict-ref-lens key)
                                  climber-state-limbs-lens)
                    cs
                    (λ (tip-posn) (posn-add a-posn limb-tip-orig)))))

(define (cs/set-hook-force cs key f)
  (lens-set (lens-compose rb-hook-f-lens
                          (dict-ref-lens key)
                          rb-hooks-lens
                          climber-state-torso-lens)
            cs
            f))

(define (cs/apply-limb-force torso limbs key)
  (lens-set (lens-compose rb-hook-f-lens
                          (dict-ref-lens key)
                          rb-hooks-lens)
            torso
            (cs/hook-limb-force (rb/find-hook-position torso key)
                                (rb/find-hook-velocity torso key)
                                (dict-ref limbs key))))

;; Apply limb forces to hooks
(define (cs/apply-limb-forces torso limbs)
  (foldl (λ (key t) (cs/apply-limb-force t limbs key))
         torso
         LIMB-KEYS))

(define (cs/tick cs)
  (let-values ([(stick-l stick-r) (pad-stick-posns)])
    (~> cs
        (lens-transform climber-state-torso-lens ; update torso force
                        _
                        (λ (t) (cs/apply-limb-forces t (climber-state-limbs cs))))
        (lens-transform climber-state-torso-lens ; integrate torso posn/rot
                        _
                        (λ (t) (rb/integrate t DT)))
        (lens-transform climber-state-time-lens ; update cs time
                        _
                        (λ (t) (+ t DT)))
        (cs/set-limb-tip-posn _ 'sh-l stick-l)
        (cs/set-limb-tip-posn _ 'sh-r stick-r))))

(big-bang (cs/init EMPTY-RB)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT])
