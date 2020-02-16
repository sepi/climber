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

(struct/lens climber-state (sim limbs time)
             #:transparent)

(struct/lens sim (rigid-bodies forces)
             #:transparent)

(struct/lens limb (rest-length tip)
             #:transparent)

; Represents a force generating connection between two rigid-body hooks.
; The calculated force will be applied to both hooks.
; hooks are referenced using paths i.e. pairs (rigid-body-id . hook-id)
; function is a function: sim hook-path hook-path force-state -> force
(struct/lens force (a-hook-path b-hook-path function state)
             #:transparent)

(define climber-state-torso-lens
  (lens-compose (dict-ref-lens 'torso)
                sim-rigid-bodies-lens
                climber-state-sim-lens))

(define WIDTH 600)
(define HEIGHT 600)
(define SCENE (empty-scene WIDTH HEIGHT))

(define TORSO-POSITION (posn (/ WIDTH 2)
                             (/ HEIGHT 2)))

(define TORSO-POSITION-MASS 3)
(define TORSO-ROTATION-MASS 900)

(define INITIAL-LIMB-LENGTH 20)

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

(define LIMB-MAP
  (map (lambda (key)
         (cons key (limb INITIAL-LIMB-LENGTH
                         (posn-add (dict-ref TIP-OFFSET-MAP key)
                                   TORSO-POSITION))))
       LIMB-KEYS))

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

(define (cs/find-limb cs key)
  (dict-ref (climber-state-limbs cs) key))

(define (cs/hook-limb-force hook-posn hook-vel limb)
  (rb/damped-spring-force hook-posn (limb-tip limb)
                          hook-vel (limb-rest-length limb)
                          K BETA))

(define (cs/draw cs)
  (let* ([an-rb (lens-view climber-state-torso-lens cs)]
         [hook-posns (map (λ (key) (rb/find-hook-position an-rb key))
                          LIMB-KEYS)]
         [limb-tip-posns (map (λ (key) (limb-tip (cs/find-limb cs key)))
                              LIMB-KEYS)]
         [hook-vels (map (λ (key) (rb/find-hook-velocity an-rb key))
                         LIMB-KEYS)]
         [head-hook (rb/find-hook-position (lens-view climber-state-torso-lens cs)
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

(define (cs/set-limb-tip-posn cs key limb-tip-offset)
  (let ([limb-tip-orig (posn-add TORSO-POSITION (dict-ref TIP-OFFSET-MAP key))])
    (lens-set (lens-compose limb-tip-lens
                            (dict-ref-lens key)
                            climber-state-limbs-lens)
              cs
              (posn-add limb-tip-orig
                        limb-tip-offset))))

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

(define (initialize-torso)
  (let ([torso (rb* 'torso
                    #false
                    TORSO-POSITION 0
                    TORSO-POSITION-MASS TORSO-ROTATION-MASS)])
    (define (make-hook-from-offset x y)
      (rb-hook* (posn x y)))
    (~> torso
        (lens-set (rb/hook-lens 'sh-l) _ (make-hook-from-offset -10 -20))
        (lens-set (rb/hook-lens 'sh-r) _ (make-hook-from-offset 10 -20))
        (lens-set (rb/hook-lens 'hi-l) _ (make-hook-from-offset -10 20))
        (lens-set (rb/hook-lens 'hi-r) _ (make-hook-from-offset 10 20))
        (lens-set (rb/hook-lens 'cg) _ (make-hook-from-offset 0 0))
        (lens-set (rb/hook-lens 'head) _ (make-hook-from-offset 0 -35)))))

;; Initialize hand or foot rigid-bodies
;; containing one cg hook at the limb-tips origin
(define (initialize-limb-tip key limb-tip-posn posn-mass rot-mass)
  (let ([limb-tip (rb* key #true limb-tip-posn 0 posn-mass rot-mass)])
    (lens-set (rb/hook-lens 'cg)
              limb-tip
              origin)))

(define (sim-hook-lens path)
  (match path
    [(cons rb-id hook-id)
     (lens-compose (dict-ref-lens hook-id)
                   rb-hooks-lens
                   (dict-ref-lens rb-id)
                   sim-rigid-bodies)]))

(define (rest-length _) INITIAL-LIMB-LENGTH)
(define (damped-spring-force sim a-hook-path b-hook-path spring-state)
  (let* ([a-rb (dict-ref (sim-rigid-bodies sim) (first a-hook-path))]
         [a-hook (lens-view (sim-hook-lens a-hook-path) sim)]
         [a-hook-posn (rb/find-hook-position a-rb (rest a-hook-path))]
         [b-rb (dict-ref (sim-rigid-bodies sim) (first b-hook-path))]
         [b-hook (lens-view (sim-hook-lens b-hook-path) sim)]
         [b-hook-posn (rb/find-hook-position b-rb (rest b-hook-path))])
     (rb/damped-spring-force a-hook-posn b-hook-posn
                             (rb/find-hook-velocity a-rb (rest a-hook-path))
                             (rest-length spring-state)
                             K BETA)))

(define (gravity-force _1 _2 _3 _4)
  (posn 0 GRAVITY-FORCE))

;; climber-state
(define (cs/init)
  (let ([torso (initialize-torso)]
        [hand-l (initialize-limb-tip 'hand-l (dict-ref TIP-OFFSET-MAP 'sh-l) 1 0)]
        [hand-r (initialize-limb-tip 'hand-r (dict-ref TIP-OFFSET-MAP 'sh-r) 1 0)]
        [foot-l (initialize-limb-tip 'foot-l (dict-ref TIP-OFFSET-MAP 'hi-l) 1 0)]
        [foot-r (initialize-limb-tip 'foot-r (dict-ref TIP-OFFSET-MAP 'hi-r) 1 0)])
    (climber-state (sim `((torso . ,torso)
                          (hand-l . ,hand-l)
                          (hand-r . ,hand-r)
                          (foot-l . ,foot-l)
                          (foot-r . ,foot-r))
                        (list (force '(torso . sh-l) '(hand-l . cg)
                                     damped-spring-force '())
                              (force '(torso . sh-r) '(hand-r . cg)
                                     damped-spring-force '())
                              (force '(torso . hi-l) '(foot-l . cg)
                                     damped-spring-force '())
                              (force '(torso . hi-r) '(foot-r . cg)
                                     damped-spring-force '())
                              (force '(torso . cg) '()
                                     gravity-force '())))
                   LIMB-MAP
                   0)))

(big-bang (cs/init)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT])
