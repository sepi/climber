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
(define K 1.0)
(define BETA 80.0)
(define GRAVITY-FORCE 5000)

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

(define (draw-rb rb canvas)
  (for/fold ([canvas canvas])
            ([key (in-dict-keys (rb-hooks rb))])
    (place canvas (circle 3 'outline "black") (rb/find-hook-position rb key))))

(define (draw-force f canvas sim)
  (let ([a-path (force-a-hook-path f)]
        [b-path (force-b-hook-path f)])
    (if b-path
        (let ([from (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car a-path))
                                           (cdr a-path))]
              [to   (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car b-path))
                                           (cdr b-path))])
          (line canvas from to "blue"))
        canvas)))

(define (cs/draw cs)
  (let* ([sim (climber-state-sim cs)])
    (~> SCENE
        (foldl draw-rb _ (dict-values (sim-rigid-bodies sim)))
        (foldl (lambda (f canvas) (draw-force f canvas sim))
               _ (sim-forces sim)))))
  #;(let* ([an-rb (lens-view climber-state-torso-lens cs)]
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
        (place _ (circle 10 'outline "black") head-hook)))

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

(define (cs/apply-limb-force rb limbs key)
  (lens-set (lens-compose rb-hook-f-lens
                          (dict-ref-lens key)
                          rb-hooks-lens)
            rb
            (cs/hook-limb-force (rb/find-hook-position rb key)
                                (rb/find-hook-velocity rb key)
                                (dict-ref limbs key))))

;; Apply limb forces to hooks
(define (cs/apply-limb-forces rb limbs)
  (foldl (λ (key t) (cs/apply-limb-force t limbs key))
         rb
         LIMB-KEYS))

(define (sim-update-force force sim)
  (let* ([a-hook-path (force-a-hook-path force)]
         [b-hook-path (force-b-hook-path force)]
         [force-fn (force-function force)]
         [fs (force-state force)]
         [force-value (force-fn sim a-hook-path b-hook-path fs)]
         ;; Update a force
         [sim1 (lens-transform (sim-hook-f-lens a-hook-path)
                               sim
                               (lambda (f-old)
                                 (posn-add f-old
                                           force-value)))])
    ;; Update b force
    (if b-hook-path
        (lens-transform (sim-hook-f-lens b-hook-path)
                        sim1
                        (lambda (f-old)
                          (posn-add f-old
                                    (posn-negate force-value))))
        sim1)))

(define (sim-update-forces sim)
  (foldl sim-update-force
         sim
         (lens-view sim-forces-lens sim)))

(define (sim-reset-forces sim)
  (foldl (λ (force sim*)
           (lens-set (sim-hook-f-lens (force-a-hook-path force))
                     sim
                     origin))
         sim
         (lens-view sim-forces-lens sim)))

(define (cs/tick cs)
  (let-values ([(stick-l stick-r) (pad-stick-posns)])
    (~> cs
        (lens-transform climber-state-sim-lens _ sim-reset-forces)
        (lens-transform climber-state-sim-lens _ sim-update-forces)
        ;; TODO integrate all non-static rigid-bodies, not only torso
        (lens-transform climber-state-torso-lens _
                        (λ (torso) (rb/integrate torso DT)))
        (lens-transform climber-state-time-lens _
                        (λ (t) (+ t DT)))
        (cs/set-limb-tip-posn _ 'sh-l stick-l)
        (cs/set-limb-tip-posn _ 'sh-r stick-r))))

(define (initialize-torso)
  (let ([torso (rb 'torso
                   #false
                   TORSO-POSITION origin
                   0 0
                   TORSO-POSITION-MASS TORSO-ROTATION-MASS
                   '())])
    (define (make-hook-from-offset x y)
      (rb-hook (posn x y) origin))
    (~> torso
        (lens-set (rb/hook-lens 'sh-l) _ (make-hook-from-offset -10 -20))
        (lens-set (rb/hook-lens 'sh-r) _ (make-hook-from-offset 10 -20))
        (lens-set (rb/hook-lens 'hi-l) _ (make-hook-from-offset -10 20))
        (lens-set (rb/hook-lens 'hi-r) _ (make-hook-from-offset 10 20))
        (lens-set (rb/hook-lens 'cg) _ (make-hook-from-offset 0 0))
        (lens-set (rb/hook-lens 'head) _ (make-hook-from-offset 0 -35)))))

;; Initialize hand or foot rigid-bodies
;; containing one cg hook at the limb-tips origin
(define (initialize-limb-tip key limb-tip-offset posn-mass rot-mass)
  (let ([limb-tip (rb key #true
                      (posn-add limb-tip-offset TORSO-POSITION) origin
                      0 0
                      posn-mass rot-mass
                      '())])
    (lens-set (rb/hook-lens 'cg)
              limb-tip
              (rb-hook origin origin))))

(define (sim-hook-lens path)
  (match path
    [(cons rb-id hook-id)
     (lens-compose (dict-ref-lens hook-id)
                   rb-hooks-lens
                   (dict-ref-lens rb-id)
                   sim-rigid-bodies-lens)]))

(define (sim-hook-f-lens path)
  (lens-compose rb-hook-f-lens
                (sim-hook-lens path)))

(define (rest-length _) INITIAL-LIMB-LENGTH)
(define (damped-spring-force sim a-hook-path b-hook-path spring-state)
  (let* ([a-rb (dict-ref (sim-rigid-bodies sim) (car a-hook-path))]
         [a-hook (lens-view (sim-hook-lens a-hook-path) sim)]
         [a-hook-posn (rb/find-hook-position a-rb (cdr a-hook-path))]
         [b-rb (dict-ref (sim-rigid-bodies sim) (car b-hook-path))]
         [b-hook (lens-view (sim-hook-lens b-hook-path) sim)]
         [b-hook-posn (rb/find-hook-position b-rb (cdr b-hook-path))]
         [f (rb/damped-spring-force a-hook-posn b-hook-posn
                                    (rb/find-hook-velocity a-rb (cdr a-hook-path))
                                    (rest-length spring-state)
                                    K BETA)])
    (displayln f)
    f))

(define (gravity-force _1 _2 _3 _4)
  (posn 0 GRAVITY-FORCE))

;; climber-state
(define (cs/init)
  (let ([torso (initialize-torso)]
        [hand-l (initialize-limb-tip 'hand-l (dict-ref TIP-OFFSET-MAP 'sh-l) 1 0)]
        [hand-r (initialize-limb-tip 'hand-r (dict-ref TIP-OFFSET-MAP 'sh-r) 1 0)]
        [foot-l (initialize-limb-tip 'foot-l (dict-ref TIP-OFFSET-MAP 'hi-l) 1 0)]
        [foot-r (initialize-limb-tip 'foot-r (dict-ref TIP-OFFSET-MAP 'hi-r) 1 0)])
    (climber-state (sim `((torso  . ,torso)
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
                              (force '(torso . cg) #false
                                     gravity-force '())))
                   LIMB-MAP
                   0)))

(big-bang (cs/init)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT])
