#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         rackunit
         threading
         lens
         "dualshock.rkt"
         "rigid-body.rkt"
         "posn+.rkt"
         "graphics.rkt")

;;
;; Structs
;;

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

;;
;; Lenses
;;

(define (climber-state-rigid-body-lens rb-key)
  (lens-compose (dict-ref-lens rb-key)
                sim-rigid-bodies-lens
                climber-state-sim-lens))

(define (sim-hook-lens path)
  (match path
    [(cons rb-id hook-id)
     (lens-compose (dict-ref-lens hook-id)
                   rb-hooks-lens
                   (dict-ref-lens rb-id)
                   sim-rigid-bodies-lens)]))

(define (sim-hook-f-lens path)
  (lens-compose hook-f-lens
                (sim-hook-lens path)))

;;
;; Constants
;;

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
(define GRAVITY-FORCE 5000)

;;
;; Drawing
;;

(define (knee-direction key)
  (dict-ref '((sh-l . -1) (sh-r . 1) (hi-l . 1) (hi-r . -1)) key))

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

(define (draw-rb rb canvas)
  (for/fold ([canvas canvas])
            ([key (in-dict-keys (rb-hooks rb))])
    (place* canvas (circle 3 'outline "black") (rb/find-hook-position rb key))))

(define (draw-force f sim canvas)
  (let ([a-path (force-a-hook-path f)]
        [b-path (force-b-hook-path f)]
        [f-state (force-state f)])
    (if b-path
        (let ([from (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car a-path))
                                           (cdr a-path))]
              [to   (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car b-path))
                                           (cdr b-path))])
          (draw-limb from to f-state canvas)
          #;(line* canvas from to "blue"))
        canvas)))

(define (draw-limb from-posn to-posn force-state canvas)
  (let* ([limb-length (dict-ref force-state 'limb-length)]
         [limb-key (dict-ref force-state 'key)]
         [knee-dir (knee-direction limb-key)]
         [knee (knee-posn from-posn to-posn limb-length limb-length knee-dir)])
    (~> canvas
        (line* _ from-posn knee "black")
        (line* _ knee to-posn "black"))))

(define (cs/draw cs)
  (let* ([sim (climber-state-sim cs)])
    (~> SCENE
        (foldl draw-rb _ (dict-values (sim-rigid-bodies sim)))
        (foldl (lambda (f canvas) (draw-force f sim canvas))
               _ (sim-forces sim)))))
  
#;(define (cs/set-limb-tip-posn cs key limb-tip-offset)
  (let ([limb-tip-orig (posn-add TORSO-POSITION (dict-ref TIP-OFFSET-MAP key))])
    (lens-set (lens-compose limb-tip-lens
                            (dict-ref-lens key)
                            climber-state-limbs-lens)
              cs
              (posn-add limb-tip-orig
                        limb-tip-offset))))

;;
;; Simulation
;;

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
         (sim-forces sim)))

(define (sim-reset-forces sim)
  (foldl (λ (force sim*)
           (lens-set (sim-hook-f-lens (force-a-hook-path force))
                     sim*
                     origin))
         sim
         (sim-forces sim)))

(define (cs/tick cs)
  (let-values ([(stick-l stick-r) (pad-stick-posns)])
    (~> cs
        (lens-transform climber-state-sim-lens _ sim-reset-forces)
        (lens-transform climber-state-sim-lens _ sim-update-forces)
        ; Integrate non-static rigid bodies
        (foldl (λ (key cs)
                 (lens-transform (climber-state-rigid-body-lens key)
                                 cs
                                 (λ (rb)
                                   (if (rb-static rb)
                                       rb
                                       (rb/integrate rb DT)))))
               _
               (dict-keys (sim-rigid-bodies (climber-state-sim cs))))
        (lens-transform climber-state-time-lens _
                        (λ (t) (+ t DT)))
        #;(cs/set-limb-tip-posn _ 'sh-l stick-l)
        #;(cs/set-limb-tip-posn _ 'sh-r stick-r))))

(define (initialize-torso)
  (let ([torso (rb 'torso
                   #false
                   TORSO-POSITION origin
                   0 0
                   TORSO-POSITION-MASS TORSO-ROTATION-MASS
                   '())])
    (define (make-hook-from-offset x y)
      (hook (posn x y) origin))
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
              (hook origin origin))))

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
    f))

(define (gravity-force _1 _2 _3 _4)
  (posn 0 GRAVITY-FORCE))

;;
;; Initialize state
;;

(define TIP-OFFSET-MAP
  `((sh-l . ,(posn -40 -30))
    (sh-r . ,(posn 40 -30))
    (hi-l . ,(posn -40 30))
    (hi-r . ,(posn 40 30))))

(define LIMB-MAP
  (map (lambda (key)
         (cons key (limb INITIAL-LIMB-LENGTH
                         (posn-add (dict-ref TIP-OFFSET-MAP key)
                                   TORSO-POSITION))))
       '(sh-l sh-r hi-l hi-r)))

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
                                     damped-spring-force '((key . sh-l)
                                                           (limb-length . 25)))
                              (force '(torso . sh-r) '(hand-r . cg)
                                     damped-spring-force '((key . sh-r)
                                                           (limb-length . 25)))
                              (force '(torso . hi-l) '(foot-l . cg)
                                     damped-spring-force '((key . hi-l)
                                                           (limb-length . 25)))
                              (force '(torso . hi-r) '(foot-r . cg)
                                     damped-spring-force '((key . hi-r)
                                                           (limb-length . 25)))
                              (force '(torso . cg) #false
                                     gravity-force '())))
                   LIMB-MAP
                   0)))

(big-bang (cs/init)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT])
