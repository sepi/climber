#lang racket

(require 2htdp/image
         2htdp/universe
         posn
         threading
         lens
         "pad.rkt"
         "rigid-body.rkt"
         "posn+.rkt"
         "graphics.rkt")

;;
;; Structs
;;

(struct/lens climber-state (sim time control)
             #:transparent)

;; Controls wether hand or foot is controlled by left or right stick
;; Can be 'hand or 'foot
(struct/lens control (left right)
             #:transparent)

(struct/lens sim (rigid-bodies forces)
             #:transparent)

; Represents a force generating connection between two rigid-body hooks.
; The calculated force will be applied to both hooks.
; hooks are referenced using paths i.e. pairs (rigid-body-id . hook-id)
; function is a function: sim hook-path hook-path force-props -> force
(struct/lens force (id a-hook-path b-hook-path function props)
             #:transparent)

;;
;; Lenses
;;

(define (climber-state-rigid-body-lens rb-key)
  (lens-compose (dict-ref-lens rb-key)
                sim-rigid-bodies-lens
                climber-state-sim-lens))

(define (cs-rigid-body-prop-lens rb-key prop-key)
  (lens-compose (dict-ref-lens prop-key)
                rb-props-lens
                (climber-state-rigid-body-lens rb-key)))

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
  (dict-ref '((sh-hand-l . -1) (sh-hand-r . 1) (hi-foot-l . 1) (hi-foot-r . -1)) key))

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
  (let* ([a-path (force-a-hook-path f)]
         [b-path (force-b-hook-path f)]
         [f-props (force-props f)]
         [f-id (force-id f)])
    (if (member f-id '(sh-hand-l sh-hand-r hi-foot-l hi-foot-r))
        (let ([from (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car a-path))
                                           (cdr a-path))]
              [to   (rb/find-hook-position (dict-ref (sim-rigid-bodies sim)
                                                     (car b-path))
                                           (cdr b-path))])
          (draw-limb f-id from to (dict-ref f-props 'limb-length) canvas)
          #;(line* canvas from to "blue"))
        canvas)))

(define (draw-limb f-id from-posn to-posn limb-length canvas)
  (let* ([knee-dir (knee-direction f-id)]
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

;;
;; Simulation
;;

(define (sim-update-force force sim)
  (let* ([a-hook-path (force-a-hook-path force)]
         [b-hook-path (force-b-hook-path force)]
         [force-fn (force-function force)]
         [fs (force-props force)]
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
  (pad-update)
  (~> cs
      (handle-controls _)
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
                      (λ (t) (+ t DT)))))

;; Initialize hand or foot rigid-bodies
;; containing one cg hook at the limb-tips origin
(define (rest-length _) INITIAL-LIMB-LENGTH)
(define (damped-spring-force sim a-hook-path b-hook-path spring-props)
  (let* ([a-rb (dict-ref (sim-rigid-bodies sim) (car a-hook-path))]
         [a-hook (lens-view (sim-hook-lens a-hook-path) sim)]
         [a-hook-posn (rb/find-hook-position a-rb (cdr a-hook-path))]
         [b-rb (dict-ref (sim-rigid-bodies sim) (car b-hook-path))]
         [b-hook (lens-view (sim-hook-lens b-hook-path) sim)]
         [b-hook-posn (rb/find-hook-position b-rb (cdr b-hook-path))]
         [f (rb/damped-spring-force a-hook-posn b-hook-posn
                                    (rb/find-hook-velocity a-rb (cdr a-hook-path))
                                    (rest-length spring-props)
                                    K BETA)])
    f))

(define (gravity-force _1 _2 _3 _4)
  (posn 0 GRAVITY-FORCE))

;;
;; Controls
;;

(define (cs/set-limb-tip-posn cs side limb-tip-offset)
  (let* ([rb-key (select-controlled-rb side (cs/controlled-tip cs side) )]
         [ref-posn (lens-view (cs-rigid-body-prop-lens rb-key 'ref-posn) cs)]
         [limb-tip-posn (posn-add ref-posn
                                  limb-tip-offset)]
         [tip-posn-lens (lens-compose rb-position-lens
                                      (climber-state-rigid-body-lens rb-key))])
    (lens-set tip-posn-lens
              cs
              limb-tip-posn)))

(define (cs/controlled-tip cs side)
  (match side
    ['left (control-left (climber-state-control cs))]
    ['right (control-right (climber-state-control cs))]))

(define (select-controlled-rb side tip)
  (match (cons side tip)
    ['(left . hand) 'hand-l]
    ['(right . hand) 'hand-r]
    ['(left . foot) 'foot-l]
    ['(right . foot) 'foot-r]))

(define (other-tip tip)
  (case tip
    ['hand 'foot]
    ['foot 'hand]))

(define (switch-tip cs)
  (~> cs
      (lens-transform (lens-compose control-left-lens
                                    climber-state-control-lens)
                      _
                      (λ (old-tip)
                        (if (pad-button-press 'l1)
                            (other-tip old-tip)
                            old-tip)))
      (lens-transform (lens-compose control-right-lens
                                    climber-state-control-lens)
                      _
                      (λ (old-tip)
                        (if (pad-button-press 'r1)
                            (other-tip old-tip)
                            old-tip)))))

(define (maybe-update-limb-tip-ref cs)
  ;; (lens-update (cs-rigid-body-prop-lens ))
  ;; (lens-view (lens-compose control-left-lens
  ;;                          climber-state-control-lens)
  ;;            cs)
  ;;(select-controlled-rb side (cs/controlled-tip cs side) )
  (if (pad-button-press 'l1)
      (lens-set (cs-rigid-body-prop-lens (select-controlled-rb 'left (cs/controlled-tip cs 'left))
                                         'ref-posn)
                cs
                (pad-stick-posn 'left))
      cs))

(define (handle-controls cs)
  (~> cs
      (cs/set-limb-tip-posn _ 'left (pad-stick-posn 'left))
      (cs/set-limb-tip-posn _ 'right (pad-stick-posn 'right))
      switch-tip
      maybe-update-limb-tip-ref))

;;
;; Initialize state
;;

(define TIP-OFFSET-MAP
  `((hand-l . ,(posn -40 -30))
    (hand-r . ,(posn 40 -30))
    (foot-l . ,(posn -40 30))
    (foot-r . ,(posn 40 30))))

(define (initialize-torso)
  (let ([torso (rb 'torso
                   #false
                   TORSO-POSITION origin
                   0 0
                   TORSO-POSITION-MASS TORSO-ROTATION-MASS
                   '()
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

(define (initialize-limb-tip key posn-mass rot-mass)
  (let* ([ref-posn (posn-add (dict-ref TIP-OFFSET-MAP key) TORSO-POSITION)]
         [limb-tip-rb (rb key #true
                          ref-posn origin
                          0 0
                          posn-mass rot-mass
                          '()
                          (list (cons 'ref-posn ref-posn)))])
    (lens-set (rb/hook-lens 'cg)
              limb-tip-rb
              (hook origin origin))))

(define (cs/init)
  (climber-state
   (sim `((torso  . ,(initialize-torso))
          (hand-l . ,(initialize-limb-tip 'hand-l 1 0))
          (hand-r . ,(initialize-limb-tip 'hand-r 1 0))
          (foot-l . ,(initialize-limb-tip 'foot-l 1 0))
          (foot-r . ,(initialize-limb-tip 'foot-r 1 0)))
        (list (force 'sh-hand-l '(torso . sh-l) '(hand-l . cg)
                     damped-spring-force '((limb-length . 25)))
              (force 'sh-hand-r '(torso . sh-r) '(hand-r . cg)
                     damped-spring-force '((limb-length . 25)))
              (force 'hi-foot-l '(torso . hi-l) '(foot-l . cg)
                     damped-spring-force '((limb-length . 25)))
              (force 'hi-foot-r '(torso . hi-r) '(foot-r . cg)
                     damped-spring-force '((limb-length . 25)))
              (force 'torso-g '(torso . cg) #false
                     gravity-force '())))
   0
   (control 'hand 'hand)))

(big-bang (cs/init)
          [to-draw cs/draw WIDTH HEIGHT]
          [on-tick cs/tick DT])
