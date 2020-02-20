#lang racket

(require csfml
         posn)

(provide pad-update
         pad-find-first-connected
         pad-stick
         pad-button-press
         pad-button-pressed
         pad-stick-posn)

(define (pad-find-first-connected)
  (sfJoystick_update)
  (for/first ([i 8]
              #:when (sfJoystick_isConnected i))
    i))

(define PAD-IDX
  (pad-find-first-connected))

(define BUTTON-KEYS '(w s e n l1 r1 l2 r2 share options ls rs ps pad))
(define BUTTON-PREVIOUS-STATE
  (map (λ (k) (cons k #f))
       BUTTON-KEYS))

;; Needs to be run to query new data from driver
(define (pad-update)
  (set! BUTTON-PREVIOUS-STATE
        (map (λ (k) (cons k (pad-button-pressed k)))
             BUTTON-KEYS))
  (sfJoystick_update))

;; True while button is pressed
(define (pad-button-pressed button)
  (sfJoystick_isButtonPressed PAD-IDX (button-map button)))

;; True if button is newly pressed
(define (pad-button-press button)
  (and (not (dict-ref BUTTON-PREVIOUS-STATE button))
       (pad-button-pressed button)))

(define (pad-stick side axis)
  (if PAD-IDX
      (cond [(and (equal? side 'left) (equal? axis 'x))
             (sfJoystick_getAxisPosition PAD-IDX 'sfJoystickX)]
            [(and (equal? side 'left) (equal? axis 'y))
             (sfJoystick_getAxisPosition PAD-IDX 'sfJoystickY)]
            [(and (equal? side 'right) (equal? axis 'x))
             (sfJoystick_getAxisPosition PAD-IDX 'sfJoystickZ)]
            [(and (equal? side 'right) (equal? axis 'y))
             (sfJoystick_getAxisPosition PAD-IDX 'sfJoystickR)])
      0))

(define (pad-stick-posn side)
  (posn (pad-stick side 'x) (pad-stick side 'y)))

(define (button-map button)
  (case button
    ['w 0] ['s 1] ['e 2] ['n 3]
    ['l1 4] ['r1 5] ['l2 6] ['r2 7]
    ['share 8] ['options 9]
    ['ls 10] ['rs 11]
    ['ps 12] ['pad 13]))
