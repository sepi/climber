#lang racket

(require csfml)

(provide pad-find-first-connected
         pad-stick)

(define (pad-find-first-connected)
  (sfJoystick_update)
  (for/first ([i 8]
              #:when (sfJoystick_isConnected i))
    i))

(define PAD-IDX
  (pad-find-first-connected))

(define (pad-stick side axis)
  (sfJoystick_update)
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
