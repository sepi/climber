#lang racket

(require 2htdp/image
         posn)

(provide line*
         place*)

(define (line* canvas from to color)
  (scene+line canvas
              (posn-x from) (posn-y from)
              (posn-x to) (posn-y to)
              color))

(define (place* canvas an-image a-posn)
  (place-image an-image
               (posn-x a-posn) (posn-y a-posn)
               canvas))
