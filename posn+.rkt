#lang racket

(require posn
         rnrs/base-6 ; mod
         rackunit)

(provide posn-dotp
         posn-len
         posn-normalize
         posn-abs
         posn-angle
         posn-angle-diff
         posn-rot)

(define (posn-dotp a b)
  (+ (* (posn-x a) (posn-x b))
     (* (posn-y a) (posn-y b))))

(define (posn-len a-posn)
  (sqrt (+ (* (posn-x a-posn) (posn-x a-posn))
           (* (posn-y a-posn) (posn-y a-posn)))))

(define (posn-normalize a-posn)
  (posn-scale (/ 1 (posn-len a-posn)) a-posn))

(define (posn-abs a-posn)
  (posn (abs (posn-x a-posn)) (abs (posn-y a-posn))))

(define (posn-angle a-posn)
  (if (equal? a-posn (posn 0 0))
      0
      (radians->degrees (atan (posn-y a-posn) (posn-x a-posn)))))

(define (posn-angle-diff a-posn b-posn)
  (- (posn-angle b-posn)
     (posn-angle a-posn)))

(define (posn-rot anchor rotation point)
  (posn-rotate-ccw anchor (mod rotation 360) point))
(check-equal? (posn-rot (posn 0 0) 0 (posn 1 0))
              (posn 1 0))

(define (posn-equal a b)
  (and (= (posn-x a) (posn-x b))
       (= (posn-y a) (posn-y b))))
(check-equal? (posn-equal (posn 0 0) (posn 0 0)) #t)
(check-equal? (posn-equal (posn 1 0) (posn 1 0)) #t)

