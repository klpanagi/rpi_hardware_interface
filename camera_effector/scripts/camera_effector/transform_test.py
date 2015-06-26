#!/usr/bin/python

# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Konstantinos Panayiotou"
__maintainer__ = "Konstantinos Panayiotou"
__email__ = "klpanagi@gmail.com"


import math
import time
import sys, os


def transform(angle):
  A = 32  
  B = 13
  r1 = 10
  r2 = 30
  r3 = 12

  print
  y2 = 1.0 * math.cos(angle) * r3
  print "y2: %s" % y2
  x2 = 1.0 * math.sin(angle) * r3
  print "x2: %s" % x2
  y1 = B - y2
  print "y1: %s" % y1
  x1 = A + x2
  print "x1: %s" % x1

  temp = 1.0 * x1 / y1
  fi_angle = math.atan(temp)
  print "fi_angle: %s" % fi_angle

  p = 1.0 * x1 / math.sin(fi_angle)
  print "p: %s" % p

  temp = 1.0 * ( 1.0 * r1 * r1 + 1.0 * p * p - 1.0 * r2 * r2) / (2.0 * r1 * p)
  print "temp: %s" % temp
  r1p_angle = math.acos(temp)
  print "r1p_angle: %s" % r1p_angle

  theta = fi_angle - r1p_angle
  #print
  #print "x1: %s ,  y1: %s ,  x2: %s ,  y2: %s , p: %s ,  fi_angle: %s ,  r1p_angle: %s" % (x1, y1, x2, y2, fi_angle, r1p_angle, p)
  return theta

def main():
  angle = 0
  while True:
    theta = transform(angle)
    print "Desired camera angle: %s ,   Serco command angle: %s" \
        % (angle,theta)
    angle += 0.1
    time.sleep(1)

if __name__ == "__main__":
  main()
