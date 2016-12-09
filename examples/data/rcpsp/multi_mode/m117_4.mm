************************************************************************
file with basedata            : cm117_.bas
initial value random generator: 377737617
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  83
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       39       12       39
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        1          1          14
   3        1          3           5   6  10
   4        1          3           5   6  11
   5        1          1           7
   6        1          1           8
   7        1          3           8  12  14
   8        1          2           9  13
   9        1          3          15  16  17
  10        1          2          11  14
  11        1          2          13  17
  12        1          2          13  16
  13        1          1          15
  14        1          3          15  16  17
  15        1          1          18
  16        1          1          18
  17        1          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     5       0    8    7    0
  3      1     2       0    3    0    9
  4      1     7       8    0    0    7
  5      1     7       0    3    0    3
  6      1     8       0    2    0    5
  7      1     6       0    8    4    0
  8      1     6       0    2    0    1
  9      1     8       0    7    0    6
 10      1     3       0    2    0    1
 11      1     8       5    0    0    6
 12      1     6       3    0    0    6
 13      1     3       7    0    8    0
 14      1     2       0    8    6    0
 15      1     3       6    0    0    8
 16      1     4       0    4    0    3
 17      1     5       0    8    0    9
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
    8    9   25   64
************************************************************************