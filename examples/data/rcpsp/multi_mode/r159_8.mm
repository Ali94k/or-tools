************************************************************************
file with basedata            : cr159_.bas
initial value random generator: 977838243
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  138
RESOURCES
  - renewable                 :  1   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       17       10       17
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          3           9  11  13
   3        3          3          11  15  16
   4        3          2           5   8
   5        3          3           6   7  16
   6        3          2          12  13
   7        3          3           9  10  15
   8        3          3           9  13  15
   9        3          2          12  14
  10        3          2          11  12
  11        3          1          14
  12        3          1          17
  13        3          1          14
  14        3          1          17
  15        3          1          18
  16        3          1          18
  17        3          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0
  2      1     2       0    7    8
         2    10       3    7    4
         3    10       0    7    6
  3      1     1       0    3    6
         2     7       0    2    4
         3     9       0    2    2
  4      1     4       4    6    9
         2     6       4    6    7
         3     9       0    6    7
  5      1     1       2    8    9
         2     4       2    7    9
         3     8       0    7    9
  6      1     1       7    7    8
         2     3       3    7    6
         3     5       2    6    4
  7      1     2       6    6    9
         2     2       0    7    9
         3     9       0    1    6
  8      1     2       7    8    9
         2     7       7    7    7
         3    10       0    5    7
  9      1     6       5    7    5
         2     7       0    7    4
         3    10       0    1    4
 10      1     6       5    7    5
         2     9       4    2    2
         3     9       4    1    3
 11      1     1       0    8    9
         2     1       0    9    8
         3     5       0    7    8
 12      1     2       2    7   10
         2     2       2    9    9
         3     8       0    6    4
 13      1     5       1    9    9
         2     8       0    7    6
         3     8       0    9    5
 14      1     1       7    8    3
         2     4       0    7    2
         3    10       0    7    1
 15      1     4       6    8    9
         2     7       4    5    9
         3    10       3    3    8
 16      1     3       0    7    2
         2     7       0    6    2
         3     8       6    5    2
 17      1     2       0    9    7
         2     6       3    8    6
         3    10       0    8    5
 18      1     0       0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  N 1  N 2
   23  119  117
************************************************************************