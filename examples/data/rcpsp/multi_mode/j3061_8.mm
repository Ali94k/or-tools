************************************************************************
file with basedata            : mf61_.bas
initial value random generator: 1890488623
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  32
horizon                       :  230
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     30      0       37        2       37
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          1           6
   3        3          3           5   9  16
   4        3          3           7   8  25
   5        3          3          10  13  14
   6        3          3           8  10  14
   7        3          2           9  27
   8        3          2          12  16
   9        3          1          28
  10        3          1          11
  11        3          3          15  17  23
  12        3          1          26
  13        3          3          18  20  23
  14        3          2          25  31
  15        3          3          18  19  20
  16        3          2          17  28
  17        3          3          18  19  22
  18        3          2          26  30
  19        3          2          24  26
  20        3          1          21
  21        3          2          24  28
  22        3          1          27
  23        3          2          24  30
  24        3          1          27
  25        3          1          30
  26        3          1          29
  27        3          2          29  31
  28        3          1          29
  29        3          1          32
  30        3          1          32
  31        3          1          32
  32        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     8       9    3    3    7
         2     8       6    2    4    7
         3     9       4    1    2    4
  3      1     7       5    7    5    6
         2    10       4    6    4    1
         3    10       3    7    4    1
  4      1     2       6    4    9    3
         2     5       5    2    4    3
         3    10       3    1    1    2
  5      1     4       4    9    5    4
         2     6       4    4    5    4
         3    10       3    3    4    3
  6      1     2       8    9    3    7
         2     7       5    8    2    7
         3     8       2    7    2    6
  7      1     4       7    6    8    8
         2     4       7    6    9    5
         3     6       7    5    5    2
  8      1     2       6    6    9    7
         2     5       4    3    6    6
         3     6       4    1    5    4
  9      1     1       7    5   10    5
         2     1       8    5    6    5
         3     2       7    2    2    4
 10      1     6       6    8    8    8
         2     7       4    4    6    8
         3     7       5    6    5    8
 11      1     2       7   10    9    3
         2     3       4    7    7    2
         3     8       2    7    6    1
 12      1     3       9    5    3    6
         2     4       8    4    3    5
         3     6       8    2    2    5
 13      1     6       6    9    3    3
         2     8       6    5    2    3
         3    10       5    2    2    3
 14      1     1       9    3    3    9
         2     1      10    3    2    9
         3     5       9    3    2    9
 15      1     2       5    8    9    8
         2     6       3    8    8    5
         3     9       2    7    8    4
 16      1     7       6   10   10    3
         2     8       5    8   10    3
         3    10       4    3   10    3
 17      1     6       5    9    8    4
         2     7       5    8    7    4
         3     9       4    8    6    4
 18      1     1       9    9    7    7
         2     5       8    7    4    3
         3     6       7    4    2    1
 19      1     1       7    9    9    8
         2     2       4    9    9    8
         3     5       1    8    8    8
 20      1     1       2    6    7    5
         2     1       2    6    9    4
         3     3       1    5    4    3
 21      1     3      10    6    9    7
         2     4       9    4    8    6
         3     6       8    1    6    3
 22      1     1       6    7    6    7
         2     5       5    7    6    6
         3     7       4    7    6    6
 23      1     6       3    5   10    3
         2     7       3    4    7    2
         3    10       1    4    4    1
 24      1     1       5    6    5    5
         2     5       5    6    4    4
         3     7       5    6    3    3
 25      1     7       4    9    9    7
         2     9       4    9    6    6
         3    10       4    6    5    1
 26      1     2       3    8    9    9
         2     5       3    4    8    8
         3     9       2    2    8    8
 27      1     3       7    7    6    6
         2     7       6    7    6    4
         3     9       4    7    6    2
 28      1     3       4    4    8    4
         2     6       3    3    6    4
         3     7       1    3    4    2
 29      1     4       5    8    9    7
         2     6       4    7    9    6
         3     8       4    5    9    6
 30      1     3       4   10    7    9
         2     4       3   10    5    9
         3     8       1   10    4    8
 31      1     7       6    4    8   10
         2     9       6    4    7    5
         3    10       6    3    7    4
 32      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   15   18  218  185
************************************************************************