# A* path find algorithm

to run the code:

    python3 A_Star_Map

result:

    start:(x:1, y:1), end:(x:36, y:12)
    -------------------------
    Map:
    0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 
    0 S 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 
    0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 1 0 0 
    0 0 0 0 0 0 1 0 1 1 1 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 1 0 1 1 1 1 1 1 0 0 0 1 0 0 
    0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 1 1 1 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 E 0 0 0 
    -------------------------
    Map After Expansion:
    0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 0 0 0 
    0 S 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1 1 0 
    0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 0 
    1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 0 
    1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 E 0 0 0 
    -------------------------
    Route Node:
    0:(x:2, y:2) 1:(x:3, y:3) 2:(x:4, y:4) 3:(x:5, y:5) 4:(x:6, y:6) 5:(x:7, y:7) 6:(x:8, y:8) 7:(x:9, y:9) 8:(x:10, y:10) 9:(x:11, y:10) 10:(x:12, y:10) 11:(x:13, y:10) 12:(x:14, y:10) 13:(x:15, y:10) 14:(x:16, y:10) 15:(x:17, y:10) 16:(x:18, y:10) 17:(x:19, y:10) 18:(x:20, y:10) 19:(x:21, y:10) 20:(x:22, y:10) 21:(x:23, y:10) 22:(x:24, y:10) 23:(x:25, y:10) 24:(x:26, y:11) 25:(x:27, y:11) 26:(x:28, y:12) 27:(x:29, y:12) 28:(x:30, y:12) 29:(x:31, y:12) 30:(x:32, y:12) 31:(x:33, y:12) 32:(x:34, y:12) 33:(x:35, y:12) 34:(x:36, y:12) 
    ----------------------
    Route:
    0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 0 0 0 
    0 S 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 1 1 1 1 1 0 
    0 0 # 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 # 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 0 # 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 1 1 0 
    0 0 0 0 0 # 0 0 1 1 1 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 0 
    1 1 1 1 1 1 # 1 1 1 1 0 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 0 1 1 1 1 1 1 1 0 
    1 1 1 1 1 1 1 # 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    1 1 1 1 1 1 1 1 # 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 # 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 # # # # # # # # # # # # # # # # 1 0 0 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 # # 1 0 0 0 0 0 0 0 0 0 0 0 
    0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 0 1 1 1 1 1 1 1 1 1 1 1 1 # # # # # # # # E 0 0 0

the 0 means Node can be pass, 1 means barrier, S is start point and E represents end point.