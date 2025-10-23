This file explains how to runs the set-partitioning code with lambda-binary search.

The current code runs for 1 iteration, which is bound to be changed in later updates.
---------------------------------------------------------
**NOTICE**
1. generated columns should be stored in src/data/genCols folder
2. config.json should be modified to match the targeting instance
3. the main file is located at src/domain/solver/exact/main2.cc
4. the location of Gurobi in CMakeLists.txt should be changed
5. **important** modify codes in main2.cc to configure the target
    i. **instanceName**:: the instance name, which is defined in the same way as Longfei's code
    ii. **col_file**:: the file contains columns of the corresponding instance.
---------------------------------------------------------
**How to run**
1. compile with standard procedure in ../build folder
2. go to build folder and use **./amdahl** to run
3. after few iterations (you can change the starting lambda and EPS for stopping criteria in SetPartitioning.h file)
4. In **main2.cc**, you can change the MAXITERATION to be the max time you want to run the binary search. Besides, you can set the initial lambda for each run by modifying the code.