#include <filesystem>
#include <iostream>
#include <string>

#include "config.h"
#include "domain/solver/memetic/insert/Insert.h"
#include "domain/solver/memetic/removal/Removal.h"
#include "domain/utils/Utils.h"
#include "io/pdptw_reader.h"
#include "io/sol_reader.h"
#include "model/pdptw.h"
#include "model/solution.h"
#include "util/convert_utils.h"
#include "util/string_utils.h"
#include "util/distance_utils.h"
#include "domain/solver/exact/SetPartitioning.h"


int main(int argc, char* argv[]) {
    std::string configFileName = "/Users/xiaodongluo/amdahl/src/config4.json";
    std::string instanceName = "LR1_10_6_new";
    instanceName = "LR1_10_6_new";
    //int maxRoute = 46;
    // std::string col_file = "/home/lxyang/git/pdpwd/exact/amdahl/src/build/LR1_10_5-cache-routes-47223.txt";
    // std::string col_file = "/home/lxyang/git/pdpwd/exact/amdahl/src/data/genCols/cache_lr1_10_5/LR1_10_5-cache-routes-47223.txt";
    // std::string col_file = "/home/lxyang/git/pdpwd/exact/amdahl/src/data/genCols/cache_lr2_10_4/lr2_10_4-cache-routes-99368.txt";
    //std::string col_file = "/home/lxyang/git/pdpwd/exact/amdahl/src/data/genCols/cache_lr1_10_6/LR1_10_6-cache-routes-38365.txt";
    //std::string col_file = "E:\\Xiaodong\\pdVRPTW\\pdptw1000\\pdp_1000_223\\cache_lr1_10_5\\LR1_10_5-cache-routes-47223.txt";
    //std::string col_file = "/Users/xiaodongluo/amdahl/src/data/pdp_1000_223/cache_lr1_10_6/LR1_10_6-cache-routes-xd.txt";
    std::string col_file = "/Users/xiaodongluo/amdahl/src/data/pdp_1000_223/cache_lr1_10_6/LR1_10_6-cache-routes-new.txt2";
    //std::string col_file = "/Users/xiaodongluo/amdahl/src/data/pdp_1000_223/cache_lr1_10_5/LR1_10_5-cache-routes-xd.txt";
    std::filesystem::path instanceFilePath = AMDAHL_DIR;
    instanceFilePath /= "data";
    instanceFilePath /= "instance";
    instanceFilePath /= instanceName + ".txt";
    PDPTW problem;
    if (argc < 2) {
        printf("No configuration file name passed in, using default %s.\n", configFileName.c_str());
        //exit(0);
    } else
        configFileName = argv[1];

    ReadPDPTW(instanceFilePath.string(), problem);
    VrpProblem vrpProblem;
    ConvertVrpProblem(problem, vrpProblem);
    SetPartitioning sp(&vrpProblem.vrpData, &vrpProblem, configFileName, col_file, -1);
    
    //sp.setRoute(maxRoute);
    sp.solveSP();

    int MAXITERATION = 200;

    for (int i=0;i<MAXITERATION;++i){
        sp.setInitLambda(1.0f);
        sp.binSearch(i);
        sp.solveSP();
    }

}