#ifndef AMDAHL_SRC_IO_VRP_SOL_WRITER_H_
#define AMDAHL_SRC_IO_VRP_SOL_WRITER_H_

#include "domain/model/VrpSolution.h"

bool WriteSol(const VrpSolution& solution, const std::string& sol_file_path);

#endif