#ifndef AMDAHL_SRC_DOMAIN_SOLVER_EXACT_IMPROVED_TWO_INDEX_FORMULATION_H_
#define AMDAHL_SRC_DOMAIN_SOLVER_EXACT_IMPROVED_TWO_INDEX_FORMULATION_H_

#include "domain/solver/exact/TwoIndexFormulation.h"

/**
 * @brief 改进型双指标车辆路径问题求解器 - 基于改进的时间和载重约束建模
 * 
 * @details
 * 该函数为PDPTW问题的改进型精确求解器，相比原始版本具有以下改进：
 * - 使用基于弧的时间变量 y_ij 和载重变量 z_ij，而非节点时间和载重
 * - 时间和容量约束建模更加紧凑和高效
 * - 支持基于弧的时间窗约束和时间传播约束
 * - 改进的数值稳定性和求解效率
 * 
 * 使用Gurobi求解器构建和求解MIP模型。
 *
 * @param vrpConfig 车辆路径问题配置参数
 * @param vrpProblem 指向车辆路径问题实例的指针
 * @param initSolution 指向初始解的指针（可为nullptr）
 * @param freeNodes 是否释放某些节点的指派约束
 * @param freeNodesNum 要释放指派约束的节点数量
 * @param freeRouteIndices 要释放的路由索引列表
 * @param timeLimit Gurobi求解器的时间限制（秒）
 * @param pricingSubProblem 是否作为列生成的定价子问题求解
 * @param pricingSubProblemData 定价子问题所需的数据
 * @param pricingSubProblemSolution [输出] 定价子问题的求解结果
 * @param vrpSolution [输出] 求得的VRP最优或近优解
 * @param resultFolder 结果文件的输出文件夹路径
 *
 * @throw GRBException Gurobi库异常
 * @throw std::exception 标准异常
 *
 * @note 
 * - 初始解必须是可行解（无未分配节点、无容量和时间窗违反）
 * - 求解过程中会在resultFolder中生成中间解文件
 * - 求解日志输出到 "two_index_formulation_*.log" 文件
 * 
 * @see twoIndexFormulationSolve 原始版本
 * @see VrpSolution VrpProblem
 */
void improvedTwoIndexFormulationSolve(const VrpConfig& vrpConfig, 
                                      const VrpProblem* vrpProblem, 
                                      const VrpSolution* initSolution,
                                      bool freeNodes, 
                                      int freeNodesNum, 
                                      const std::vector<int>& freeRouteIndices,
                                      double timeLimit, 
                                      bool pricingSubProblem,
                                      const PricingSubProblemData& pricingSubProblemData,
                                      PricingSubProblemSolution& pricingSubProblemSolution, 
                                      VrpSolution& vrpSolution,
                                      const std::string& resultFolder);

#endif  // AMDAHL_SRC_DOMAIN_SOLVER_EXACT_IMPROVED_TWO_INDEX_FORMULATION_H_
