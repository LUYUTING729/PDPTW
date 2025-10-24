# `main.cc` 项目解读

本文档对项目入口文件 `main.cc` 的核心函数、主要调用流程以及与其它模块的交互关系进行梳理，帮助快速理解程序运行机制。

## 文件结构与外部依赖概览

入口文件首先引入了标准库组件（时间、文件系统、文件流、格式化输出、JSON 解析等）以及项目内部头文件，包括：

- **问题/解模型**：`model/pdptw.h`、`model/solution.h` 用于描述带时间窗的取送货路径规划问题（PDPTW）及其解；
- **求解器组件**：`domain/solver/memetic/*` 提供遗传/大邻域搜索求解框架，`domain/solver/exact/TwoIndexFormulation.h`（可选）封装 Gurobi 精确求解器；
- **IO 工具**：`io/pdptw_reader.h`、`io/sol_reader.h` 用于读取实例与初始解；
- **转换与字符串工具**：`util/convert_utils.h`、`util/string_utils.h` 将领域模型与 VRP 表达互转并进行字符串处理。

除主函数外，文件内还定义了两个辅助函数：`sisr` 与 `writeFeasibleArcs`。下面依次介绍。

## 函数一览

### `void sisr(VrpProblem& vrpProblem, int vehicleNum)`

`sisr` 实现了一个基于**模拟退火**思想的“大邻域搜索”流程，用于改进给定的 VRP 问题初始解。其主要步骤如下：

1. **初始化解与参数**：
   - 通过 `buildInitialSolution` 构造给定车辆数的初始解 `vrpSolution`；
   - 设置退火温度、迭代次数以及冷却率，并复制当前解/候选解对象。
2. **迭代搜索**：在每次迭代中：
   - 将 `currentSol` 拷贝为 `newSol` 并执行 `stringRemoval`、`greedyInsertWithBlinks` 与 `insertEmptyRoute`，即先移除部分任务再以贪心方式重新插入，尝试生成邻域解；
   - 计算当前解与新解的目标值（同时考虑距离、时间窗违例、容量违例等惩罚项）；
   - 若新解更优，则接受并更新最优解；否则以退火概率 `exp(-Δ/temperature)` 决定是否接受以跳出局部最优；
   - 每 1000 代打印一次迭代编号、距离、约束违例及耗时信息。
3. **温度控制**：每轮迭代将当前温度乘以冷却率，逐步降低接受劣解的概率。

该函数未在当前 `main` 函数中调用，但保留用于实验或单独调度模拟退火流程。

### `void writeFeasibleArcs(const VrpSolution& vrpSolution)`

该函数遍历 `vrpSolution` 中每个节点的候选后继集合 `candiNextNodes`，将可行弧（节点对）写入 `<实例名>_feasible_arcs.txt`。它主要用于调试或分析可行图结构，目前同样未在主流程中使用。

### `int main(int argc, char* argv[])`

主函数负责从配置文件驱动整个求解过程，核心步骤如下：

1. **读取配置与实例**：
   - 从命令行获取 JSON 配置文件路径，并解析出实例名称 `instanceName`、额外车辆数 `additionalVehicleNum`、时间限制 `timeLimit`、结果目录 `resultFolder` 以及初始解列表 `initialSolFiles`；
   - 基于 `AMDAHL_DIR/data/instance/<instanceName>.txt` 构造实例文件路径，并调用 `ReadPDPTW` 加载 PDPTW 问题；
   - 将实例名转为小写，为后续匹配最佳已知解（BKS）文件做准备。

2. **定位并读取 BKS**：
   - 遍历 `AMDAHL_DIR/data/bks` 目录，依据文件名前缀匹配实例，处理了诸如 `lr2_10_5` 的特殊命名；
   - 找到对应的 `.sol` 文件后，使用 `ReadSol` 读取并通过 `ConvertVrpProblem`、`ConvertVrpSolution` 转换成内部的 `VrpProblem` 与 `VrpSolution`；
   - 输出 BKS 的距离、时间窗违例、容量违例等信息。

3. **处理初始解**：
   - 若配置提供多个初始解文件，则逐一读取并转换为 `VrpSolution`，存入 `initialSols` 供后续求解策略使用。

4. **选择求解器**：
   - **Gurobi 精确求解（可选）**：若编译时启用了 `USE_GUROBI` 且配置项 `solverType` 为 `"two_index"`，则构建相关配置，包括是否释放节点、可释放节点数量、释放路径索引、价格子问题参数等，随后调用 `twoIndexFormulationSolve` 启动基于二指标模型的精确求解并将结果输出至 `resultFolder`；
   - **Memetic 元启发式（默认）**：否则构造 `VrpConfig`（设定车辆成本、时间窗/容量惩罚系数），然后调用 `memetic`，传入：
     - 允许的车辆数（BKS 车辆数 + 额外车辆数）；
     - 时间限制 `timeLimit`；
     - BKS 作为参考解；
     - 初始解集合 `initialSols`；
     - 问题数据 `vrpProblem`；
     - 空的 `memeticSol` 以接收最终求解结果；
     - 结果目录 `resultFolder`。

5. **程序结束**：
   - 精确求解分支在完成后直接 `return 0`；
   - 默认分支在 `memetic` 结束后退出 `main`（返回 0 隐式完成）。

## 调用关系总结

整体流程可概括为：

1. `main` 解析配置 → 加载 PDPTW 实例 → 搜索并读取 BKS → 组装 VRP 问题/解对象；
2. 根据配置选择求解策略：
   - 精确求解：`twoIndexFormulationSolve`
   - 元启发式：`memetic`
3. 辅助函数 `sisr`、`writeFeasibleArcs` 可在需要时单独调用，用于局部搜索或输出可行弧调试信息。

通过以上步骤，`main.cc` 将外部数据、配置与求解组件串联起来，完成对 PDPTW 实例的求解流程。
