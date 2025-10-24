# `CallBack::callback()` 执行逻辑梳理

本文聚焦 `domain/solver/exact/TwoIndexFormulation.cpp` 中自定义 Gurobi 回调 `CallBack` 的行为，说明它在求解主问题时如何实时还原整数解，并给出所有外部调用关系，便于调试与复用。

## 触发时机与注册位置

- `CallBack` 继承自 `GRBCallback`，由 `twoIndexFormulationSolve()` 在构建模型后、执行 `model.optimize()` 之前注册。仅当 `pricingSubProblem` 为 `false`（即求解原始二指标模型而非定价子问题）时才会安装该回调。【F:domain/solver/exact/TwoIndexFormulation.cpp†L15-L170】【F:domain/solver/exact/TwoIndexFormulation.cpp†L700-L740】
- 回调对象以栈变量形式构造，并通过 `model.setCallback(&cb)` 交给 Gurobi。随后 Gurobi 在分支定界过程中每当发现新的整数可行解时，会在 `where == GRB_CB_MIPSOL` 事件下调用 `CallBack::callback()`。【F:domain/solver/exact/TwoIndexFormulation.cpp†L32-L170】

## `callback()` 主流程

回调专门处理 `GRB_CB_MIPSOL` 事件，执行步骤如下：

1. **提取弧变量解值**：调用 `getSolution()` 读取弧变量矩阵 `arcVariables` 的解，首先从仓库出发的第一行找出所有起始节点，再遍历其余行获取每个节点的后继节点编号，填充 `nodeNextNodeIds` 数组。过程中加入空指针与越界检查，避免 Gurobi 返回空指针导致崩溃。【F:domain/solver/exact/TwoIndexFormulation.cpp†L32-L110】
2. **重建路径链表**：将 `newSol.totalValueValid` 置为 `false` 以迫使后续重新计算目标，然后依次处理每个起始节点：
   - `clearData()` 清空对应 `VrpRoute`；
   - 通过 `newOriginalNodeIndexMap` 找回原始节点索引，并把 `routeNodes` 中的节点指针串成链表，维护 `preNode`/`nexNode` 关系；
   - 统计路径长度并设置 `startNode`、`endNode`、`nodeNum`；
   - 调用 `newSol.updateRouteData()` 根据新链表重新计算时间窗与容量信息。【F:domain/solver/exact/TwoIndexFormulation.cpp†L110-L152】
3. **输出解文件**：利用 `std::filesystem` 和 `std::stringstream` 构建 `resultFolder/实例名.车辆数_距离_时间窗违约_容量违约.txt` 文件名，随后调用 `WriteSol(newSol, solFilePath)` 写出当前解，供列生成或调试使用。【F:domain/solver/exact/TwoIndexFormulation.cpp†L152-L170】【F:io/vrp_sol_writer.cc†L13-L74】
4. **异常处理**：任何 Gurobi 异常会打印错误码和消息，其它异常会输出通用告警，避免回调中断整个求解流程。【F:domain/solver/exact/TwoIndexFormulation.cpp†L166-L170】

## 外部依赖与调用关系

- **模型入口**：
  - `main.cc` 在解析配置后直接调用 `twoIndexFormulationSolve()`，因此任何从命令行触发的精确求解都会进入该回调逻辑。【F:main.cc†L213-L217】
  - `SetPartitioning::solveTwoInd()` 作为 Branch&Price 定价环节，也可能调用 `twoIndexFormulationSolveZ()`；不过它内部会将 `pricingSubProblem` 设为 `true`，从而绕过回调，仅输出 LP/解文件用于调试。【F:domain/solver/exact/SetPartitioning.cpp†L144-L171】【F:domain/solver/exact/TwoIndChecker.cpp†L14-L200】
- **与 `VrpSolution` 的交互**：
  - `callback()` 依赖 `VrpSolution` 暴露的路由数组 `routes`、节点池 `routeNodes` 以及 `updateRouteData()`、`getTotalDistance()` 等方法来维护链表并生成统计值。【F:domain/model/VrpSolution.h†L9-L152】【F:domain/solver/exact/TwoIndexFormulation.cpp†L110-L170】
- **文件输出**：
  - `WriteSol()` 位于 `io/vrp_sol_writer.cc`，负责生成标准化的 VRP 解文件。回调将当前构造的 `VrpSolution` 传入该函数完成持久化，必要时会自动创建输出目录。【F:domain/solver/exact/TwoIndexFormulation.cpp†L152-L170】【F:io/vrp_sol_writer.cc†L13-L74】
- **Gurobi API**：
  - `getSolution()` 来自基类 `GRBCallback`，用于批量提取变量值；`model.setCallback()` 与 `GRB_CB_MIPSOL` 则是 Gurobi MIP 回调机制的一部分，是回调触发的根源。【F:domain/solver/exact/TwoIndexFormulation.cpp†L32-L170】【F:domain/solver/exact/TwoIndexFormulation.cpp†L700-L740】

通过以上梳理，可以把握 `CallBack::callback()` 如何在每次找到可行整数解时即时恢复线路结构、校验解的内部一致性，并输出可供复现的解文件，从而支撑 Branch&Bound 环节的调试与结果追踪。
