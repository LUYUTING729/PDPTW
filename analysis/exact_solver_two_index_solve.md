# `twoIndexFormulationSolve` 详解

本文基于 `domain/solver/exact/TwoIndexFormulation.cpp` 的源码，对 `twoIndexFormulationSolve` 的参数、建模步骤、启发式热启动、回调与输出流程进行逐行梳理，便于理解二指标主问题的完整求解逻辑。

## 入口与参数语义
函数签名如下：

```cpp
void twoIndexFormulationSolve(
    const VrpConfig& vrpConfig,
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
```

* `vrpProblem` 携带节点列表、距离矩阵、车辆容量等静态数据；`vrpConfig` 在热启动时传入局部破坏算子。`initSolution` 若非空则被复制到 `vrpSolution`，作为求解的初始解框架，并断言其无未分配节点、无容量/时间窗违反。【F:domain/solver/exact/TwoIndexFormulation.cpp†L183-L188】
* `freeNodes/freeNodesNum/freeRouteIndices` 指示热启动时要释放的节点或整条路径；`timeLimit` 直接作为 Gurobi 的 `TimeLimit`；`pricingSubProblem` 形参随即被强制置为 `false`，因此本函数只构建主问题版本；定价专用逻辑由 `twoIndexFormulationSolveZ` 负责。【F:domain/solver/exact/TwoIndexFormulation.cpp†L219-L231】
* `pricingSubProblemData` 与 `pricingSubProblemSolution` 在主问题中不会使用；`resultFolder` 既用于写回调生成的 `.txt` 解，又作为 `.lp` 模型文件的输出位置。【F:domain/solver/exact/TwoIndexFormulation.cpp†L151-L165】【F:domain/solver/exact/TwoIndexFormulation.cpp†L792-L817】

## 节点重编号与索引映射
模型采用“二指标”结构：所有取货点排在 `[1, pickupCount]`，对应送货点排在 `[pickupCount+1, 2*pickupCount]`，`0` 是起点虚拟仓库，`2*pickupCount+1` 是终点仓库。因此首先建立 `originalNewNodeIndexMap` 与 `newOriginalNodeIndexMap` 双向映射，并统计取货节点数量。【F:domain/solver/exact/TwoIndexFormulation.cpp†L190-L218】

* 迭代原图节点时，仅当节点类型为取货才计数，并同步写入对应送货节点的映射。若 `corresNodeId` 越界会打印错误并跳过，避免非法访问。【F:domain/solver/exact/TwoIndexFormulation.cpp†L202-L217】
* 变量 `totalPickupNodeNumInGraph`、`totalNodeNumInGraph` 分别表示取货节点数与引入终点后的总节点数（=原节点数+1）。【F:domain/solver/exact/TwoIndexFormulation.cpp†L253-L255】

> ⚠️ 由于 `pricingSubProblem` 被立即清零，后续所有“定价模式”分支都不会触发；这些代码保留下来但在主问题求解时被跳过。【F:domain/solver/exact/TwoIndexFormulation.cpp†L219-L231】

## Gurobi 环境初始化
函数在栈上创建 `GRBEnv env(true)`，设置日志文件名、`MIPGap=0` 与时间上限，然后启动环境并构建 `GRBModel model`。【F:domain/solver/exact/TwoIndexFormulation.cpp†L272-L288】

日志文件名固定为 `two_index_formulation_original.log`（主问题），若未来重新开放定价模式，则会追加 `_pricing_sub_problem` 后缀。【F:domain/solver/exact/TwoIndexFormulation.cpp†L274-L281】

## 决策变量
### 弧变量 `x_{ij}`
对所有 `i,j∈[0, totalNodeNumInGraph-1]` 创建二元弧变量，利用候选邻接表和若干结构性限制裁剪不可行弧（例如自环、直接回仓、送货指向起点等），不可行时将上界设为 0。目标系数为距离；从起点出发的弧附加 `10000` 大常数，鼓励模型减少使用车辆。【F:domain/solver/exact/TwoIndexFormulation.cpp†L289-L343】

### 服务开始时间 `b_i`
在 `[tw_{i}^{\min}, tw_{i}^{\max}]` 上创建连续变量。仓库起点的上下界被强制相等，固定其时间。【F:domain/solver/exact/TwoIndexFormulation.cpp†L343-L361】

### 车辆载重 `q_i`
在 `[\max(0, demand_i), \min(C, C + demand_i)]` 上创建连续变量。这里的上界允许送货节点在抵达时卸货后低于容量上限。【F:domain/solver/exact/TwoIndexFormulation.cpp†L363-L379】

### 路径标识 `v_i`
为每个真实节点创建连续变量 `[1, pickupCount]`，用于同步同一路径内节点的第一访问取货点索引；后续约束用它实现取送配对与子环消除。【F:domain/solver/exact/TwoIndexFormulation.cpp†L401-L408】【F:domain/solver/exact/TwoIndexFormulation.cpp†L530-L602】

> 注：数组 `pathVertexVariables` 仅在定价子问题中使用，因此主问题下其值保持默认初始状态。【F:domain/solver/exact/TwoIndexFormulation.cpp†L258-L302】

## 约束系统
1. **入度/出度**：主问题下每个真实节点入度、出度都固定为 1，从而强制完全覆盖；定价模式原本会改写为流平衡方程。【F:domain/solver/exact/TwoIndexFormulation.cpp†L410-L440】
2. **时间窗传递**：标准 MTZ 型约束，确保若 `x_{ij}=1` 则 `b_j` 至少等于 `b_i` 加上服务时间与行驶时间，使用 Big-M 处理未激活的弧。【F:domain/solver/exact/TwoIndexFormulation.cpp†L442-L475】
3. **载重传递**：同样的 Big-M 结构，保证沿弧运输后车辆载重增加 `demand_j` 并保持在容量以内。【F:domain/solver/exact/TwoIndexFormulation.cpp†L478-L509】
4. **取送先后**：送货节点的服务时间必须不少于对应取货节点的服务时间加上行驶与服务耗时。【F:domain/solver/exact/TwoIndexFormulation.cpp†L512-L528】
5. **同路约束**：利用 `v_i` 确保取货和送货共享同一路径标识，并将路径编号绑定到从仓库发出的第一条弧，以此阻断子环、保持顺序一致。【F:domain/solver/exact/TwoIndexFormulation.cpp†L530-L602】

若未来重新启用定价模式，还会追加“单条路径”“路径长度”之类的约束，但在主问题中不会触发。【F:domain/solver/exact/TwoIndexFormulation.cpp†L604-L627】

## 热启动与固定策略
若提供 `initSolution`，代码会根据 `freeNodes` 决定解锁粒度：

* **释放部分节点**：复制初始解，调用 `stringRemoval` 逐步释放 `freeNodesNum/2` 对取送节点，标记为“自由节点”。随后以原路线顺序填充所有变量的 `Start` 值，并对未释放的弧/时间/载重设置 `LB=UB`，从而锁定其取值。【F:domain/solver/exact/TwoIndexFormulation.cpp†L629-L706】
* **释放部分路径**：若 `freeNodes=false`，则根据 `freeRouteIndices` 标记整条路径可调整。未释放的路径同样通过 `LB/UB` 固定弧与资源变量；释放路径则仅提供 `Start` 值作为热启动提示。【F:domain/solver/exact/TwoIndexFormulation.cpp†L710-L781】

在两种模式下，都会更新终点仓库的 `Start` 值，使其不小于所有路线的回仓时间。【F:domain/solver/exact/TwoIndexFormulation.cpp†L701-L709】【F:domain/solver/exact/TwoIndexFormulation.cpp†L770-L781】

## 回调注册与解重建
主问题在 `model.update()` 后注册自定义 `CallBack`：

* 回调在 MIP 找到整数解时读取弧变量，提取所有从仓库出发的弧作为路线起点，随后沿 `x_{ij}` 重建整条路径。
* 借助 `newOriginalNodeIndexMap` 将“新索引”映射回原问题节点，逐条更新 `vrpSolution.routes`，计算时间窗/容量代价，并调用 `WriteSol` 输出到 `resultFolder` 下命名为 `<instance>.<vehicle_count>_<distance>_<tw>_<cap>.txt` 的文件。【F:domain/solver/exact/TwoIndexFormulation.cpp†L15-L165】【F:domain/solver/exact/TwoIndexFormulation.cpp†L785-L790】

定价模式没有注册回调，因此此逻辑仅在当前函数中生效。

## 求解与文件输出
在回调设置完毕后，模型会尝试写出 `.lp` 文件（若 `resultFolder` 可写），最后调用 `model.optimize()` 进入 Gurobi 求解。异常通过 `GRBException` 捕获并打印错误码、信息。结束时释放 `vars` 指针（虽然它保持为 `nullptr`，但保持与旧代码兼容）。【F:domain/solver/exact/TwoIndexFormulation.cpp†L792-L826】

## 结果汇总
综上，`twoIndexFormulationSolve` 以固定顺序执行以下步骤：

1. 拷贝初始解并建立新旧索引映射；
2. 构建带时间窗、载重与路径标识约束的主问题模型；
3. 依据配置将部分节点/路径设为可调整，以热启动 Gurobi；
4. 注册回调实时重建整数解并落盘；
5. 写出 `.lp` 模型、调用 Gurobi 优化。

函数自身不返回值，而是通过 `vrpSolution`、`resultFolder` 和回调输出提供最优或当前最好解的明细数据。【F:domain/solver/exact/TwoIndexFormulation.cpp†L183-L826】
