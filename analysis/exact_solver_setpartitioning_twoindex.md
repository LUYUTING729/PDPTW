# `SetPartitioning.{h,cpp}` 与二指标模型详解

本文聚焦 `domain/solver/exact/` 目录内的两大核心文件 —— `SetPartitioning.{h,cpp}` 与 `TwoIndexFormulation.{h,cpp}`。前者负责列生成主问题（RMP）的构建与迭代更新，后者给出二指标定价子问题的完整 Gurobi 建模与求解过程。下文将按成员变量、函数参数、运算流程及模块间调用关系进行拆解，帮助阅读者快速掌握精确求解框架的细节。

## `SetPartitioning` 类概览

### 关键成员

- `double gloabl_lb`：全局下界缓存（拼写留有遗留问题，当前实现未显式赋值）。
- `std::vector<double> pi_k` / `rho_k`：分别保存当前迭代的拉格朗日乘子基准值与“工作”乘子，用于列生成的双对偶更新。【F:domain/solver/exact/SetPartitioning.h†L34-L53】
- `std::vector<std::vector<int>> routes`：当前列池中的路径，每条路径以节点序列形式存储，并与列文件 `cf` 同步。【F:domain/solver/exact/SetPartitioning.cpp†L6-L56】
- `GRBEnv* env` / `GRBModel* model` / `std::vector<GRBVar> spVars`：Gurobi 环境、模型与列变量集合，用于维护 RMP。【F:domain/solver/exact/SetPartitioning.h†L37-L44】【F:domain/solver/exact/SetPartitioning.cpp†L58-L107】
- `std::unordered_map<int, GRBConstr> spConstrs`：以节点 ID 为键记录覆盖约束与车辆数量约束，便于读取对偶值。【F:domain/solver/exact/SetPartitioning.h†L41-L44】
- `VrpProblem* vp` / `VrpData* vd`：指向原问题数据与实例元信息，后者提供节点坐标、需求、时间窗等校验所需的静态属性。【F:domain/solver/exact/SetPartitioning.h†L46-L57】
- `int nX`：节点数量（含仓库），决定拉格朗日向量长度。【F:domain/solver/exact/SetPartitioning.h†L46-L57】
- `double lambda`：对偶向量更新用的平滑系数，初值 `0.04`，可通过 `setInitLambda()` 调整。【F:domain/solver/exact/SetPartitioning.h†L59-L64】【F:domain/solver/exact/SetPartitioning.cpp†L129-L178】
- `int maxRoute`：车辆数限制，若为负数表示放松车辆约束并在目标中施加大罚项；由构造参数或读取的 BKS 自动设置。【F:domain/solver/exact/SetPartitioning.h†L59-L69】【F:domain/solver/exact/SetPartitioning.cpp†L58-L107】
- `std::string configFileName` / `std::string cf`：分别指向 JSON 配置与列文件，用于驱动初始化、读写列。【F:domain/solver/exact/SetPartitioning.h†L71-L102】
- `std::vector<VrpSolution> initialSols`、`std::vector<double> refNodeSericeStartTimes`：缓存初始解族与参考服务时间，为定价子问题和可行性检验提供起点。【F:domain/solver/exact/SetPartitioning.h†L86-L134】

### 构造与初始化

`SetPartitioning(VrpData* vd_ori, VrpProblem* vp_ori, std::string config_file, std::string col_file, int maxRoute1)` 完成以下步骤：

1. 保存问题指针、车辆上限并初始化乘子向量长度为 `nX = vd->nodeList.size()`；
2. 构造 Gurobi 环境与模型指针，设置列文件路径 `cf`；
3. 调用头文件末尾内联定义的 `init()`，解析 JSON 配置：
   - 读取实例名、时间限制、初始解文件列表等元信息并打印日志（`additionalVehicleNum` 虽被读取但当前实现未使用）；
   - 从 `data/instance/` 载入 PDPTW 文本实例，并通过 `ConvertVrpProblem` 构造 `vrpProblem`；
   - 加载初始解及 BKS，转换为 `VrpSolution`，并以 BKS 的车辆数更新 `maxRoute`；
   - 提取初始解各节点的 `latestStartServiceTime`，写入 `refNodeSericeStartTimes` 作为时间窗参考。【F:domain/solver/exact/SetPartitioning.h†L61-L310】

初始化阶段会在控制台打印实例信息及初始解质量，以便校验输入。

### 核心方法与参数

| 方法 | 主要参数 | 功能要点 | 输出/副作用 |
| --- | --- | --- | --- |
| `setRoute(int mr)` | `mr`：车辆数上限 | 更新 `maxRoute`，影响 RMP 车辆数约束 | 即时修改成员，无返回值。【F:domain/solver/exact/SetPartitioning.h†L67-L69】 |
| `std::vector<double> validateRoute()` | - | 遍历 `routes` 中的每条路径，逐弧累加距离、服务时间，检查容量、取送顺序、时间窗，若违反则打印告警。额外依据 `refNodeSericeStartTimes` 计算时间窗偏差罚分。 | 返回每条路径的时间罚分，用于修正列成本。【F:domain/solver/exact/SetPartitioning.cpp†L108-L211】 |
| `void readStartingRoutes(std::string file_name, std::vector<std::vector<int>>& xy_relate, std::vector<double>& net_distance_vec)` | `file_name`：列文件；`xy_relate`：节点-列关联输出；`net_distance_vec`：列成本输出 | 解析列文件中的“Route i : a b c ...”格式，依次计算弧长、回仓成本，将节点到列的映射写入 `xy_relate` 并更新 `routes`。 | 填充 `routes`、`xy_relate`、`net_distance_vec`；调用 `validateRoute()` 后追加时间罚分。【F:domain/solver/exact/SetPartitioning.cpp†L6-L56】【F:domain/solver/exact/SetPartitioning.cpp†L108-L143】 |
| `void createSP(std::string file_name)` | `file_name`：列文件 | 清空旧模型、重新读取列，按列成本创建 Gurobi 连续变量（车辆受限时系数为原成本，否则加 10000 罚项），并仅为需求大于 0 的节点添加覆盖约束；若限制车辆数则额外加入带松弛变量的“≤ maxRoute”约束。 | 刷新 `model`、`spVars`、`spConstrs`，并导出 `rmp.lp` 供调试。【F:domain/solver/exact/SetPartitioning.cpp†L59-L115】 |
| `void solveSP()` | - | 调用 `createSP(cf)` 构建模型，`model->optimize()` 求解。随后读取覆盖约束的对偶值填充 `rho_k`（当未固定车辆数时跳过索引 0 的总车辆约束），并输出当前目标值。 | 更新 `rho_k`、保留 RMP 最优解供下一轮定价使用。【F:domain/solver/exact/SetPartitioning.h†L92-L108】 |
| `double solveTwoInd(const std::vector<double>& rho_kz, std::vector<std::vector<int>>& sol_vec)` | `rho_kz`：当前拉格朗日乘子；`sol_vec`：输出路径集合 | 重新读取配置，准备 `PricingSubProblemData`，将乘子写入 `dualValueForPickupNodes`，调用 `twoIndexFormulationSolveZ(...)` 求解二指标定价子问题；把返回的所有候选路径附加到列文件 `cf` 与 `routes`。 | 返回子问题目标（即列的 reduced cost），并在 `sol_vec` 中写入每条新路线的节点序列。【F:domain/solver/exact/SetPartitioning.cpp†L144-L184】 |
| `void binSearch(int iter)` | `iter`：当前迭代编号，用于日志 | 执行对偶向量的 λ 搜索：以 `rho_current` 为工作向量，不断调用 `solveTwoInd` 获取最负 reduced cost，若 `tmp <= -EPS` 则根据 column cost 更新 `lambda`、用凸组合收缩 `rho_current`，并将新列写入文件；若 reduced cost 已非负则终止，最后将 `pi_k` 更新为最终 `rho_current` 并打印下界。 | 更新 `routes`、`lambda`、`pi_k`，打印当前下界并为下一轮 `solveSP()` 提供乘子起点。【F:domain/solver/exact/SetPartitioning.cpp†L129-L211】 |
| `double getDistance(int x, int y)` | 节点索引 | 以节点经纬度近似计算欧式距离，供列解析、可行性校验使用。 | 返回两节点间距离。【F:domain/solver/exact/SetPartitioning.h†L60-L82】 |
| `void setInitLambda(double init_lambda)` | 新初值 | 修改 `lambda`，常用于调参。 | 更新成员，无返回值。【F:domain/solver/exact/SetPartitioning.h†L116-L118】 |

### 运行节奏与输出

1. **读取初始列**：`readStartingRoutes()` 将历史列加载到模型中，并通过 `validateRoute()` 修正成本；
2. **求解主问题**：`solveSP()` 获取覆盖约束对偶值，作为定价子问题的成本调整；
3. **定价 + 列扩充**：`binSearch()` 持续调用 `solveTwoInd()`，当 reduced cost 仍为负时将新路线写入列文件与内存；
4. **对偶更新**：根据 reduced cost 计算新的 `lambda`，将 `rho_current` 与 `pi_k` 做凸组合收缩，最终更新全局乘子；
5. **日志**：每次迭代输出当前列、λ、新增路线与下界估计，帮助观察收敛情况。

## `TwoIndexFormulation` 二指标模型

### 辅助结构

- `PricingSubProblemData`
  - `dualValueForPickupNodes`：从 Set Partitioning 传来的取货节点对偶值，用于调节目标系数；
  - `pickupNodeNumLimitInRoute`：定价子问题允许的最大取货节点数量。
- `PricingSubProblemSolution`
  - `nodesInRoute`：回传的单条路线节点序列；
  - `reducedCost`：路线的 reduced cost 估计值（当前实现未在 `twoIndexFormulationSolve` 中填充，主要在 `twoIndexFormulationSolveZ` 中使用）。【F:domain/solver/exact/TwoIndexFormulation.h†L7-L22】

### `twoIndexFormulationSolve` 参数说明

```
void twoIndexFormulationSolve(
    const VrpConfig& vrpConfig,
    const VrpProblem* vrpProblem,
    const VrpSolution* initSolution,
    bool freeNodes, int freeNodesNum,
    const std::vector<int>& freeRouteIndices,
    double timeLimit, bool pricingSubProblem,
    const PricingSubProblemData& pricingSubProblemData,
    PricingSubProblemSolution& pricingSubProblemSolution,
    VrpSolution& vrpSolution,
    const std::string& resultFolder)
```

- `vrpConfig`：罚系数配置，传递给可能使用的局部扰动/移除算子（例如 `stringRemoval`）。
- `vrpProblem`：包含节点数据、距离矩阵、车辆容量，用于构建线性约束。
- `initSolution`：可选的 warm-start 解。若存在，首先拷贝到 `vrpSolution` 作为初始结构，并根据 `freeNodes`/`freeRouteIndices` 决定放松或锁定哪些节点/线路。
- `freeNodes` & `freeNodesNum`：当需要释放若干请求由二指标模型重新布置时，调用 `stringRemoval` 移除对应节点并放宽相关变量上下界。
- `freeRouteIndices`：若采用“释放整条路线”的策略，按索引集合放松相应路径，使模型可以重排这些路线。
- `timeLimit`：Gurobi 求解时间上限（秒）。
- `pricingSubProblem`：标记当前是否作为定价子问题运行；但实现会在重编号后将其重置为 `false`，因此默认始终构建覆盖全部节点的主问题模型。
- `pricingSubProblemData`：仅在外部手动启用定价模式时才会使用到，默认情况下不会影响模型。
- `pricingSubProblemSolution`：用于回填定价子问题的最优路径数据（当前函数中未显式写入，但保持接口一致）。
- `vrpSolution`：输出的整数解结构；若启用回调，会在求解过程中被实时更新，并在获得新整数解时落盘。
- `resultFolder`：结果输出目录，用于写出 `.lp` 模型文件和 `.txt` 解文件。

虽然接口预留了定价模式相关参数，但 `TwoIndexFormulation.cpp` 会在节点重编号后统一把 `pricingSubProblem` 置为 `false`。因此下文的建模步骤对应的是“覆盖全部请求”的主问题；真正执行列生成定价的 `twoIndexFormulationSolveZ` 位于 `TwoIndChecker.cpp`，其差异将在后文单独说明。【F:domain/solver/exact/TwoIndexFormulation.cpp†L193-L217】【F:domain/solver/exact/TwoIndChecker.cpp†L14-L83】

### 建模步骤

1. **节点重编号**：遍历 `vrpProblem` 的节点，将所有取货节点及其对应的送货节点映射到新的索引区间 `[1, pickupNum]` 和 `[pickupNum+1, 2*pickupNum]`，并记录双向映射表，便于约束中引用原始索引。【F:domain/solver/exact/TwoIndexFormulation.cpp†L72-L141】
2. **Gurobi 环境与模型**：初始化日志、Gap、时间限制，并创建 `GRBModel` 实例。【F:domain/solver/exact/TwoIndexFormulation.cpp†L175-L208】
3. **变量定义**：
   - `arcVariables[i][j]`：二进制弧变量 `x_ij`，根据可行弧筛选上下界，目标系数为行驶距离；当前版本始终将 `i=0` 的弧系数加 10000 以惩罚新增车辆，代码中“减去对偶值”的分支未被触发。【F:domain/solver/exact/TwoIndexFormulation.cpp†L331-L352】
   - `pathVertexVariables`：仅在 `pricingSubProblem` 为真时才会创建的 0-1 变量，但由于该标志被重置为 `false`，默认不会出现。【F:domain/solver/exact/TwoIndexFormulation.cpp†L355-L364】
   - `startServiceTimeVariables` (`b_i`)：每个节点的服务开始时间，范围取节点时间窗，并在非 depot 情况下以 0 目标系数创建；`i=0` 的时间被固定在窗口下界。【F:domain/solver/exact/TwoIndexFormulation.cpp†L367-L380】
   - `vehicleLoadVariables` (`q_i`)：车辆载荷变量，范围由节点需求与车辆容量决定。【F:domain/solver/exact/TwoIndexFormulation.cpp†L381-L408】
   - `routeIdentifierVariables` (`v_i`)：连续变量标识节点所属路线首节点 ID，用于断开子回路并维系取送对的同路线约束。【F:domain/solver/exact/TwoIndexFormulation.cpp†L401-L408】
4. **流量约束**：
   - 若手动恢复定价模式，将使用入度 = 出度的流量守恒并配合 `z_i` 限制单条路径；
   - 默认情况下执行的是固定每个节点入度、出度均为 1 的完整覆盖约束。【F:domain/solver/exact/TwoIndexFormulation.cpp†L410-L439】
5. **时间窗（MTZ）约束**：对任意可行弧 `(i, j)` 添加 `b_i - b_j + M * x_ij ≤ M - travelTime`，保证时间累积合法。【F:domain/solver/exact/TwoIndexFormulation.cpp†L442-L476】
6. **载荷约束**：类似地添加 `q_i - q_j + M * x_ij ≤ M - demand_j`，维持容量守恒并防止超载。【F:domain/solver/exact/TwoIndexFormulation.cpp†L478-L509】
7. **取送先后与同路线约束**：
   - 对每对取送节点加入 `b_pickup - b_delivery ≤ -travelTime`，确保先取后送；
   - `v_pickup = v_delivery` 强制送货节点与取货节点属于同一路线；
   - 结合 depot 出发弧的上下界，约束 `v_i` 的取值范围，进而在所有弧 `(i, j)` 上加上下界/上界不等式以消除子回路。【F:domain/solver/exact/TwoIndexFormulation.cpp†L510-L520】
8. **定价特定约束**：若 `pricingSubProblem = true`，会限制离开仓库的路径数量及取货节点数；但在默认执行路径中这些约束不会被添加。【F:domain/solver/exact/TwoIndexFormulation.cpp†L520-L627】
9. **初始解/暖启动**：
   - 当 `freeNodes = true`，调用 `stringRemoval()` 随机释放若干节点，设置对应变量的 `Start` 值，并对未释放的弧添加下界，使模型继承原解结构；
   - 当 `freeNodes = false` 时，根据 `freeRouteIndices` 决定固定哪些路线，未释放线路的弧、时间、载荷、路线标识全部锁定在原值。【F:domain/solver/exact/TwoIndexFormulation.cpp†L548-L668】
10. **回调与模型导出**：
    - 在默认模式（`pricingSubProblem = false`）下注册自定义 `CallBack`，于 `GRB_CB_MIPSOL` 中重构整数解并写入 `resultFolder`；
    - 导出 `two_index_formulation_original.lp`（若手动启用定价模式则生成 `_pricing_sub_problem` 版本）以便调试。【F:domain/solver/exact/TwoIndexFormulation.cpp†L14-L170】【F:domain/solver/exact/TwoIndexFormulation.cpp†L700-L738】
11. **优化与异常处理**：调用 `model.optimize()`，捕获异常并清理临时变量。【F:domain/solver/exact/TwoIndexFormulation.cpp†L717-L740】

### 与 `TwoIndChecker` 的协同

`twoIndexFormulationSolveZ`（定义于 `TwoIndChecker.{h,cpp}`）在 `twoIndexFormulationSolve` 的基础上额外：

- 针对定价场景调整目标系数（例如 depot 弧加罚、服务时间顺序奖励）；
- 写出 `sp*.lp` / `sp*.sol`，迭代读取上一轮解作为温启动；
- 在求解后遍历解池，将每条路径的节点序列压入 `solVec`，并返回 `reduced cost`（减去时间窗修正项）；
- 提供更多裁剪约束（2/3 节点 cut）以加强松弛。【F:domain/solver/exact/TwoIndChecker.cpp†L1-L338】【F:domain/solver/exact/TwoIndChecker.cpp†L338-L705】

该函数正是 Set Partitioning 中 `solveTwoInd()` 调用的实现。理解 `TwoIndexFormulation` 的变量/约束组织有助于在阅读 `TwoIndChecker` 时辨析新增裁剪与目标调整的意义。

## 模块间调用关系

1. **主问题初始化**：`SetPartitioning::createSP()` 根据列文件构建 Gurobi 模型；
2. **求解与对偶提取**：`solveSP()` 调用 Gurobi，记录每个节点覆盖约束的对偶值 `rho_k`；
3. **定价子问题**：`binSearch()` 将 `rho_current` 传入 `solveTwoInd()`，后者调用 `twoIndexFormulationSolveZ()`：
   - `twoIndexFormulationSolveZ()` 构建与 `twoIndexFormulationSolve()` 类似的模型，但加入额外裁剪、时间窗参考等；
   - 求解完成后返回 reduced cost，并把所有候选路径通过 `sol_vec` 回传；
   - `SetPartitioning` 将新路径持久化到列文件并更新 `routes`；
4. **对偶更新与循环**：若 reduced cost 仍为负，`binSearch()` 继续调整 `lambda` 并重复定价；否则退出循环，刷新 `pi_k` 为新的工作乘子，为下一轮主问题求解提供初值。

通过以上流程，`SetPartitioning` 与二指标模型协作完成经典的列生成：主问题提供对偶价格，定价子问题寻找负 reduced cost 的路线列，回补主问题后迭代逼近最优解。掌握两侧的参数、约束和日志输出有助于调试列生成收敛、排查不合法路线、或按需增加新的裁剪条件。
