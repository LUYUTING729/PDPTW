# `domain/` 目录总览

`domain/` 目录承载了 PDPTW 求解器的业务建模与算法核心，按照职责可拆分为 API 接口层、领域模型（含实体定义与解表示）、算法求解器以及基础工具常量几大部分。本说明试图帮助新成员快速了解每个子目录的角色、文件组织与关键方法之间的协作关系。

## 目录结构概览

```
domain/
├── api/                 # 对外暴露的求解接口定义
├── model/               # 问题、解与实体的数据结构
│   └── entity/          # 细粒度实体：节点、路径、配置等
├── solver/              # 多套求解策略的实现（启发式、精确法等）
│   ├── exact/           # 基于列生成 / Set Partitioning 的精确算法
│   ├── hgs/             # 试验性的混合遗传策略（Order Crossover 等）
│   └── memetic/         # 主力启发式：Memetic + LNS + 多种局部搜索
└── utils/               # 通用枚举、常量
```

## `api/`：求解器接口层

- **`IVrpSolver.h`**：预留的求解器抽象接口，目前仅包含 include guard。后续若要对接多种求解策略，可在此放置纯虚基类定义，统一 `solve()`、`configure()` 等方法签名。

## `model/`：领域数据模型

### 问题与基础实体（`model/entity/`）

- **`VrpNode`**：封装节点 ID、坐标、节点类型（取货/送货/仓库）、需求、服务时间、时间窗、对应节点 ID 以及可行前继/后继集合，作为构造可行路径的原子单元。
- **`VrpData`**：对应输入实例的全量静态数据，包括节点列表、距离矩阵、车辆容量上限、可选车辆数、取货节点索引集合等。
- **`VrpRoute` / `RouteNode`**：按车辆划分的路径视图与链表化的路段节点，提供 `clearData()` 重置、维护行驶距离、时间窗罚分、容量违约等指标，支撑解中局部操作（插入、移除）。
- **`VrpConfig`**：记录求解过程中各类成本系数（车辆成本、时间窗罚系数、容量罚系数），在不同算法模块间传递权重。

### 问题聚合与解表示

- **`VrpProblem`**：聚合 `VrpData` 并缓存邻接表、请求间距离/需求差，用于启发式在局部搜索时快速查询相似性。
- **`VrpSolution`**：核心解结构，内部维护车辆路径数组、路段节点池、未指派请求、各类罚分及可行性标记。提供丰富的操作接口：
  - 构造函数根据 `VrpProblem` 初始化路段节点并建立取送节点互指。
  - `insertNode()` / `removeNode()`：对取送对进行插入、删除并维护链表关系。
  - `updateRouteData()`：重算某条路径的时间窗、容量、距离等指标，同时执行前向/后向检查。
  - `copyRoute()` / `copySolution()`：在代际演化、局部搜索过程中复制个体。
  - `calTotalValue()` 及各类 `getTotal*`：延迟计算总距离、时间窗/容量罚分、目标值。
  - `checkData()`：在调试模式下验证链表一致性、互指关系等，帮助捕捉结构破坏。
- **`VrpSolutionPool`**：存放最优/最差解指针，可用于元启发式中的精英策略。

## `utils/`：辅助常量

- **`Utils.h`**：定义节点类型枚举、节点/车辆数量上限以及双精度最大值常量，供模型与算法模块引用。

## `solver/`：求解策略集合

### `memetic/`：混合启发式框架

该目录实现主力求解流程：以 Memetic 算法为骨架，融合局部搜索、LNS、扰动与多种交叉算子。

- **`Memetic.{h,cpp}`**：
  - `buildInitialSolution()` 将取货节点平均分配到车辆，快速构造可行初始群体。
  - `memetic()` 主循环维护种群，依次执行：初始化/局部搜索、基于 SREX/EAX 的交叉生成子代、罚系数自适应、最优解落盘。
  - 内部大量使用 `VrpSolution` 的拷贝、插入、数据更新能力，并与 `io/vrp_sol_writer.h` 协作输出解。
- **局部改进 (`LocalSearch.{h,cpp}`)**：提供 `outRelocateMove`、`outExchangeMove` 等操作，对跨线路的取送对进行重新插入/交换，借助 `calculateRemovalCost` 与 `calInsertionData` 评估增量成本。
- **大邻域搜索 (`LargeNeighborhoodSearch.{h,cpp}`)**：循环执行随机 / 最坏 / Shaw / 串式等多种移除策略，配合贪心插入与“插入空路线”机制，加速恢复可行解。
- **扰动 (`Perturb.{h,cpp}`)**：通过随机选择两条路径互换请求，实现小幅度跳出局部最优。
- **插入算子 (`insert/Insert.{h,cpp}`)**：封装插入候选的增量指标，支持“眨眼”随机化贪心插入、不同排序策略以及可行性筛选。
- **移除算子 (`removal/Removal.{h,cpp}`)**：实现随机、最坏、Shaw、串式等多种请求移除策略，同时提供 `calculateRemovalCost()` 供局部/大邻域调用。
- **交叉相关 (`crossover/` 子目录)**：
  - `CrossoverStrategy.h` 定义交叉策略枚举。
  - `EdgeAssemblyCrossover.{h,cpp}` 实现 EAX 图构造、环检测、子路径拼接与再插入逻辑，必要时调用 LNS/局部搜索巩固可行性。
  - `SrexCrossover.{h,cpp}`、`RouteSubset.{h,cpp}`、`LcsDp.{h,cpp}` 组合实现 Selective Route Exchange（SREX），利用最长公共子序列 DP、子路径候选集合等技术选择交换子结构。

### `exact/`：列生成与严格可行性校验

- **`SetPartitioning.{h,cpp}`**：封装列生成框架，内部借助 Gurobi 维护主问题、拉格朗日乘子、`binSearch()` 做 λ 二分搜寻，同时与 `Memetic`、`Insert`、`Removal` 组件交互生成/修复列。
- **`TwoIndexFormulation.{h,cpp}`**：实现二指标模型求解与回调逻辑，利用 Gurobi 回调抽取整数解、构造子路线并落盘，可用于列生成子问题或独立求解。
- **`TwoIndChecker.{h,cpp}`**：对二指标模型的包装与校验，复用 `twoIndexFormulationSolve`，为 Set Partitioning 提供列成本评估接口。
- **`main2.cc`**：示例主程序，读取 `config.json` 与实例数据，构造 `SetPartitioning` 后多轮执行 `binSearch` + `solveSP()`；需按 README 指示修改路径/参数。
- **`README.md`**：说明如何配置列文件、Gurobi 路径以及运行流程。

### `hgs/`：实验性混合遗传组件

- **`ox.{h,cpp}`**：实现 Order Crossover 与最短路分解策略，包含 `oxCross()`、`decompose()`、`spDA()` 等方法，用于从两条解中重组顺序并通过最短路生成可行路线；`main2.cc`（如存在）可单独运行测试。
- `.gitkeep` 用于保持空目录在版本库中。

## 典型协作关系

1. **数据流**：`io/` 模块读取实例后填充 `VrpProblem` → `Memetic`/`SetPartitioning` 等求解器消耗 `VrpSolution` 进行优化 → `io/vrp_sol_writer` 输出结果。
2. **启发式循环**：`Memetic::memetic()` 调用 `LocalSearch`、`LargeNeighborhoodSearch`、`Perturb`、`Insert`、`Removal`，同时动态调节 `VrpConfig` 的罚系数，依赖 `VrpSolution` 的增量更新保证效率。
3. **精确法互补**：`SetPartitioning` 可利用 `TwoIndexFormulation` 求解子问题；在构造初始列或修补解时，同样复用 `memetic/` 下的插入/移除工具，以减少重复实现。

> 若后续扩展新的求解策略，建议复用既有的 `VrpSolution` 操作接口，并在 `api/IVrpSolver` 补全统一入口，以简化主程序切换不同算法的成本。
