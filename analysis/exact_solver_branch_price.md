# exact solver Branch&Price 算法文档

本文聚焦 `domain/solver/exact/` 中精确求解器的 Branch&Price 流程，强调算法视角下的阶段划分、关键变量、循环控制与分支节点管理方式。与面向代码结构的项目文档不同，这里以“如何运行一次列生成 + 分支”来叙述，让接手者能够据此复现和扩展求解策略。

## 总体流程概览

Branch&Price 主流程由 `SetPartitioning` 类驱动：

1. **初始化与种子列**：解析 JSON 配置、读取实例/BKS/初始解，并从列文件加载历史路径，形成根节点的列池与时间窗参考。`SetPartitioning::init()`、`readStartingRoutes()` 完成这一步。【F:domain/solver/exact/SetPartitioning.h†L61-L310】【F:domain/solver/exact/SetPartitioning.cpp†L6-L57】
2. **根节点主问题（RMP）**：`createSP()` 根据当前列池构建放松的集合划分模型，含节点覆盖约束与（可选）车辆数约束。变量为连续型列权重，目标系数来自路线实际成本及罚项。【F:domain/solver/exact/SetPartitioning.cpp†L59-L116】
3. **求解与提取对偶**：`solveSP()` 调用 Gurobi 求解 RMP，并读取覆盖约束对偶值 `rho_k`，作为定价子问题的节点成本校正。【F:domain/solver/exact/SetPartitioning.h†L92-L108】
4. **定价子问题**：`solveTwoInd()` 把当前对偶传入二指标模型 `twoIndexFormulationSolveZ()`，求解带对偶调整的最短路（列生成）并返回负 reduced cost 的新路线集合。【F:domain/solver/exact/SetPartitioning.cpp†L119-L171】【F:domain/solver/exact/TwoIndChecker.cpp†L14-L200】
5. **列接受与对偶稳定**：`binSearch()` 将所有新路线写回列文件、更新 `routes`，并据 reduced cost 通过 λ-搜索平滑地更新工作对偶向量，迭代直至 reduced cost 非负，刷新全局对偶 `pi_k`。【F:domain/solver/exact/SetPartitioning.h†L116-L191】
6. **分支控制**：通过车辆数约束、`freeNodes`/`freeRouteIndices` 的弧固定策略以及列文件，手动刻画分支节点并在每个节点重复上述列生成循环，必要时输出 `two_index_formulation*.lp/.sol` 供调试。【F:domain/solver/exact/SetPartitioning.cpp†L75-L113】【F:domain/solver/exact/TwoIndexFormulation.cpp†L548-L720】【F:domain/solver/exact/TwoIndChecker.cpp†L559-L639】【F:domain/solver/exact/TwoIndChecker.cpp†L720-L739】

下文按阶段深入说明各组成部分的算法要点，并在结尾给出整体伪代码。

## 阶段 0：初始化与种子列

- `init()` 解析配置文件，载入实例数据、附加车辆数、求解时间上限与初始解路径，并将 BKS 的车辆数作为 `maxRoute` 约束基线。读取的 `latestStartServiceTime` 会写入 `refNodeSericeStartTimes`，供后续列验证使用。【F:domain/solver/exact/SetPartitioning.h†L61-L310】
- `readStartingRoutes()` 从列文件逐行解析“Route i : v1 v2 …”格式，按顺序累积弧长并补上回仓成本，同时建立节点到列的关联矩阵 `xy_relate`。随后调用 `validateRoute()` 检查容量、取送顺序、时间窗并返回罚分，将其叠加到列成本上，确保 RMP 目标反映违反约束的代价。【F:domain/solver/exact/SetPartitioning.cpp†L6-L57】【F:domain/solver/exact/SetPartitioning.cpp†L174-L211】

这一阶段的输出是：列池 `routes`、修正后列成本、节点-列关联及参考服务时间，为根节点 RMP 提供初始数据。

## 阶段 1：根节点主问题构建

- `createSP()` 会清空旧的 Gurobi 模型，重新导入列池。对于每条路线添加连续变量 `y_r ∈ [0,1]`，若当前分支节点尚未固定车辆数（`maxRoute < 0`），则在目标中加上 10000 的大罚项以惩罚新增车辆；否则使用真实路径成本。【F:domain/solver/exact/SetPartitioning.cpp†L59-L99】
- 仅对需求大于 0 的节点添加覆盖约束 `∑_{r∋i} y_r ≥ 1`，实现列生成框架常见的乘车需求覆盖。若 `maxRoute ≥ 0`，额外增加 `∑_r y_r - s ≤ maxRoute` 并引入带高罚系数的松弛变量 `s`，使得车辆数可被软约束控制，构成常见的分支条件之一。【F:domain/solver/exact/SetPartitioning.cpp†L87-L113】
- 模型构建完毕后导出 `rmp.lp`，便于排查根节点或后续分支的列池状态。【F:domain/solver/exact/SetPartitioning.cpp†L114-L115】

## 阶段 2：求解 RMP 与提取对偶

- `solveSP()` 执行 `model->optimize()`，随后读取所有需求节点覆盖约束的对偶值写入 `rho_k`。当车辆数尚未固定时，会跳过索引 0 的“车辆数约束”对偶，仅收集货运节点的价格。【F:domain/solver/exact/SetPartitioning.h†L92-L108】
- 日志输出当前 RMP 目标值，便于监控列生成收敛趋势。`rho_k` 同时作为 `binSearch()` 初始的工作对偶向量，实现对偶稳定化的 warm start。【F:domain/solver/exact/SetPartitioning.h†L92-L118】

## 阶段 3：定价子问题（Two-Index Formulation）

### 对偶注入与参数准备

- `solveTwoInd()` 重新读取配置，收集 `freeNodes`、`freeNodesNum`、`freeRouteIndices` 等分支控制参数，并将 RMP 的对偶值写入 `PricingSubProblemData.dualValueForPickupNodes`。若配置缺失最大取货节点数限制，则 fallback 为当前 BKS 中最长路线的节点数，避免子问题返回过长路线。【F:domain/solver/exact/SetPartitioning.cpp†L132-L170】

### 二指标模型构建

- `twoIndexFormulationSolveZ()` 首先将所有取送节点重编号，建立原始索引与新索引的双向映射，并在初始解的服务时间中提取 `nodeSericeStartTimes`，用于后续的时间窗裁剪与目标调节。【F:domain/solver/exact/TwoIndChecker.cpp†L14-L83】
- 与 `TwoIndexFormulation.cpp` 不同，`TwoIndChecker` 保留了 `pricingSubProblem` 标记并据此构建单路径的定价模型；`TwoIndexFormulation.cpp` 则会在重编号后将该标志清零，仅用于求解覆盖全部请求的原问题。【F:domain/solver/exact/TwoIndexFormulation.cpp†L193-L232】【F:domain/solver/exact/TwoIndChecker.cpp†L14-L83】
- 对每条可行弧 `(i,j)` 创建二进制变量 `x_ij`，不可行弧则将上界设为 0。目标系数以距离为主，并对 depot 出入弧施加额外奖励/惩罚，以匹配 RMP 的车辆成本；若 `i=0` 且运行在定价模式，则减去相应的对偶值，实现 reduced cost 计算。【F:domain/solver/exact/TwoIndChecker.cpp†L103-L158】
- 辅助变量：
  - `z_i` 控制选入路线的取货节点数量，只在定价模式启用。【F:domain/solver/exact/TwoIndChecker.cpp†L160-L169】
  - `b_i`（服务开始时间）在初始解附近截断范围，并以极小系数加入目标形成轻微稳定化；`q_i`（载重）和 `v_i`（路线标识）配合 `x_ij` 去除子回路并维护取送对同路性。【F:domain/solver/exact/TwoIndChecker.cpp†L172-L248】
- 约束体系覆盖：流量守恒/单路线约束、时间窗与容量 MTZ 不等式、取送先后顺序、路线 ID 同步、以及针对不可达节点组合的 2-节点与 3-节点割，显式剔除时间窗不可能满足的序列。【F:domain/solver/exact/TwoIndChecker.cpp†L400-L557】

### 分支节点的局部固定

- 若当前分支节点通过 `freeNodes=true` 释放部分请求，则从 warm-start 解中调用 `stringRemoval` 移除指定数量的节点，并对未释放的弧设置下界（锁定在 1），同时保留释放节点的自由度。【F:domain/solver/exact/TwoIndChecker.cpp†L559-L639】
- 若采用 `freeRouteIndices`（`freeNodes=false`），则仅释放指定线路，其余线路的弧、服务时间、载重与路线 ID 全部固定在原值，相当于在分支节点中只允许部分车辆重新优化。【F:domain/solver/exact/TwoIndexFormulation.cpp†L548-L720】

### 求解与结果回传

- 在原问题模式下注册 `CallBack`，于 `GRB_CB_MIPSOL` 中实时构造整数解并输出 `.txt` 文件；在定价模式下则直接求解模型并写出 `two_index_formulation_pricing_sub_problem.lp` 与 `.sol`，可用于排查列生成失败的原因。【F:domain/solver/exact/TwoIndexFormulation.cpp†L15-L170】【F:domain/solver/exact/TwoIndexFormulation.cpp†L700-L738】【F:domain/solver/exact/TwoIndChecker.cpp†L720-L739】
- `twoIndexFormulationSolveZ()` 遍历解池，将所有新路线的节点序列压入 `solVec`，并返回目标值减去补偿项 `objAdj`，对应 reduced cost 的估计。【F:domain/solver/exact/TwoIndChecker.cpp†L720-L739】

## 阶段 4：列接受与对偶稳定

- `binSearch()` 在每轮定价后遍历 `sol_vec_vec`，把所有候选路线以“Route k : …”格式写回列文件 `cf` 并追加到内存列池 `routes`，保持 RMP 与外部列缓存同步。【F:domain/solver/exact/SetPartitioning.h†L141-L158】
- 若返回的 reduced cost `tmp` 仍为负，则依据 `new_lambda = 1 + tmp / (column_cost - dual_sum_pi_k - tmp)` 更新平滑系数，使用凸组合在 `pi_k` 与当前 `rho_current` 之间移动，实现对偶稳定化；否则终止循环并将最终 `rho_current` 写回 `pi_k`，供下一次 RMP 使用。【F:domain/solver/exact/SetPartitioning.h†L159-L191】
- 日志打印每轮下界估计 `dual_sum`，便于监控 Lagrangian 下界的提升情况。【F:domain/solver/exact/SetPartitioning.h†L182-L185】

## 阶段 5：分支节点管理与搜索策略

虽然仓库中尚未实现自动的分支树管理，但代码提供了下列手段来手工或脚本化地探索 Branch&Price 节点：

- **车辆数分支**：通过 `setRoute()` 或在配置阶段修改 `maxRoute`，配合 RMP 中的车辆数约束与高罚松弛变量，模拟在树上固定“≤ k 辆车”的分支。【F:domain/solver/exact/SetPartitioning.h†L61-L78】【F:domain/solver/exact/SetPartitioning.cpp†L102-L113】
- **列文件持久化**：每个节点的列池都会写回同一列文件，可在人工分支时复制不同版本的列文件，实现“左右子树各自维护列缓存”的策略。【F:domain/solver/exact/SetPartitioning.cpp†L141-L158】
- **局部重优化分支**：借助 `freeNodes`/`freeRouteIndices` 控制哪些节点/车辆被释放，迫使子问题在指定结构内寻找改进路线，相当于在分支树节点上添加“该节点/车辆必须保留/必须调整”的判定。【F:domain/solver/exact/SetPartitioning.cpp†L137-L170】【F:domain/solver/exact/TwoIndexFormulation.cpp†L548-L720】
- **解与模型导出**：`rmp.lp`、`two_index_formulation*.lp/.sol` 以及回调写出的 `.txt` 解，可以作为分支节点的快照，用于验证列生成和分支约束是否匹配预期。【F:domain/solver/exact/SetPartitioning.cpp†L114-L115】【F:domain/solver/exact/TwoIndexFormulation.cpp†L700-L738】

综合运用这些机制即可搭建半自动的 Branch&Price 框架：在根节点完成列生成后，通过调节配置/列文件复制构造子节点，再在每个子节点重复阶段 1–4 的列生成循环。

## 伪代码总结

```text
procedure BranchAndPrice(config, column_file):
    sp ← SetPartitioning(vrp_data, problem, config, column_file, maxRoute_hint)
    repeat
        sp.solveSP()                      // 阶段 2
        negative ← false
        repeat                            // 阶段 3–4
            reducedCost, routes ← sp.solveTwoInd(sp.rho_current)
            if reducedCost < 0 then
                sp.appendColumns(routes)
                sp.updateLambda(reducedCost) // 对偶稳定
                negative ← true
            end if
        until reducedCost ≥ 0
    until not negative
    if branch_condition then             // 阶段 5
        for child in create_branch_nodes(config):
            BranchAndPrice(child.config, child.column_file)
    end if
```

其中 `appendColumns` 与 `updateLambda` 分别对应 `binSearch()` 中对列文件的写入和 λ-搜索步骤；`create_branch_nodes` 依赖人工/脚本修改 `maxRoute`、`freeNodes`、列文件等参数来实现具体的分支策略。

通过以上算法视角的梳理，可以在现有代码基础上实现自动分支、列池复制或高级裁剪，并更精准地调试列生成与对偶稳定过程。
