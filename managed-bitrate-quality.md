# Vorbis 受管码率（managed bitrate）下的瞬态质量与蓄水池约束分析

代码基线：本仓库 libvorbis（分支 task/40）。核心文件：`lib/bitrate.c`、`lib/bitrate.h`、`lib/mapping0.c`、`lib/block.c`、`lib/envelope.c`、`lib/vorbisenc.c`、`lib/res0.c`、`lib/floor1.c`。

## 0. 机制总览：受管模式实际做什么

受管模式不是"边编码边压码率"，而是**每个音频块先按 15 个质量档各编码一遍，再按蓄水池账本挑一档输出**：

- `PACKETBLOBS = 15`（`lib/codec_internal.h:28`）。`packetblob[7]`（即 `PACKETBLOBS/2`）就是非受管 VBR 会输出的那个包（`lib/block.c:88-95`），其余 14 个是围绕它的升降档变体。
- 仅在 `vorbis_bitrate_managed(vb)` 为真时，mapping 才编码全部 15 档（`lib/mapping0.c:596-598`）。各档差异来自三条路径：
  1. floor1 拟合曲线：在基准掩蔽曲线之外多做 hi/lo 两次拟合，中间档线性插值（`lib/mapping0.c:500-572`）；
  2. 每档独立的滑动低通与声道耦合点：`sliding_lowpass[vb->W][k]`、`coupling_pointlimit`（`lib/mapping0.c:638-646`，参数在 `lib/vorbisenc.c:272-292` 按块尺寸换算成频点）；
  3. 受管专用残差码书 `books_base_managed` / `book_aux_managed`（`lib/vorbisenc.c:476-490`）。
- 选档与账本在 `vorbis_bitrate_addblock()`（`lib/bitrate.c:73-227`）完成；`vorbis_bitrate_flushpacket()` 把选中的 blob 作为输出包（`lib/bitrate.c:229-252`）。
- 受管模式**只在 `reservoir_bits > 0` 时启用**（`lib/bitrate.c:34`）。这是一个静默边界：蓄水池设为 0 不报错，而是整个 min/max/avg 管理失效，退化为纯 VBR。

## 1. 四类约束的优先级与失败边界

### 1.1 约束分类：单包限制 vs 时间窗口限制

| 约束 | 类型 | 代码位置 | 作用域 |
|---|---|---|---|
| 平均码率 avg | 软约束，floater 慢速跟踪 | `lib/bitrate.c:101-136` | 时间窗口（受 `slew_damp` 阻尼，约秒级） |
| 瞬时上限 max | 硬约束，但经蓄水池折算 | `lib/bitrate.c:153-162` | 名义上是窗口约束；蓄水池耗尽时退化为单包截断 |
| 瞬时下限 min | 硬约束，经蓄水池折算 | `lib/bitrate.c:141-150` | 同上；兜底手段是零填充 |
| 蓄水池 reservoir | 把 min/max 从"每包"变成" leaky bucket 窗口"的账本 | `lib/bitrate.c:194-218` | 窗口长度 = `reservoir_bits / avg_rate` 秒（`lib/vorbisenc.c:841-844`） |
| 长/短块选择 | 纯信号驱动，**不参与码率决策** | `lib/block.c:557-569`、`lib/envelope.c:215` | 单块 |

关键换算：每块比特目标按"短块半采样数"为基准（`avg_bitsper = avg_rate * (blocksizes[0]/2) / rate`，`lib/bitrate.c:41-43`），长块目标再乘 `short_per_long = blocksizes[1]/blocksizes[0]`（`lib/bitrate.c:38, 84-85`）。即**码率目标是按时间（每秒比特）而非按包恒定**的——长块允许成比例地大。

### 1.2 冲突时的优先级（`vorbis_bitrate_addblock` 内的实际执行顺序）

1. **avg floater 先动**（`lib/bitrate.c:101-136`）：按 `avg_reservoir` 与期望水位 `desired_fill = reservoir_bits * reservoir_bias` 的偏差方向逐档试探，且每块调整量被限 slew：`slewlimit = 15 / slew_damp` 档/秒（`lib/bitrate.c:104, 131-134`）。默认 `slew_damp = 1.5`（`lib/vorbisenc.c:993`），即 floater 扫完全部 15 档最快也要约 1.5 秒。**avg 是慢变量，故意不能响应瞬态。**
2. **min 其次**（`lib/bitrate.c:141-150`）：仅当"按当前档位记账后蓄水池会跌破 0"时才升档。
3. **max 最后**（`lib/bitrate.c:153-162`）：仅当"记账后蓄水池会溢出 `reservoir_bits`"时才降档。因为 max 在 min 之后执行，**两者同时触发时 max 覆盖 min 的选档结果**。
4. **边界兜底**（`lib/bitrate.c:167-190`）：
   - 降到 0 档仍超限（`choice < 0`）→ **物理截断包**：`oggpack_writetrunc` 截到 `maxsize = (max_target + reservoir_bits − minmax_reservoir)/8` 字节。max 由此成为**绝对的单包上限**。
   - 升到顶档仍不足 → 钳到 14 档并**零填充**到 `minsize`。min 由此成为**绝对的单包下限**（填充总能满足，代价是浪费比特）。

因此优先级可概括为：**块大小选择（信号驱动）> max 硬上限（可截断）> min 硬下限（可填充）> avg 软目标（被前两者随时覆盖）**。蓄水池决定 min/max 何时从"窗口约束"退化为"单包约束"：池内有余量时单包可以超限，池空/池满时约束立即作用于当前包。

### 1.3 失败边界（源码可确证的）

- **蓄水池 = 0**：受管模式静默关闭（`lib/bitrate.c:34`），min/max/avg 全部失效，无任何错误返回。
- **蓄水池耗尽于瞬态突发**：max 分支逐档降无可降 → 截断（`lib/bitrate.c:173-176`）。这是窗口约束的失败边界，也是瞬态质量的最坏路径。
- **min > max 等不变式违反**：`OV_ECTL_RATEMANAGE2_SET` 拒绝（`lib/vorbisenc.c:1122-1148`），检查项仅有 min ≤ avg ≤ max（>0 时）、damping > 0、reservoir ≥ 0、0 ≤ bias ≤ 1。**这些检查只保不变式，不保质量。**
- **API 误用**：受管模式下若绕过 `vorbis_bitrate_addblock` 直接 `vorbis_analysis(vb, &op)` 取包，返回 `OV_EINVAL`（`lib/analysis.c:50-53`）。
- **avg_reservoir 无界**：`lib/bitrate.c:221-224` 只累加不钳制，长期偏置只通过 floater slew 缓慢纠正；极端输入下 avg 的收敛速度完全由 `slew_damp` 决定。

## 2. 对瞬态质量、码率预算与可解码性的影响

### 2.1 源码能直接支持的结论

**瞬态与块结构**：瞬态检测（`_ve_envelope_search` / `_ve_envelope_mark`）完全由信号包络驱动，与码率账本无数据交互（`lib/block.c:557-569, 606`；`lib/envelope.c:215-327`）。瞬态触发短块（`BLOCKTYPE_IMPULSE`），短块序列使每秒包数上升 `short_per_long` 倍，但每包比特目标同比缩小（§1.1 的换算），因此**块切换本身不冲击码率预算——冲击来自瞬态频谱更难压缩，同档位下比特需求上升，差额由蓄水池吸收**。

**瞬态期间的实际码率路径**：avg floater 因 slew 限制（约 1.5 s 扫全程）跟不上突发，瞬态的额外比特首先计入 `minmax_reservoir`；只有当累计超发将溢出 `reservoir_bits` 时才降档。`reservoir_bias` 默认 0.1（`lib/vorbisenc.c:995`），即期望水位贴近池底——账本偏向"平时把池子腾空留着给突发用"（`lib/bitrate.c:194-218` 的回拉逻辑）。**蓄水池比特数 ÷ 平均码率 = 瞬态可全额报销的最长时长**；超过此时长的连续瞬态必然降档或截断。

**降档的质量代价结构**：降一档不只是"量化更粗"，而是同时生效的三种退化——floor 曲线更保守（插值拟合）、滑动低通下移（`sliding_lowpass[vb->W][k]`，高频被整体切除）、耦合点下移（更多频段转为强度立体声）。截断则是第四种退化：残差尾部直接丢失。

**可解码性**：截断与填充后的包仍是合法 Vorbis 包，且解码端对截断容错——`lib/res0.c:650` 注释明确"a truncated packet here just means 'stop working'; it's not an error"，残差解码到包尾即停，未解码系数为零；floor 解码遇包尾返回 NULL，该声道本帧按全零处理（`lib/floor1.c:1037`、`lib/mapping0.c` 逆映射中 `nonzero[i]=0` 分支）。零填充的尾部字节被解码器忽略。**因此受管模式的任何失败路径都不产生解码错误或不兼容流，只产生质量降级——"能解码"与"质量未受损"必须分开判定。**

### 2.2 必须依赖质量证据、源码无法给出的部分

- 蓄水池"多大才够"：源码只给出会计规则（窗口秒数公式），某窗口下瞬态降档是否可闻（预回声、高频塌缩、声像漂移）取决于信号与听感，需 PEAQ/ABX 等客观或主观评测。
- 15 档插值的质量单调性与均匀性：floor 拟合插值（`floor1_interpolate_fit`）和每档低通/耦合步进是否产生单调、可接受的降级梯度，源码无任何保证，需逐档测质。
- `slew_damp = 1.5 s`、`reservoir_bias = 0.1`、`impulse_noisetune`（`lib/vorbisenc.c:803-807, 1186-1189`，仅作用于 impulse 块）等调参的经验性质：它们是 `lib/modes/` 模板中的调音结果，改动后的听感影响只能实测。
- 截断造成的残差丢失在时频上的具体可闻度：解码器"不报错"不等于"不可闻"。

## 3. 跨配置兼容判据

判据分三层，**不要把"无崩溃/能解码"当作质量结论**：

1. **比特流兼容（可判，源码保证）**：受管是纯编码端机制，输出流始终是标准 Vorbis I；任何合规解码器都能解码任何 reservoir/limit 配置下的流，包括被截断和填充的包（§2.1）。判据：流通过规范解码器无错误回放——这只是兼容性的下限。
2. **编码器 API 与参数不变式（可判，源码强制）**：
   - 必须经 `vorbis_bitrate_addblock`/`vorbis_bitrate_flushpacket` 取包，否则 `OV_EINVAL`；
   - `min ≤ avg ≤ max`（>0 时）、`damping > 0`、`0 ≤ bias ≤ 1`、`reservoir_bits > 0`（为 0 时静默退化为 VBR——配置审查时必须显式检查 `vorbis_bitrate_managed()` 而非假设生效）。
3. **质量等效判据（需质量证据，源码不能判定）**：跨配置比较时，应以"瞬态段在目标窗口内不触发降档/截断"为可计算的必要条件——
   - 窗口时长 `reservoir_bits / avg_rate` ≥ 预期连续瞬态时长；
   - 单包绝对上限 `(max_bitsper·(W?short_per_long:1) + reservoir_bits − minmax_reservoir)/8` 字节 ≥ 瞬态块在最差蓄水池状态下的需求；
   满足后仍需以客观质量度量（如 PEAQ ODG）或听测确认瞬态段质量不劣于基线配置，方可宣称两配置"质量兼容"。仅核对参数含义或确认编码不崩溃，均不构成该结论。

## 附：关键符号速查

| 符号 | 含义 | 位置 |
|---|---|---|
| `PACKETBLOBS` | 每块预编码质量档数（15） | `lib/codec_internal.h:28` |
| `avgfloat` / `slew_damp` | avg 浮点档位 / 限速（默认 1.5 s 扫全程） | `lib/bitrate.c:45,104` |
| `minmax_reservoir` / `reservoir_bits` | min/max  leaky bucket 水位与容量 | `lib/bitrate.c:194-218` |
| `reservoir_bias` | 期望水位比例（默认 0.1，偏向留空） | `lib/vorbisenc.c:995` |
| `short_per_long` | 长/短块长度比，用于长块目标放大 | `lib/bitrate.c:38` |
| `desired_fill` | `reservoir_bits * reservoir_bias` | `lib/bitrate.c:87` |
