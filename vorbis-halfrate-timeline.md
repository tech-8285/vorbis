# Vorbis 半速解码、定位与交叉淡化的时间线分析

> 分析对象：本仓库 libvorbis（libvorbis 1.3.7 基线 + 本分支提交
> `1b75110 [vorbisfile] Fix use-after-free in ov_pcm_page_seek`）。
> 所有结论均基于实际源码，并以一次独立的 out-of-tree 构建 + 实测程序补证。
> 文中"实测"指本报告第 9 节描述的实验（源码未改动；仅在一份**副本**中加过
> 临时打印，且已与未改动构建的结果交叉核对）。

---

## 1. 问题与三类"时间量"

半速（halfrate）解码把**输出 PCM 的采样密度减半**，但 Vorbis 的交叠定位
（overlap-add / lap）和跨逻辑流（chained link）衔接仍必须表达**原始音频
时间线上的播放位置**。代码里同时存在三套不同单位的时间量，必须先分清：

| 时间量 | 单位/域 | 存放位置 | 半速下是否改变 |
|---|---|---|---|
| **原始流采样位置**（granule 域） | 编码时 `vi->rate` 的 PCM 样本序号 | Ogg 页 `granulepos`；`vf->pcm_offset`；`vd->granulepos`；`b->sample_count` | **不变**，仍是原速率刻度 |
| **输出 PCM 样本位置**（解码缓冲域） | 实际交给应用的 float 样本序号 | `vd->pcm_returned / pcm_current`；`ov_read_float()` 的返回值与读入量 | **减半** |
| **墙钟时间**（秒） | 秒 | `ov_time_tell / ov_time_total` | 刻度不变（仍按原速率换算） |

源码用一个位移量 `hs = ci->halfrate_flag`（取值 0 或 1）在两个域之间换算，
核心恒等式是：

```
原始流样本数 = 输出 PCM 样本数 << hs          （乘以 2^hs）
输出 PCM 样本数 = 原始流样本数 >> hs          （除以 2^hs，向下取整）
```

`hs` 只出现在解码侧（`lib/block.c`、`lib/synthesis.c`、`lib/vorbisfile.c`），
编码侧不读取该标志（`codec_internal.h:130` 注释明确写着
"painless downsample for decode"）。

---

## 2. 半速模式在源码里到底做了什么

### 2.1 开关与守卫

- `vorbis_synthesis_halfrate(vi, flag)`（`lib/synthesis.c:166-174`）只设置
  `ci->halfrate_flag`，但有硬性守卫：
  ```c
  /* right now, our MDCT can't handle < 64 sample windows. */
  if(ci->blocksizes[0]<=64 && flag) return -1;
  ```
  即**小 blocksize ≤ 64 时拒绝开启半速**，返回 -1 且标志保持 0。
  实测（直接构造 `codec_setup`）：`bs0=64` 被拒、`bs0=128/256/512` 全部接受，
  与源码完全一致。
- `vorbis_synthesis_halfrate_p()`（`synthesis.c:176-179`）只读标志。

### 2.2 半速发生在 MDCT 逆变换长度上，而不是事后抽点

`_vds_shared_init()`（`lib/block.c:170-262`）按**半长**初始化逆 MDCT：

```c
hs=ci->halfrate_flag;                                  /* block.c:182 */
mdct_init(b->transform[0][0], ci->blocksizes[0]>>hs);  /* block.c:197 */
mdct_init(b->transform[1][0], ci->blocksizes[1]>>hs);  /* block.c:198 */
```

而 `mapping0_inverse()`（`lib/mapping0.c:698-799`）里：

- `vb->pcmend = ci->blocksizes[vb->W]`（`mapping0.c:706`）——块向量仍按
  **完整 blocksize** 分配，频谱系数也仍是完整 `n/2` 条；
- 逆变换 `mdct_backward(b->transform[vb->W][0], pcm, pcm)`
  （`mapping0.c:794`）作用于 `n>>hs` 长度的查找表，因此每个块只产出
  **一半长度的时域向量**，写入完整 `vb->pcm` 的前半段。

也就是说，半速是"**用一半长度的 IMDCT 重建时域**"，配合下面的半长交叠窗口
完成 2:1 降采样，而不是先全速率解码再简单丢样本。

### 2.3 交叠/窗口尺度全部按 `>>hs`（或 `>>(hs+1)`）缩放

`vorbis_synthesis_blockin()`（`block.c:731-944`）是理解尺度的关键：

```c
int hs=ci->halfrate_flag;                            /* block.c:735 */
int n =ci->blocksizes[v->W]>>(hs+1);                 /* block.c:755 当前块半窗长 */
int n0=ci->blocksizes[0]>>(hs+1);                    /* block.c:756 短块半窗长 */
int n1=ci->blocksizes[1]>>(hs+1);                    /* block.c:757 长块半窗长 */
```

四种 lW/W 组合（大/大、大/小、小/大、小/小，`block.c:781-815`）的
overlap-add 循环长度都用 `n0/n1`，窗口表通过
`_vorbis_window_get(b->window[W]-hs)` 选取，即窗口索引也随 `hs` 下移一档。

窗口表本身是固定的 8 张半窗表 `vwin64..vwin8192`，长度依次为
32,64,128,…,4096（`lib/window.c:2087-2100`），表 `k` 的长度为 `32<<k`。
设 `idx = window[W]-hs`，其表长 `32<<idx` 恰好等于 `blocksizes[W]>>(hs+1)`
（即 `n0`/`n1`），因此 overlap 循环的最大下标 `n-1` 始终落在表内。
**这一尺度恒等式在未改动构建的全速率/半速全路径 ASan 运行中均无越界报告**
（见第 9 节）。

每个块交叠后真正"新产出"的样本量（`block.c:840-842`）：

```c
v->pcm_current = prevCenter +
    ((ci->blocksizes[v->lW]/4 + ci->blocksizes[v->W]/4) >> hs);
```

全速率时相邻块中心推进 `(bs_l + bs_c)/4` 个原速率样本；半速时该推进量
右移 1 位，输出端只产生一半样本——这正是输出密度减半的块级来源。

### 2.4 原始流位置的记账仍然用完整 blocksize，且只在必要处换算

同函数后半段（`block.c:858-937`）维护 `b->sample_count` 与 `vd->granulepos`，
这里的累加**不带 `hs`**：

```c
b->sample_count += ci->blocksizes[v->lW]/4 + ci->blocksizes[v->W]/4; /* :861 */
v->granulepos   += ci->blocksizes[v->lW]/4 + ci->blocksizes[v->W]/4; /* :907 */
```

即**原始流刻度完全不受半速影响**。只有在需要把"多出来的原始样本"折算成
输出缓冲样本去裁剪时才右移 `hs`，例如 EOF 短帧裁剪：

```c
if(extra > (v->pcm_current - v->pcm_returned)<<hs)
    extra = (v->pcm_current - v->pcm_returned)<<hs;   /* block.c:892-893 */
v->pcm_current -= extra>>hs;                          /* block.c:895 */
```

以及起始裁剪 `v->pcm_returned += extra>>hs;`（`block.c:898`）。

---

## 3. 输出 PCM 长度 ↔ 原始流采样位置 ↔ 交叠窗口尺度的对应关系

### 3.1 稳态对应（可由源码直接证实）

对连续解码，设应用累计取走 `M` 个**输出**样本，则原始流播放位置推进

```
Δgranule = M << hs = M · 2^hs
```

证据链：`ov_read_float()`（`vorbisfile.c:2124-2153`）与
`ov_read_filter()`（`vorbisfile.c:2095-2097`）每取 `samples` 个输出样本后执行
`vf->pcm_offset += (samples<<hs)`；`_fetch_and_process_packet()` 中
`vf->samptrack += (vorbis_synthesis_pcmout(...)<<hs)`（`vorbisfile.c:725`）
同理。

### 3.2 实测数值（单链路、1 秒、不同原速率）

| 原速率 | bs0/bs1 | 全速率输出 | 半速输出 | 半速 `pcm_tell` 末值 | `pcm_total` |
|---|---|---|---|---|---|
| 44100 | 256/2048 | 44100 | 22050 | 44100 | 44100 |
| 32000 | 256/2048 | 32000 | 16000 | 32000 | 32000 |
| 16000 | 512/1024 | 16000 | 8000 | 16000 | 16000 |
| 8000  | 512/512  | 8000  | 4000 | 8000 | 8000 |
| 11025 | 512/512  | 11025 | **5513** | **11026** | 11025 |

- 前四行：`输出数 × 2 = pcm_total`，`pcm_tell` 末值精确等于 `pcm_total`，
  对应关系严格成立。
- 11025 行：总样本数 11025 为奇数，半速输出 5513（=⌈11025/2⌉），而
  `pcm_tell` 末值 **11026 比 `pcm_total` 多 1**。根因是 vorbisfile 层
  `pcm_offset += samples<<1` 对奇数 total 向上取整（`5513·2=11026`）；
  blockin 的 EOF 裁剪负责把**输出样本数**限制正确，但不回改该记账值。
  这是整数右移/左移在奇数 granule 上的边界现象，第 7 节专门分析。

### 3.3 交叠窗口尺度的对应（可由源码直接证实）

- 全速率短块半窗长 = `bs0/2`；半速短块半窗长 = `bs0/4 = bs0>>(1)`。
  44.1kHz（bs0=256）：全速率 128、半速 64；8kHz（bs0=512）：全速率 256、
  半速 128。
- 跨流淡化 `ov_crosslap()` 的淡化长度取
  `n1 = vorbis_info_blocksize(vi1,0)>>(1+hs1)`（`vorbisfile.c:2301-2302`），
  seek-lap 同理（`:2383`、`:2465`）。因此淡化窗口在**输出域**始终是
  "半个短块"，在原始流时间线上它覆盖的物理时长保持不变
  （`(bs0/2)/rate` 秒），只是由一半数量的输出样本表达。
- 实测：44.1kHz crosslap 真正改写新流前 126 个全速率样本（≈半短块 128），
  半速时改写前 63 个样本（≈半短块 64），之后两路输出逐样本相同
  （`lapcmp` 实测，splice 区外 RMS 差为 0）。

### 3.4 缓冲容量不随半速收缩（可由源码证实）

`v->pcm_storage = ci->blocksizes[1]`（`block.c:245`）按**完整**长 blocksize
分配，半速只使用其前半部分索引，因此半速不会引入新的缓冲越界；这也是
ASan 全路径干净的结构性原因之一。

---

## 4. 定位（seek / tell）在半速下的语义

### 4.1 定位参数与返回位置都是"原始流样本"，不是输出样本

- `ov_pcm_seek(vf, pos)` 的 `pos` 是原速率 granule 位置。其内部
  `ov_pcm_seek_lap`/普通 seek 的丢弃循环（`vorbisfile.c:1763-1778`）：
  ```c
  int hs=vorbis_synthesis_halfrate_p(vf->vi);
  while(vf->pcm_offset < ((pos>>hs)<<hs)){
      ogg_int64_t target=(pos-vf->pcm_offset)>>hs;   /* 换算成输出样本数 */
      ...
      vf->pcm_offset += samples<<hs;                  /* 仍按原刻度推进 */
  }
  ```
  注意 `(pos>>hs)<<hs` 把目标**对齐到偶数**（hs=1 时）。实测：半速下
  `ov_pcm_seek(1001)` 后 `pcm_tell=1000`，`seek(1003)` 后 `pcm_tell=1002`；
  即半速无法停在奇数原始位置，统一向下对齐到偶数 granule。
- `ov_pcm_tell()` 直接返回 `vf->pcm_offset`（`vorbisfile.c:1851-1854`），
  单位恒为原始流样本，**与 `hs` 无关**。
- `ov_time_tell()`（`vorbisfile.c:1857-1876`）用
  `(pcm_offset - pcm_total_in_link)/vi[link].rate` 换算秒，分母是原速率，
  所以半速下时间显示仍然正确（实测 `ov_time_total` 在两种模式下都返回
  1.0000 s）。
- `ov_pcm_total/ov_time_total`（`:1195-1226`）读取 `pcmlengths`，
  **完全不感知 halfrate**，半速不改变总时长/总样本数的报告值。

### 4.2 实测定位行为

| 调用 | 模式 | 结果 |
|---|---|---|
| `ov_pcm_seek(1000)` | 半速 | tell=1000，随后读 32 个输出样本后 tell=1064（+64 原样本） |
| `ov_pcm_seek(1001)` | 半速 | tell=1000（向下偶对齐） |
| `ov_pcm_seek(1003)` | 半速 | tell=1002 |
| `ov_pcm_seek(1001)` | 全速率 | tell=1001（可停奇数位） |
| `ov_pcm_seek_lap(2000)`，8kHz 半速 | 半速 | tell=2000，首读 24 样本后 tell=2048 |

结论：**定位 API 的坐标域是原始流时间线；半速只改变"为到达该位置需要
消费多少个输出样本"，不改变位置本身的含义**。奇数目标的偶对齐是半速
2:1 栅格的必然结果（源码可证）。

---

## 5. 交叉淡化（crosslap / seek-lap）与半速同时出现

### 5.1 淡化长度、窗口、lap 缓冲都随各自的 `hs` 缩放

`ov_crosslap(vf1,vf2)`（`vorbisfile.c:2273-2333`）：

```c
hs1=ov_halfrate_p(vf1); hs2=ov_halfrate_p(vf2);        /* :2296-2297 */
n1=vorbis_info_blocksize(vi1,0)>>(1+hs1);              /* :2301 */
n2=vorbis_info_blocksize(vi2,0)>>(1+hs2);              /* :2302 */
w1=vorbis_window(&vf1->vd,0); w2=vorbis_window(...);   /* :2303-2304 */
...
_ov_splice(pcm,lappcm,n1,n2,ch1,vi2->channels,w1,w2); /* :2328 */
```

`_ov_splice()`（`:2157-2190`）取 `n=min(n1,n2)`、`w=` 较短一侧的窗口，
用平方余弦权重 `d = d·w² + s·(1-w²)` 做等功率交叉淡化；新流多出的声道
从静音加窗，旧流多出的声道被忽略。淡化只发生在新流开头的
`min(n1,n2)` 个**输出**样本内，不增删任何样本（官方文档
`doc/vorbisfile/crosslap.html` 亦明确"will not introduce any samples into
or remove any samples from the decoded audio"）。

`_ov_getlap()`（`:2226-2262`）从旧流取 `lapsize` 个输出样本；不足时调用
`vorbis_synthesis_lapout()` 暴露编码器后置外推（post-extrapolation）的
隐式缓冲。`lapout()`（`block.c:973-1046`）内部同样用
`n/n0/n1 = blocksizes[...]>>(hs+1)`（`:978-980`）做半速缩放，因此 EOF
处的合成 lap 数据在半速下也是半长、同尺度的。

### 5.2 实测

- 同速率 44.1k→32k、两流都半速：`ov_crosslap` 返回 0，新流输出 16000
  （=32000/2），末 tell=32000，样本数与无 crosslap 完全一致。
- 8k→11.025k（bs0 均为 512）两流半速：返回 0，新流输出 5513，末
  tell=11026（同样的奇数 granule 现象）。
- 淡化影响区间：全速率 0..125、半速 0..62，区间外两路逐样本相同。
- ASan 下所有 crosslap / `*_seek_lap` 组合（含 8k 小 blocksize）均无
  内存错误报告。

### 5.3 `vorbis_window()` 访问器的守卫口径与未初始化句柄陷阱（源码可证）

`block.c:1048-1056` 的公开访问器：

```c
const float *vorbis_window(vorbis_dsp_state *v,int W){
  ...
  if(b->window[W]-1<0)return NULL;          /* 守卫用 -1，不是 -hs */
  return _vorbis_window_get(b->window[W]-hs);
}
```

这里有两处需要分清的事实：

1. **守卫与真实索引的口径不一致，但在受支持配置下不会出错。**
   守卫判断 `window[W]-1<0`，而真实表索引是 `window[W]-hs`。
   `window[W]` 在 `_vds_shared_init`（`block.c:207-208`）被赋为
   `ilog(blocksize)-7`。受支持的最小 blocksize 为 64
   （`_vds_shared_init` 在 `blocksizes[0]<64` 时直接返回失败，
   `block.c:176-181`），因此 `window[W]>=0` 恒成立；半速下
   `window[W]-hs>=-1`，而等于 -1 仅当 blocksize=64，但该情形已被
   `vorbis_synthesis_halfrate` 的 `blocksizes[0]<=64` 守卫
   （`synthesis.c:171`）拒绝。结合第 2.3 节的表长恒等式
   （表 `k` 长 `32<<k == blocksize>>(hs+1)`），真实索引在所有受支持
   组合下都指向长度恰好足够的窗口表，**无越界**（ASan 全路径复核通过）。

2. **对未初始化 dsp 调用该访问器会崩溃，且与 halfrate 无关。**
   实测对一个内存被清零、尚未 `vorbis_synthesis_init` 的
   `vorbis_dsp_state` 调用 `vorbis_window(&vd,0)`，无论全速率还是半速都
   在 `block.c:1054` 解引用空的 `backend_state`（`b`）而 SEGV
   （ASan 报告地址 0x8）。这是"对未初始化句柄调用 API"的通用误用，
   不是半速引入的缺陷。

`ov_crosslap`/seek-lap 在取窗前已通过 `_ov_initset/_ov_initprime`
（`vorbisfile.c:2193-2221`）确保解码器 INITSET 且已 primed，此时
`b->window[W]` 已正确赋值。在一份加了打印的副本构建中实测 crosslap 时
`window[0]=2, hs=1, guard=0`，返回有效指针。
**结论：库内所有 crosslap/seek 路径在半速下安全；唯一的崩溃来自对未
初始化 dsp 的 API 误用，全速率下同样存在。**

---

## 6. 模式切换对"已有调用结果"的影响

### 6.1 `ov_halfrate()` 的运行时切换逻辑（源码可证）

`ov_halfrate(vf,flag)`（`vorbisfile.c:1027-1050`）：

1. 若解码器已超过 STREAMSET（即已建立解码机），先
   `vorbis_dsp_clear` + `vorbis_block_clear` 并把状态降回 STREAMSET
   （`:1033-1035`）——注释说明这是为了按新 `hs` 重建 MDCT 查找表；
2. 若 `pcm_offset>=0`，保存当前位置后调用 `ov_pcm_seek(vf,pos)`
   （`:1036-1040`）重新定位，使切换后从**同一原始流位置**继续；
3. 对**每个 link** 调 `vorbis_synthesis_halfrate(vi+i,flag)`；任一 link
   失败（返回 -1）且本次是开启操作，则递归 `ov_halfrate(vf,0)` 回滚
   全部 link，并返回 `OV_EINVAL`（`:1043-1047`）。

### 6.2 实测：切换不破坏播放时间连续性

44.1kHz 文件，全速率读 2112 输出样本（tell=2112）→ 切半速 → 读 1024 个
半速输出样本后 tell=4160（增量 2048 原样本 = 1024×2）→ 切回全速率 →
再读 1024 样本后 tell=5184（增量 1024）。8kHz 文件结果同构。
即：**切换前后 `pcm_tell` 在原始流刻度上严格单调连续，密度变化只体现在
"每个输出样本代表 1 个还是 2 个原始样本"**。切换会清空并重做交叠缓冲
（重建 dsp + seek），因此切换点附近的交叠上下文被重置，但位置不丢。

### 6.3 多 link 一致性（源码可证 + 实测）

- 源码注释（`vorbisfile.c:1764-1765`）："halfrate could be set
  differently in each link, but vorbisfile enforces all links are set or
  unset"。`ov_halfrate` 对全部 link 统一设置，任一回滚全部回滚。
- 实测双 link 链式文件（44.1k/32k，总 38050 原样本）：全速率输出
  38050、半速输出 19025=38050/2，跨 link 时 `section` 正确从 0 变 1，
  末 tell 均为 38050。跨 link 边界由 `_fetch_and_process_packet`
  （`:788-810`）在 BOS 页处 `_decode_clear` 并按新 link 的 vi 重建，
  新 link 的 `hs` 已被统一设置。

---

## 7. 不支持/不受支持的组合及其后果

| 组合 | 源码行为 | 对已有调用结果的影响 |
|---|---|---|
| `blocksizes[0] <= 64` 开半速 | `vorbis_synthesis_halfrate` 返回 -1，标志不变（`synthesis.c:171`） | 该 link 维持全速率；经 `ov_halfrate` 时整文件回滚为全速率并返回 `OV_EINVAL`，已解码数据不受影响 |
| 半速下 seek 到奇数 granule | `(pos>>1)<<1` 偶对齐（`vorbisfile.c:1767`） | 实际停在前一个偶数位置；不报错，应用若假设精确定位会偏差 ≤1 个原样本 |
| 奇数总样本数的流在半速下解码到 EOF | 输出样本数被 EOF 裁剪正确限制，但 `pcm_offset` 按 `samples<<1` 记账，奇数 total 时 `ceil(total/2)·2 = total+1`（实测 11025→11026） | 仅 `ov_pcm_tell` 报告值多 1；输出样本数 5513 本身正确。实测插桩显示 blockin 的 EOF 裁剪分支（`block.c:882-895`）**确实触发**并把输出裁到 5513，但该裁剪只调整 `pcm_current/pcm_returned`，不回改 vorbisfile 层按输出样本数左移累加的 `pcm_offset`，故 +1 保留在 tell 上 |
| 两个 `OggVorbis_File` 以**不同** `hs` 做 `ov_crosslap` | 源码不拒绝，返回 0；`n1/n2`、`w1/w2` 各自按 `hs1/hs2` 计算（`:2296-2304`），`_ov_splice` 取 `min` | 可运行、无崩溃（实测+ASan），但两侧输出样本代表的原始时长不同（一侧每个输出=1 原样本，另一侧=2），等长 `n` 的淡化在物理时间上不等长，衔接的时间对齐语义被破坏，**不保证听感正确** |
| 不同采样率/声道数的 crosslap | 源码允许；采样率不做隐式重采样，声道按 5.1 规则取舍（文档 `crosslap.html` 与 `_ov_splice`） | 样本数不增删，但时间一致性需应用自行保证；与 `hs` 正交 |
| 对未初始化 dsp 调 `vorbis_window()` | `backend_state` 为空，`block.c:1054` 解引用空指针 SEGV（全/半速皆然） | 属 API 误用；库内 crosslap/seek 路径因先 prime 而不可达。半速守卫口径（`-1` vs `-hs`）虽与真实索引不同，但受 bs0≤64 拒绝与表长恒等式保护，受支持组合下不越界 |
| 非 seekable 流上切换半速 | `ov_halfrate` 里 `pcm_offset<0` 时不 seek（`:1036-1040`），直接重建解码机 | 位置计数被丢弃（`pcm_offset=-1`），后续靠页 granulepos 重新锚定；无法保证回到切换前位置 |

---

## 8. 保持播放时间连续的成立条件

综合源码，时间连续（原始流时间线不跳、不重、不错位）的**充要/前置条件**
可归纳为：

1. **坐标统一在原始流 granule 域做定位与 tell**。应用必须把
   `ov_pcm_tell/ov_pcm_seek/ov_pcm_total` 当作原速率样本坐标，把
   `ov_read*` 的返回值当作输出域样本，并用 `<<ov_halfrate_p` 在两者间
   换算；切勿把半速输出样本数直接当播放位置。
2. **seek 目标按 2^hs 栅格对齐**。半速下只申请偶数 granule，或显式接受
   `(pos>>1)<<1` 的向下对齐，避免把 ≤1 原样本的对齐误差误判为不连续。
3. **模式切换只通过 `ov_halfrate()` 进行，且流可 seek**。该函数负责
   清机→重建 MDCT→按原 `pcm_offset` 重定位；非 seekable 流无法重定位，
   不满足连续条件。
4. **crosslap / seek-lap 的两侧使用相同 `hs`**（并尽量相同采样率）。
   源码不强制，但只有 `hs1==hs2` 时 `n1` 与 `n2` 在物理时间上等长，
   `_ov_splice` 的等功率淡化才对齐同一时间栅格。
5. **小 blocksize 流（bs0≤64）不要申请半速**，并处理 `OV_EINVAL`/回滚。
6. **应用侧以原速率时钟播放**：半速输出应按 `rate/2` 的输出采样率播放，
   才能还原与 `ov_time_tell` 一致的墙钟时长；输出密度减半但每个输出样本
   承载的原始时间翻倍，二者由第 1 节恒等式绑定。
7. **容忍 EOF 奇数 granule 的 tell 越界**（第 7 节 +1 现象）：以
   `ov_pcm_total` 为播放长度上界做钳制，而不是无条件信任末尾
   `ov_pcm_tell`。

只要 1–6 成立，源码可证 `pcm_offset` 在解码、seek、crosslap、模式切换、
跨 link 各操作之间保持同一原始时间线的单调连续；条件 7 处理已知的整数
边界例外。

---

## 9. 证据分级：源码可证实 vs 需听感/解码输出补证

### 9.1 仅凭源码即可证实的行为

- 半速标志的存放、守卫阈值（bs0≤64 拒绝）与回滚逻辑。
- MDCT 按 `blocksize>>hs` 初始化、块向量仍按完整 blocksize 分配。
- 所有 overlap/lap 长度按 `>>(hs+1)`、窗口索引按 `-hs` 缩放，且窗口表
  长度与循环下标的尺度恒等关系（无越界的静态结论，ASan 动态复核通过）。
- `granulepos/sample_count` 用完整 blocksize 记账、仅在输出缓冲裁剪处
  `>>hs`；`pcm_offset` 用 `samples<<hs` 推进。
- seek/tell/total/time 各 API 的坐标域不随 halfrate 改变；seek 的偶对齐。
- crosslap/seek-lap 的淡化长度、平方余弦权重、不增删样本、声道取舍规则。
- 运行时切换的清机/重建/重定位流程与多 link 统一设置。
- `vorbis_window()` 守卫口径（`-1`）与真实索引（`-hs`）的差异，及其在
  bs0≤64 守卫与窗口表长度恒等式保护下不可达的结论；未初始化 dsp 调用
  该访问器的空指针崩溃属与 halfrate 无关的 API 误用。

### 9.2 需要实际解码输出/听感才能补证的部分

源码只规定了**算术与索引**，下列性质无法单靠读码断言，已用本仓库构建的
实测给出客观证据，但**主观听感**仍需人工/音频管线确认：

- **降采样的频域质量与抗混叠**：实测（44.1kHz、含 0.47·rate 近 Nyquist
  分量与周期脉冲的信号）半速输出在半速 Nyquist 以上的能量从 63579 降到
  33.4（约 −33 dB），带内能量 107490→106728 基本保留，未见明显镜像谱线；
  说明半速 IMDCT+窗口路径等效为带抗混叠的低通 2:1 降采样。**听感上是否
  透明、顶部 octave 的软化是否可闻，需试听确认**（官方文档亦只承诺
  "at the expense of decoded audio quality"）。
- **时域波形对应关系**：实测半速样本与全速率相邻两样本的 2 抽头平均
  高度吻合（稳态正弦 RMS 差 1.1e-4，相关系数 0.944、零样本偏移），但
  并非逐点等于任一全速率样本；它是 MDCT 域重建的结果而非简单均值，
  瞬态段差异更大（瞬态信号 RMS 差 0.065，RMS 能量比 0.62）。
- **crosslap 的 clicks/pops 抑制效果**：实测淡化严格限制在半短块窗口内、
  区间外零差异，权重连续；但"是否完全听不到接缝"属听感判断，源码与
  客观测量只能保证无样本数变化、无突变权重。
- **EOF 外推 lap 的自然度**：`_ov_getlap` 在数据不足时使用编码器后置
  外推缓冲，文档自称"usually completely unnoticeable"，自然度需听感。
- **11025 类奇数 granule 的 +1 tell**：数值现象与机制已实测确认
  （blockin EOF 裁剪正常触发、输出样本数正确，+1 仅来自 vorbisfile 的
  `samples<<1` 记账），但其是否在具体应用的进度条/时长显示上产生可见
  跳变，取决于应用如何钳制与取整。

---

## 10. 关键源码位置索引

| 主题 | 位置 |
|---|---|
| 半速开关/守卫/查询 | `lib/synthesis.c:166-179` |
| 半速标志定义 | `lib/codec_internal.h:130` |
| 半速 MDCT 初始化 | `lib/block.c:182,197-198` |
| 块半窗长/overlap 缩放 | `lib/block.c:735,755-815` |
| 块产出量与 granule 记账 | `lib/block.c:840-842,858-937` |
| EOF/起始裁剪的 `>>hs` | `lib/block.c:892-898,920-930` |
| lapout 半速缩放 | `lib/block.c:973-1046` |
| `vorbis_window()` 守卫口径与未初始化陷阱 | `lib/block.c:1048-1056` |
| 窗口表与尺度 | `lib/window.c:2087-2100` |
| 逆 MDCT 作用于完整块向量 | `lib/mapping0.c:706,792-795` |
| `ov_halfrate` 清机/重定位/回滚 | `lib/vorbisfile.c:1027-1055` |
| 读路径 `pcm_offset += samples<<hs` | `vorbisfile.c:725,2096-2097,2134-2138` |
| seek 偶对齐与丢弃循环 | `vorbisfile.c:1763-1778` |
| tell/total/time 坐标域 | `vorbisfile.c:1195-1226,1851-1876` |
| crosslap 长度/窗口/拼接 | `vorbisfile.c:2157-2190,2273-2333` |
| seek-lap 系列 | `vorbisfile.c:2335-2487` |
| 官方 crosslap 不增删样本说明 | `doc/vorbisfile/crosslap.html` |
| 官方 halfrate 语义说明 | `doc/libvorbis/vorbis_synthesis_halfrate.html` |

---

## 附：实测可复现性说明

实验在仓库外的独立目录完成（未污染本仓库）：用 libogg 1.3.5 静态依赖
out-of-tree 构建本仓库源码（普通构建 + ASan 构建各一套），自写编码器
生成 8k/11.025k/16k/32k/44.1k 单链路与双 link 链式文件，分别在全速率/
半速下跑解码、seek、seek-lap、crosslap、运行时切换、守卫阈值、频谱与
时域对齐测量。ASan 在所有**库支持的常规路径**上零报告；唯一崩溃来自
对未初始化 dsp 调用 `vorbis_window()` 的显式 API 误用，与第 5.3 节源码
分析一致。
