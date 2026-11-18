# Vorbis/Ogg 丢页（容器空洞）后的解码恢复责任分析

> 代码基线：libvorbis 1.3.7（本仓库，`lib/`）与 libogg 1.3.5（位于
> `…/tech-8285-20261116-21-40/native/src/libogg-1.3.5/src/framing.c`）。
> 所有结论尽量锚定到具体文件与行号；凡源码不能直接证明、需要结合 Ogg/Vorbis
> 规范或运行时行为外推的内容，均显式标注 **【推断】**。
>
> 核心结论先行：**后续收到一个 CRC 合法、自洽的完整音频包，只能证明“从这个包
> 开始可以继续产出 PCM”，不能证明空洞期间丢失的原始样本已知或被恢复。** 库不做
> 丢包隐藏（PLC），不会把未知音频补写成原样本；绝对播放位置只有在再次遇到带
> granulepos 的页/包（或调用方主动 seek）时才能重新锚定。

---

## 1. 涉及的两层状态与责任边界总览

| 层 | 组件 | 维护的状态 | 丢页时的职责 |
|---|---|---|---|
| 容器/成帧层 | libogg `ogg_sync_state`（同步器）、`ogg_stream_state`（逻辑流） | 页序列号 `pageno`、包序列号 `packetno`、段表 `lacing_vals`、半组包缓冲 `body_data` | 检测页序号断档与 CRC 失败，丢弃残缺半组包，插入一个 **hole 标记（`0x400`）**；不理解“音频” |
| 编解码层 | libvorbis `vorbis_dsp_state vd` / `vorbis_block vb`，由 libvorbisfile 的 `OggVorbis_File` 持有 | 重叠相加缓冲 `v->pcm`、`v->sequence`、`v->granulepos`、`sample_count`、窗口状态 `lW/W` | 对乱序包**主动作废内部计数**；继续解码后续完整包，但与空洞前的旧缓冲做重叠相加；无 PLC |
| 便利层 | libvorbisfile（`vorbisfile.c`） | `vf->pcm_offset`（对外播放位置）、`vf->os`、`vf->vd/vb`、`ready_state` | 把 hole 翻译成 **`OV_HOLE`** 返回给调用方；在后续带 granulepos 的包上**重新锚定** `pcm_offset`；seek 时做完整复位 |
| 调用方 | 应用 | 自己的时间轴/呈现状态 | 处理 `OV_HOLE`：静音/淡入淡出/插补、UI 不连续提示、或 seek 重建；**不得假设库补回了样本** |

关键事实：`OV_HOLE` 是一个**一次性的信号**，不是一种持续状态。容器层插入的
hole 标记被 `ogg_stream_packetout()` 消费一次后即消失，解码随即在后续完整包上
继续。

---

## 2. 容器空洞是如何被检测出来的（libogg 证据链）

### 2.1 两个检测点

**(a) 同步层 CRC/捕获失败 —— `ogg_sync_pageseek()`（framing.c:641-725）**

- 页头捕获模式 `OggS` 不匹配，或整页缓冲齐后 CRC 校验失败（framing.c:684-693），
  跳到 `sync_fail`：从下一个 `'O'` 字节重新找捕获，返回**负数**（跳过的字节数）。
- libvorbisfile 的 `_get_next_page()` 对负返回只做 `vf->offset-=more` 记账
  （vorbisfile.c:119-121），然后继续找下一页。**坏页整体不进入 `ogg_stream_pagein`**。

**(b) 流层页序号断档 —— `ogg_stream_pagein()`（framing.c:771-898）**

```c
/* framing.c:822-835 */
if(pageno!=os->pageno){
  /* unroll previous partial packet (if any) */
  for(i=os->lacing_packet;i<os->lacing_fill;i++)
    os->body_fill-=os->lacing_vals[i]&0xff;
  os->lacing_fill=os->lacing_packet;
  /* make a note of dropped data in segment table */
  if(os->pageno!=-1){
    os->lacing_vals[os->lacing_fill++]=0x400;
    os->lacing_packet++;
  }
}
...
os->pageno=pageno+1;   /* framing.c:895：以新页的序号为准续接 */
```

- 只要实际页号与期望页号 `os->pageno` 不一致（丢页、跳过坏页、重复页都算），
  且这不是首次喂页（`pageno!=-1`），就在段表写入一个 `0x400` 占位段——即 hole 标记。
- 此前正在组的**跨页半包被回滚**（`body_fill` 减去未完成段长度），其数据作废。

**hole 的消费 —— `_packetout()`（framing.c:940-956）**

```c
if(os->lacing_vals[ptr]&0x400){
  /* we need to tell the codec there's a gap; it might need to
     handle previous packet dependencies. */
  os->lacing_returned++;
  os->packetno++;      /* 包序号也跳过一个 */
  return(-1);
}
```

返回 -1，且只返回一次（标记随 `lacing_returned++` 被消费）。注意 `packetno++`：
**空洞后第一个真实包的 `packetno` 比空洞前最后一个包大 2 以上**，这是编解码层
判定“乱序”的输入。

### 2.2 跨页包（continued packet）在空洞中的具体命运

Ogg 大包可跨多页：前一页以 `0xFF` 段结束、后一页置 `continued` 标志。
`ogg_stream_pagein()` 对“孤儿续页”的处理在 framing.c:839-854：

```c
if(continued){
  if(os->lacing_fill<1 ||
     (os->lacing_vals[os->lacing_fill-1]&0xff)<255 ||
     os->lacing_vals[os->lacing_fill-1]==0x400){
    bos=0;
    for(;segptr<segments;segptr++){   /* 跳过属于残缺包的段 */
      int val=header[27+segptr];
      body+=val; bodysize-=val;
      if(val<255){ segptr++; break; }
    }
  }
}
```

可观察结果分三种情形：

1. **丢失的是中间页，且它承载某跨页包的中段/尾段**：空洞前缓存的该包前半段在
   断档处理时被回滚（framing.c:826-828）；空洞后第一个续页因为“前面没有
   `0xFF` 未完成段/前面正好是 hole 标记”，其属于残缺包的段被**跳过丢弃**
   （framing.c:844-852）。该跨页包整体不可解码，且不会以错误包形式提交给
   Vorbis——它在容器层就消失了，只剩一个 hole 标记。
2. **空洞后第一页不是 continued 页（新包边界正常开始）**：该页上的完整包照常
   入段表，可被 `packetout` 取出并解码。
3. **granulepos 的连带影响**：libogg 只把页 granulepos 挂到该页**最后一个完整
   包**上（framing.c:882-885，`saved` 只在遇到 `<255` 的收尾段时更新）。若一页上
   没有任何完整包（只有残缺续段），该页不产生带 granulepos 的包，位置锚点也要
   再等后面的页。

> 【推断】情形 1 中被丢弃的字节数/包大小，库不向调用方报告具体数值；调用方只能
> 看到一次 `OV_HOLE`，无法从 API 得知丢了多少字节或多少样本。源码中没有任何保存
> 丢失长度的字段，此点可由 `0x400` 标记不携带长度信息（framing.c:832）直接支持。

---

## 3. libvorbisfile 如何把容器空洞上抛为 OV_HOLE

### 3.1 读路径上的唯一上抛点

`_fetch_and_process_packet()`（vorbisfile.c:678-871）：

```c
/* vorbisfile.c:701-705 */
int result=ogg_stream_packetout(&vf->os,op_ptr);
...
if(result==-1)return(OV_HOLE); /* hole in the data. */
```

`ov_read` / `ov_read_float` / `ov_read_filter` 都在取包循环里直接透传该返回值
（vorbisfile.c:1985-1989、2147-2149）：

```c
int ret=_fetch_and_process_packet(vf,NULL,1,1);
if(ret==OV_EOF) return(0);
if(ret<=0) return(ret);     /* OV_HOLE(-3) 原样返回，本次不产出任何 PCM */
```

文档侧的定义（`doc/vorbis-errors.txt:14-17`、`doc/vorbisfile/ov_read.html`）：
`OV_HOLE` 表示“数据中断：页间垃圾、失步后重新捕获、或坏页”三者之一。

### 3.2 时序细节：先排空已解码 PCM，再报 hole

`ov_read_filter()` 的循环（vorbisfile.c:1977-1992）先调用
`vorbis_synthesis_pcmout()`，**只有当解码器里已没有可返回样本时**才去取包、才可能
拿到 `OV_HOLE`。因此可观察顺序是：空洞前已解码的样本先全部交付 → 下一次读调用
返回一次 `OV_HOLE`（无数据）→ 再下一次调用才开始交付空洞后的新样本。
`ov_read_float()` 行为相同（vorbisfile.c:2129-2152）。

### 3.3 库内部对 OV_HOLE 的“容忍”与对外的“严格”不一致

- 跨链接拼接/交叉淡化辅助路径 `_ov_initset()`、`_ov_initprime()` 显式吞掉
  `OV_HOLE` 继续取包（vorbisfile.c:2199、2217：`if(ret<0 && ret!=OV_HOLE)return`）。
- 打开可寻址文件时的 `_initial_pcmoffset()` 也忽略 hole（vorbisfile.c:442：
  `if(result>0){ /* ignore holes */`），只累计块大小来推算首个音频页的 PCM 偏移。
- 但**头包阶段不容忍**：`_fetch_headers()` 中 `packetout` 返回 -1 直接判定
  `OV_EBADHEADER` 失败（vorbisfile.c:372-375）。即头部页缺失会导致打开失败，
  这与音频数据区的 hole 是两条不同的错误路径。
- 精确 seek 的跟踪循环同样容忍 `OV_HOLE`（vorbisfile.c:1732：
  `if(ret<0 && ret!=OV_HOLE)break;`）。

---

## 4. 解码历史重建：Vorbis 层实际做了什么、没做什么

### 4.1 乱序包使内部样本/granule 计数主动作废

`vorbis_synthesis()` 把 Ogg 包号复制进块结构（synthesis.c:75：
`vb->sequence=op->packetno`）。`vorbis_synthesis_blockin()` 入口检查
（block.c:745-749）：

```c
if((v->sequence==-1)||
   (v->sequence+1 != vb->sequence)){
  v->granulepos=-1; /* out of sequence; lose count */
  b->sample_count=-1;
}
v->sequence=vb->sequence;
```

由于 §2.1 中 hole 消费时 `packetno++`，空洞后第一个真实包必然不满足
`v->sequence+1 == vb->sequence`，于是：

- 解码器**内部的 granulepos 跟踪与 sample_count 同时作废为 -1**。这是源码明确
  注释的“lose count”，即解码器承认自己失去了连续计数基准。
- 恢复条件在 block.c:858-867、864-868：只有当后续某个包**自带有效
  granulepos**（来自带标记的页）时，`v->granulepos` 才被重新赋值；在此之前
  `sample_count` 每次 blockin 只是从 0 重新累加（block.c:858-862），属于
  “相对计数”，不代表绝对位置。

block.c:847-856 的注释还明确承认该机制在 seek/hole 后落到末页时**可能被误导**
（“This is not foolproof! … confused if we begin decoding at the last page after
a seek or hole”），并说明 vorbisfile 因此尽量保证按顺序读到最后两个带标记页。

### 4.2 重叠相加缓冲不复位 —— 空洞边界的音频是“接”出来的，不是“重建”的

- `vorbis_synthesis_blockin()` 的 overlap/add 段（block.c:779-824）无条件使用
  `v->pcm` 中**上一块**的数据与新块做窗口叠加。
- 全仓搜索表明 `vorbis_synthesis_restart()` 只在 `vorbis_synthesis_init()`
  （block.c:723）和三个 **seek** 位置调用（vorbisfile.c:1257、1580、1622）；
  **`OV_HOLE` 的流式读路径不调用任何 restart/reset 来清空 `v->pcm`**。
- `v->pcm` 在初始化时是 calloc 清零的（block.c:245-252），但空洞发生时里面保留
  的是空洞前最后一块的真实样本。

**可观察结果（源码可证部分）**：空洞后第一个可解码块的左半段，会与空洞前的
陈旧 PCM 做一次重叠相加，产生一个边界拼接块；之后的块恢复正常块间叠加。
**【推断】听感上通常表现为空洞后第一个块附近的一次咔哒/毛刺或轻微调制**——
叠加运算本身可证，但具体听感属于信号层面的推断，库内没有任何代码评估或抑制它。

### 4.3 库不做丢包隐藏（PLC）

- 合成侧没有任何针对缺失包的插值/外推：`_preextrapolate_helper` 等外推函数
  （block.c:426 起）只在**分析（编码）路径** `vorbis_analysis_wrote` 中调用；
  `mapping0_inverse()`（mapping0.c:698-799）只处理实际提交的包。
- 丢失的包不产生 PCM，`vorbis_synthesis_pcmout()` 返回的样本流里**没有与空洞
  等长的占位**。时间轴上这一段就是缺失的，不会被静音或任何合成样本填充。
- `vorbis_synthesis_lapout()`（block.c:973 起）能暴露“合成/后外推”缓冲，但它
  服务于 `ov_crosslap`/`*_seek_lap` 这类**主动 seek 交叉淡化** API
  （vorbisfile.c:2226-2262、2335-2365），**与被动遇到 OV_HOLE 的恢复路径无关**，
  读路径不会自动调用它补洞。

### 4.4 自洽但内容损坏的包：另一条“静默”路径

若页 CRC 合法、包完整，但包内载荷违反 Vorbis 范围约束，`vorbis_synthesis()`
返回 `OV_EBADPACKET`/`OV_ENOTAUDIO`（synthesis.c:42-56）。
`_fetch_and_process_packet()` 对非零返回只是不做 blockin、继续 while 循环取下一
包（vorbisfile.c:709-766 的 `if(!vorbis_synthesis(...)){...}` 结构）：

- **不向上返回错误、不计 samptrack、不更新 pcm_offset**，该包被静默跳过。
- 这类数据丢失**不伴随 `OV_HOLE`**（容器层没看到断档）。
- 【推断】调用方无法通过返回码区分“安静跳过一个坏包”与正常解码，只能从
  `ov_pcm_tell()` 的停顿/后续 granulepos 锚定时的跳变间接察觉；源码中无专门
  计数器暴露此事。

---

## 5. 播放位置（pcm_offset）何时可重新确定

`OggVorbis_File.pcm_offset` 是对外的“下一个待读样本”位置
（`ov_pcm_tell()` 直接返回它，vorbisfile.c:1851-1854）。它有两套维护机制：

### 5.1 增量机制（空洞期间不可信）

- 每读出 PCM：`vf->pcm_offset += samples<<hs`（vorbisfile.c:2097、2138）。
- 每成功 blockin：`vf->samptrack += pcmout<<hs`（vorbisfile.c:725）。
- 空洞期间没有包被 blockin、没有 PCM 读出，**`pcm_offset` 停在空洞前最后一个
  已交付样本处，既不前进也不反映丢失样本数**。此时 `ov_pcm_tell()`/
  `ov_time_tell()`（vorbisfile.c:1857-1876，基于 pcm_offset 换算）给出的是
  **陈旧位置**。

### 5.2 绝对重锚机制（恢复位置的唯一正常途径）

`_fetch_and_process_packet()` 在处理带 granulepos 的包时
（vorbisfile.c:730-760）：

```c
if(granulepos!=-1 && !op_ptr->e_o_s){
  ...
  samples=(vorbis_synthesis_pcmout(&vf->vd,NULL)<<hs);
  granulepos-=samples;                 /* 由“块末位置”反推块首位置 */
  ...
  vf->pcm_offset=granulepos;           /* 无条件覆盖，绝对重锚 */
}
```

要点：

1. **重锚只认容器里的 granulepos**。Vorbis 规定页 granulepos 标记该页最后一个
   完整包结束处的 PCM 样本号；libogg 把它挂在最后一个完整包上（framing.c:882-885）。
2. 重锚是**覆盖式赋值**，因此 `ov_pcm_tell()` 在空洞后第一个带标记页被处理时会
   出现一次**不连续跳变**，跳变差值即容器声称的丢失/跨越样本数（含编码器延迟
   等既有偏移调整）。这是“位置已知”与“位置未知”的分界事件。
3. `!op_ptr->e_o_s` 限制：EOS 页上的 granulepos 不参与这套重锚。源码注释
   （vorbisfile.c:739-745）解释：末页可能是部分帧，缺少前序参照点时无法据此
   反推块首；这也呼应 block.c:847-856 的“not foolproof”说明。
4. 重锚成功**只意味着位置数字重新可信，不意味着空洞期样本被恢复**。锚点之间
   缺失的 PCM 永远缺失。

### 5.3 可寻址流 vs 不可寻址流

| 场景 | 位置恢复手段 | 源码保证 |
|---|---|---|
| 可寻址文件（`seekable=1`） | 被动等 granulepos 重锚；或调用方主动 `ov_pcm_seek*`/`ov_raw_seek` | seek 路径会 `ogg_stream_reset_serialno` + `vorbis_synthesis_restart` 完整复位（vorbisfile.c:1255-1257、1583、1625），并通过二分查找带 granulepos 的页重建 `pcm_offset`（ov_pcm_seek_page:1406-1680；ov_raw_seek:1296-1392） |
| 不可寻址流（管道等） | **只能**被动等下一个带 granulepos 的页 | 没有 seek 回调；若残余流一直不出现带标记页，`pcm_offset` 只能靠增量计数，**绝对位置无法重新确定**【推断：源码未提供任何其他锚定途径】 |
| 打开阶段（可寻址） | `_initial_pcmoffset()` 扫描首个带 granulepos 的音频页推算起始偏移，hole 被忽略（vorbisfile.c:424-465） | 起始偏移损坏时保守钳到 0（vorbisfile.c:462） |

seek 路径的一个细节：`ov_raw_seek()` 特意用本地 `work_os` 扫描、并先
`ogg_stream_reset()` 以“消除并非从流开头开始而产生的伪 OV_HOLE”
（vorbisfile.c:1291-1294）。这说明库作者把“任意位置切入导致的页序号不连续”与
“真实数据空洞”视为同一种 libogg 现象，靠 reset 抑制前者。

---

## 6. 两类事件的可观察结果对照

### 6.1 容器空洞（页缺失 / CRC 坏页 / 失步重捕）

按发生顺序，调用方可观察到：

1. `ov_read*()` 先排空空洞前已解码的剩余 PCM（vorbisfile.c:1978-1981）。
2. 下一次读返回 **`OV_HOLE`（-3）一次**，本次无音频数据；`OV_HOLE` 不重复，
   因为 hole 标记是一次性的（framing.c:950-956）。
3. `ov_pcm_tell()` 在重锚前持续返回空洞前的陈旧值。
4. 空洞后第一个完整音频包被正常解码，但其首块与陈旧缓冲做重叠相加
   （block.c:779-824，无 restart）——【推断】边界处可能有一次音频毛刺。
5. 解码器内部 `granulepos/sample_count` 曾被置 -1（block.c:745-749），直到带
   granulepos 的包到来才恢复绝对计数。
6. 处理到第一个带 granulepos 的完整包时，`pcm_offset` 被覆盖式重锚
   （vorbisfile.c:759），`ov_pcm_tell()`/`ov_time_tell()` 出现一次跳变。
7. 丢失样本数 = 重锚位置 − 空洞前最后位置 − 实际交付样本数；库不直接给出该值
   【推断：可由调用方在重锚前后自行差分得到，但库 API 不报告】。
8. 坏页若发生在三个 Vorbis 头包区域，打开直接失败 `OV_EBADHEADER`
   （vorbisfile.c:372-375），不会进入音频 hole 流程。

### 6.2 跨页包被截断（§2.2 情形 1）

1. 残缺包的前段在页序号断档时被回滚（framing.c:826-828），续页上的残段被
   显式跳过（framing.c:844-852）——**该包永远不会提交给 `vorbis_synthesis()`**，
   因此不会有 `OV_EBADPACKET`，只有 hole 标记。
2. 同样产生一次 `OV_HOLE`；若该续页是页上唯一内容，则该页无完整包、无
   granulepos 锚点（framing.c:882-885），位置恢复要再等后续页。
3. 与“整页丢失”相比，调用方可见行为**完全相同**（都是一次 OV_HOLE + 后续
   重锚）；【推断】库 API 层面无法区分“整页丢失”与“跨页包截断”，需要自行解析
   Ogg 页的 continued 标志与段表才能区分。

### 6.3 只能暴露不连续、无法恢复的情形

- 不可寻址流且残余数据再无带 granulepos 的页：只能继续产出相对顺序的 PCM，
  绝对播放位置无从锚定（§5.3）。
- 空洞一直延续到 EOF：读路径返回 0（EOF，vorbisfile.c:1986-1987），空洞后的
  位置基准永远缺失；EOS granulepos 又被重锚逻辑排除（vorbisfile.c:730）。
- 头区损坏：打开失败，连解码上下文都建立不起来。
- 载荷自洽性损坏（§4.4）：无错误码、无 hole，仅表现为样本/位置的静默缺口。

---

## 7. 责任边界清单

### 7.1 源码已保证的行为（库的责任）

1. **检测与上报**：页序号断档/CRC 失步最终转化为恰好一次 `OV_HOLE`
   （framing.c:831-834、950-956；vorbisfile.c:705），坏页不会被当作有效页解码。
2. **状态防呆**：乱序包到达时主动作废解码器内部 granule/sample 计数
   （block.c:745-749），避免把相对累加误当绝对位置。
3. **继续解码能力**：hole 标记消费后，后续完整、合法的音频包可立即继续解码，
   无需调用方干预；解码上下文（码本、模式、`vorbis_info`）不丢失。
4. **位置重锚**：下一个带 granulepos 的完整包会覆盖式重建 `pcm_offset`
   （vorbisfile.c:730-760）；可寻址流还可用 seek API 做带完整复位的确定性重建
   （vorbisfile.c:1255-1257、1580-1625）。
5. **seek 时的干净起点**：`vorbis_synthesis_restart()` 复位窗口/计数状态
   （block.c:695-716），`ogg_stream_reset_serialno()` 清空半包与段表
   （framing.c:912-938），抑制切入位置产生的伪 hole。
6. **多链接边界**：遇到 BOS 页按链接切换处理（vorbisfile.c:795-806），链接内
   hole 不破坏其他链接的缓存元数据。

### 7.2 调用方必须自己处理的状态（应用的责任）

1. **处理 `OV_HOLE` 返回值**：本次调用无 PCM，需要重试读；库不会自动填静音。
   官方示例 `examples/decoder_example.c:212-224` 对同步层/包层错误也只是打印
   “continuing...”后继续——即“继续”是应用行为，不是隐藏恢复。
2. **填补呈现层缺口**：若要静音、淡入淡出、插值或其他 PLC，必须由应用在自己的
   输出缓冲里做；样本缺口长度在重锚前未知，重锚后可由位置差分估算【推断】。
3. **不要把重锚前的 `ov_pcm_tell()` 当权威时间**；重锚瞬间的跳变要自行平滑
   （UI 时间显示、A/V 同步时钟等）。
4. **需要确定性恢复时主动 seek**（仅可寻址流）：例如要求空洞后严格对齐原始
   时间轴，可在收到 `OV_HOLE` 后用 `ov_pcm_seek_page` 跳到最近的带标记页重新
   建立解码状态——代价是重复解码与一次受控的重叠淡化（可用 `*_seek_lap` 系列）。
5. **容忍边界毛刺**：库不重置重叠缓冲（§4.2），对连续性要求高的应用需自行对
   空洞后首块做淡入/交叉淡化。
6. **头区失败是致命错误**：`OV_EBADHEADER`/`OV_EBADLINK` 不能靠重试取包恢复，
   需重新打开或更换数据源。

### 7.3 库明确不做、且无接口可做的事

- 不重建、不猜测空洞期间的原始 PCM 样本；不存在“把未知音频补写成已恢复样本”
  的路径。
- 不报告丢失字节数/包数/样本数（hole 标记 `0x400` 无载荷，framing.c:832）。
- 不区分整页丢失、跨页包截断、CRC 坏页三种成因（对上抛路径都是 `OV_HOLE`）。
- 流式（不可寻址）场景不提供绝对位置的再保证，只提供容器 granulepos 被动锚定。

---

## 8. 证据索引

| 结论 | 位置 |
|---|---|
| 同步层失步/CRC 失败重捕获 | libogg framing.c:641-725（684-693、713-724） |
| 页序号断档回滚半包 + 插入 `0x400` | libogg framing.c:822-835 |
| 孤儿 continued 段丢弃 | libogg framing.c:839-854 |
| granulepos 只挂最后一个完整包 | libogg framing.c:882-885 |
| 页号续接 / stream reset 语义 | libogg framing.c:895、912-938 |
| hole 标记消费、`packetout` 返回 -1、包号跳号 | libogg framing.c:940-956 |
| `OV_HOLE` 上抛点 | vorbisfile.c:701-705 |
| 读循环先排空 PCM 再透传错误 | vorbisfile.c:1977-1992、2129-2152 |
| granulepos 覆盖式重锚 `pcm_offset` | vorbisfile.c:730-760 |
| 内部忽略 hole 的辅助/打开路径 | vorbisfile.c:442、1732、2199、2217 |
| 头区 hole 即致命错误 | vorbisfile.c:372-375 |
| seek 的完整复位与伪 hole 抑制 | vorbisfile.c:1255-1257、1291-1294、1580-1625 |
| 乱序包作废 granule/sample 计数 | block.c:745-749 |
| 计数在新 granulepos 前的相对累加 | block.c:847-867 |
| 重叠相加无条件使用上一块缓冲 | block.c:779-824 |
| restart 仅用于 init 与 seek | block.c:695-723；vorbisfile.c:1257/1580/1622 |
| PCM 缓冲初始清零、无 PLC 清零路径 | block.c:245-252；mapping0.c:698-799 |
| 外推仅存在于分析路径 | block.c:426-526（无合成侧调用） |
| 坏音频包静默跳过 | synthesis.c:42-56；vorbisfile.c:709-766 |
| `OV_HOLE` 官方语义 | doc/vorbis-errors.txt:14-17；doc/vorbisfile/ov_read.html |
| 应用侧自行“继续”的示例 | examples/decoder_example.c:212-230 |

---

## 9. 【推断】汇总（源码未直接证明的判断）

1. 空洞后首块的具体**听感**（咔哒/毛刺）：叠加数学可证，听感描述属信号推断。
2. 丢失样本数需调用方在 granulepos 重锚前后做差分；库无现成字段。
3. 应用无法经 API 区分整页丢失与跨页包截断，需自行解析 Ogg 段表/continued 标志。
4. 不可寻址流若再无带 granulepos 的页，绝对位置在该会话内不可恢复（源码未见
   其他锚定途径的否定性结论）。
5. 自洽坏包被静默跳过时，调用方只能从位置停摆/跳变间接察觉，无专门通知通道。
6. Ogg 规范建议编解码器在数据间断后自行处理包依赖（libogg 注释 framing.c:951-952
   “it might need to handle previous packet dependencies”）；libvorbisfile 的实际
   选择是不重启解码器、继续叠加解码——这是实现策略而非规范强制，其他 Vorbis
   封装可以选择在 hole 后调用 `vorbis_synthesis_restart()` 获得更干净（但同样
   不补样本）的边界。
