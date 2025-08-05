# ECE 411: mp_ooo ADVANCED_FEATURES

## Advanced Feature Rules:

You will need to implement a minimum number of advanced features such that you
cover 3 different categories as described below and meet a minimum point total
of 15 points. You must meet the 3-category requirement before you can receive
points for multiple features in the same category - e.g. you should implement
features in 3 categories before you implement 2 features in the same category. 

## All Advanced Features

This list is not exhaustive. If you have a cool idea or find a relevant research
paper you are interested in, make a Campuswire post and course staff will assign
points for it. If the paper you implement is especially unique, you might
receive extra points for originality. 

Additionally, if any of the requirements are unclear, or if you have an idea that
may fall in a grey area, please contact your mentor or make a Campuswire post. 

### Memory Unit:

- Split LSQ (Loads can execute out-of-order in between in-ordered stores) [4]
  - _Loads can be issued OoO w.r.t stores to a different address_ [2]
- _Uncommitted Store Forwarding (e.g. Pre-commit Store Buffer w/ Forwarding)_ [2]
  - _Non-Committed & Misaligned Store Forwarding_ [1]

### Cache Hierarchy:

- Post-Commit Store Buffer [5]
  - _Write Coalescing in Store Buffer_ [2]
- Non-Blocking Data Cache for a single miss (hits following 1 miss do not
  stall) [3]
  - _Non-Blocking Data Cache for multiple misses (multiple outstanding
    misses tolerated without stalling hits)_ [5]
- Pipelined Cache (must meet specification provided in `mp_cache_pp`) [4]

### Prefetchers:

- You can receive credit for **one** of the following.
  - Next-Line Prefetcher [2]
  - Stride Prefetcher [2]
- Stream Buffer Prefetching [3]
- Fetch Directed Instruction Prefetching [4]
- DMP (pointer chasing) [4]

### Branch Predictors (Half points if structures implemented with FFs instead of SRAM):

- TAGE [8]
- Perceptron [6]
- GShare/GSelect [4]
- Two-Level Predictor (Local History Table & Pattern History Table) [4]
- Enhancements:
  - _BTB (Branch Target Buffer)_ [2 if implemented alongside a
    predictor]
  - _Tournament/Hybrid/Combined_ [2]
  - _Overriding branch prediction_ [2 if also using TAGE/Perceptron]

### RISC-V Extensions:

- C (Compressed Instructions) Extension [5]
  - _If also superscalar_ [3]

### Advanced Microarchitectural Optimizations:

- Superscalar (2-way) [10]
  - Superscalar (N-way parameterizable) [5]
- Early Branch Recovery (requires checkpointing & branch stack masking) [8]
- Fire & Forget [15]

---

Features below this point are not eligible for credit until you meet the
3-category minimum.

### Performance Analysis, Visualization, & Verification:


- Design Space Exploration Scripts [1-5]
- Non-Synthesizable Processor Model [1-5]
- Benchmark Analysis [1-5]
- Processor Visualization Tool [1-5]
- Full-System UVM Verification Environment [1-5]
- Full-System Cocotb Verification Environment [1-5]

All of the above features will require explicit approval from one of Pradyun,
Stanley, or Professor Wang. Please describe your suggested implementation in a
CW post tagged `[Software Feature]` in the title, and we can assign the proposal
some point value.

### Misc:

- Age-Ordered Issue Scheduling [2]
- Banked Cache [2]
- Parametrized Sets & Ways Cache (including PLRU) [2]
- Return Address Stack [2]

## Advanced Features that Especially Benefit from Early Consideration

### Superscalar

Out-of-order processors exploit instruction level parallelism for
better performance. Why not exploit it some more?

A superscalar processor is a processor that can handle >1 instructions
in the same clock cycle at every stage: Fetch, Decode, Dispatch,
Issue, Writeback, Commit, etc. The *superscalar width* of a processor
is the minimum number of instructions each stage can handle in the
same clock cycle.

For example, a processor that fetches only 1 instruction at a time is
not superscalar, even if the rest of the processor can handle more
than one instruction simultaneously. A processor with a minimum width
of 2 at all stages would be called a 2-way superscalar processor. A
parametrized-width processor would be called an N-way superscalar
processor.

Without stalls, a 2-way superscalar processor should be able to
achieve a theoretical sustained IPC of 2.0 for perfectly parallel programs.

If you are interested in this feature, you should plan ahead when
writing your code. Either make your processor superscalar from the
start, or write your code clearly so you can easily extend it later.

### Early Branch Recovery

Your out-of-order processors have a deep pipeline. It can take dozens
of clock cycles before a branch makes its way to the head of the
ROB. Therefore, mispredicted branches can have a large impact on
performance. If you think back to `mp_pipeline`, you may remember 
that you have to flush several stages whenever you mispredict a branch. 
This problem becomes worse as you add more stages, and out-of-order 
processors can be hurt even more significantly.

When you mispredict a branch, there may be several instructions
elsewhere in your pipeline that should not be committed. In
`mp_pipeline`, we were able to recover in a relatively straightforward manner
by squashing branches at pipeline stages earlier than the branch. This
is not so straightforward in an out-of-order processor. For example,
it can be tricky to keep track of which instructions are younger than
others as they can now execute out of order. If you directly implement 
the processor described in lecture, the only structure that maintains program 
order is the ROB. Consequently, the simplest way to handle mispredicts is to 
flush everything when committing a mispredicted branch (and no
earlier). Depending on exactly when the branch commits, this can take
a long time, resulting in a very large mispredict penalty.

Early branch recovery solves this by adding logic to your pipeline to
enable branches to flush only instructions younger than themselves
before commit. In the most ideal case, as soon as the branch is
resolved, you can squash all of the incorrectly fetched instructions.

If you are interested in this feature, you should consider from the
beginning how to tag instructions in your processor with the metadata
necessary for squashing logic. This can be especially tricky with
explicit register renaming, or complicate adding other features.

## A Few Words of Warning

It is critical to consider which advanced features pair well with one
another. Many features may have benefits that are not proportionate with point
value. Some features may only show meaningful benefit on certain benchmarks, 
and not all CPU designs and feature combinations will benefit identically 
from each additional feature. Depending on where the bottlenecks are in your
CPU design, some features may not help performance under any circumstances at all.
Others may dramatically improve performance. Some features may rely so heavily
on another feature that their benefit is not "unlocked" until that other feature
is implemented. 

Thus, we **strongly** recommend adding performance counters/hooks 
all over your processor so you can identify where to improve performance.
As computer architects, you should also carefully consider your prospective 
advanced features (absolutely consult with your mentor TA if you are unsure!) 
so that your team spends its time working on features with good synergies. 
