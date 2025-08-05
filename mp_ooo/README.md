# ECE 411: mp_ooo

This document, README.md, forms the specification for the machine problem. For
more details and documentation, visit the `docs` folder. Inside, you will find:

- [GUIDE.md](./docs/GUIDE.md): Some tips and resources for this MP.
- [WHAT_IS_AN_OOO.md](./docs/WHAT_IS_AN_OOO.md): Basic overview of an OoO
  processor.
- [ADVANCED_FEATURES.md](./docs/ADVANCED_FEATURES.md): List of potential
  advanced features and related advice.
- [VERILATOR.md](./docs/VERILATOR.md): About the Verilator simulation toolchain.
- [TEST_CASES.md](./docs/TEST_CASES.md): Description of test cases.

# Introduction

This machine problem involves the design of an out-of-order microprocessor. You
are required to implement the RV32IM instruction set (with the
exception of the `FENCE*`, `ECALL`, `EBREAK`, and `CSRR` instructions). This handout is an incomplete
specification to get you started with your design.

## Point Breakdown

| Checkpoint          | Points |
|---------------------|--------|
| CP 1                | 15     |
| CP 2                | 25     |
| CP 3                | 25     |
| Advanced Features   | 25     |
| Presentation/Report | 10     |

**Total Points:** 100

Detailed rubrics for each checkpoint will be added below closer to their release
date.

Students can also score up to 10 points of extra credit by making additional
advanced features. The maximum possible grade on this MP will be *110 points*.

## Overview

By the end of this MP, you will make an out-of-order processor implementing:

- Either Tomasulo's Algorithm or an Explicit Register Renaming scheme
- Split instruction cache and data cache
- Support for multiplication and division instructions
- Advanced features of your choice

In total, this MP will span 8 weeks. In the first five weeks, you will
develop the core microarchitecture for an RV32IM OoO processor. By
Week 2, you will pick either a Tomasulo-style or ERR-style processor.
Your group will be assigned a mentor CA/TA that has worked with that
microarchitecture in the past. They will be your primary source of
guidance throughout this MP, and will also be responsible for grading
your submissions. You will have weekly meetings to discuss your
progress on the MP and grade your checkpoints.

The two weeks after will be for Advanced Features, where you improve the
performance of your baseline RV32IM processor on a set of staff-provided
benchmarks. Your goal will be to outperform a fixed set of performance targets
and baseline processors. Unlike prior semesters of ECE 411, **you will not be
graded relative to your peers' submissions**.

During this time, there will be a Design Competition run in parallel.  Course
staff will maintain a "Leaderboard", which periodically runs various benchmarks
using your processor. You will be ranked against your classmates' processors, as
well as some staff-provided baselines.  While this will not impact your grade,
there are prizes up for grabs provided by our sponsor, Optiver!

## mp_cache Dependency

Checkpoints 1 and 3 of this MP involve integrating instruction and data caches
with your processor - typically, this is done by using modified versions of
`mp_cache`.

If all members in your group were unable to complete `mp_cache`, don't
worry - we provide the RTL for a direct-mapped write-through cache.
However, this cache will be less performant and area/energy-efficient
than `mp_cache`. We highly recommend attempting to complete your
`mp_cache` submission.

## Outside Code

For this MP, all the RTL/code that you use should be your own, with the
exception of staff-provided resources. In addition, you may use any IPs
categorized under `Datapath: Arithmetic Components`, `Datapath: Floating Point`,
and `Datapath: Sequential Components` as categorized by [the DesignWare
docs](https://www.synopsys.com/dw/buildingblock.php). The most relevant IPs for
implementing RISC-V extensions will be:

- Multiplier
- Divider
- Relevant functional units in `Datapath: Floating Point`

Use of other outside RTL/code is prohibited.

# Checkpoints

This MP is split into 3 mandatory checkpoints, and a final evaluation after the
advanced features stage.

## Submissions

You will be graded by your mentor during your weekly meetings. For Checkpoints 1
& 2, you will be manually graded on a commit of your choice. This commit should
be pushed to GitHub before the stated deadline on each checkpoint. For
Checkpoint 3 onwards, you will be evaluated by an autograder, and standard MP
policy will apply.

With each checkpoint, you will submit a progress report. This should
discuss the progress your team made, important design decisions,
relevant performance metrics, etc. This will serve as a way for your
mentors to keep track of your design process, and provides material
for your final report and presentation.

Though it is 8 weeks long, `mp_ooo` is a fast-paced MP with heavy dependencies
between checkpoints. It is in your best interest to start early and stay synced
with your mentor.

## Late Policy

Each checkpoint will be graded with the standard late policy shown on [the class
website](https://courses.grainger.illinois.edu/ece411/sp2025/policy.html). Unlike
prior MPs, there will be **no "late voucher"** system available for checkpoints.

There will be no partial credit awarded beyond what is outlined in the rubrics below.

## Checkpoint 1 (15 points)

**_Due 3/28, 11:59PM_**

In this checkpoint, you will be designing the high-level microarchitecture for
your processor, creating the front-end fetch stage, attaching an instruction
cache, and integrating it with a DRAM model. At this time, you should also
decide whether you will be developing a Tomasulo-style or ERR-style
microarchitecture.

Unlike `mp_pipeline`, your fetch stage should also contain a linebuffer that
fulfills the following criteria:
- When fetching an instruction from a new cacheline, you will make a request
  from the cache. 
- The cache will return the entire cacheline, which you will store in
  a linebuffer (256-bit register storing a single cacheline).
- If the next instruction fetch is from the same cacheline, you will
  get it directly from the linebuffer.
This linebuffer-based design is used to avoid `mp_cache`'s issue of 
only being able to respond every other cycle.


**Requirements:**
- Draw a design block diagram (using draw.io or similar, **not handwritten!**)
  of your OoO datapath [5]
  - Graded on completeness, state/information flow, and readability
- Implement and test a parameterizable queue (depth and width) [3]
  - You are expected to identify relevant edge-cases of a FIFO, and show your
    tests for them
- Implement and test a cacheline adapter to deserialize a burst out of the DRAM
  model, and integrate this with your cache. [3]
- Implement instruction fetch, integrate it with your cache, and feed fetched
  intructions into your queue [4]
    - Your program counter should be initialized to `0xAAAAA000`

## Checkpoint 2 (25 points)

**_Due 4/11, 11:59PM_**

By the end of this checkpoint, your processor must be able to execute all
immediate and register instructions in the RV32I spec. In addition, you will be
responsible for integrating multiplier and divider IPs from Synopsys to implement the
RISC-V M extension.

You will need to handle the DIV-0 edge case - check the RV32 ISA spec to
understand the expected behavior.

**Requirements**
- Implement your OoO architecture such that it can execute all immediate and
  register instructions in RV32I [8]
  - `AUIPC` is *optional* for this checkpoint
- Integrate a Synopsys multiplier IP with your processor to perform all `MUL`
  instructions in RV32M [3]
- Integrate a Synopsys divider IP with your processor to perform all `DIV`/`REM`
  instructions in RV32M [3]
- Modify your `mp_verif` random testbench to test coverage on all the
  instructions required for this CP [3]
- Demonstrate that your processor is able to execute instructions out-of-order
  (via `testcode/ooo_test.s`) [5]
- Show that your processor is compliant with the Spyglass linter [3]

*There will be no provided tests outside of `ooo_test.s`* - if you are unable to
correctly modify the random testbench, you must demonstrate correctness by
writing your own comprehensive tests to receive credit for the instructions you
implemented.

## Checkpoint 3 (25 points)

**Due: 4/25, 11:59PM**

By the end of this checkpoint, your processor must be able to execute the
entirety of the RV32IM spec (barring the exceptions outlined
[here](./#Introduction)). You will also integrate data and instruction
caches with your processor, and add necessary arbitration logic for accessing the DRAM model.

Deliverables for this checkpoint are:

- Integrate the data cache with your core
  - Add any necessary write/arbiter logic to the cacheline adapter
- Support all memory instructions
  - Memory instructions can be performed in order on ROB commit
  - Recommended to support loads before ROB commit (refer to lab)
- Support all control instructions (`BR`, `JAL`, `JALR`) and `AUIPC` if not yet
  implemented
  - Not required to support branch prediction yet, static taken or not-taken is okay

This will be the first checkpoint graded by an autograder. In the meantime, you
can use random/targeted tests, `coremark_im`, and any other released tests in
order to test your processor. 

Your deliverables are not graded individually. Instead, credit is assigned with
the following rubric.  Additionally, there will be no partial credit for this
checkpoint - you will receive credit for each benchmark you complete correctly.

- Pass synthesis and lint without warnings [5]
- Run each of the benchmarks listed below without any Spike mismatches or RVFI
  errors. Credit here is predicated on you fulfilling the above criteria.
  - Coremark IM [4]
  - FFT [4]
  - Mergesort [4]
  - AES-SHA [4]
  - Compression [4]

Information about the above benchmarks can be found in
[ADVANCED_FEATURES.md](./docs/ADVANCED_FEATURES.md).

## Advanced Features (25)

**Due: 5/9, 11:59PM**

This is the fun part of `mp_ooo` - optimizing your processor! You will be
implementing processor enhancements of your choice. A list of example advanced
features and their associated point values can be found in
[ADVANCED_FEATURES.md](./docs/ADVANCED_FEATURES.md). Some of these features
require significant design considerations when constructing the core
microarchitecture, so be sure to take a look at this early on in your design
process.

This portion of your grade has two parts: implementing advanced
features of your choice, and evaluating performance relative to a
set of baseline metrics on staff-selected benchmarks.

The initial benchmark suite and the related performance metrics will be
announced on the first day of Advanced Features (4/25).

Note that you will need to justify your advanced feature selections during your
final report, and include quantitative analyses of the performance
benefit/degradation caused by each advanced feature. *This is required to score
any points for a feature*. You may benchmark advanced features either relative
to CP3 or prior to adding the advanced feature to your processor. Be detailed
about your benchmarking and profiling configurations.

The overall split of the points in this section will be 15 points allocated
based on implementing features detailed in
[ADVANCED_FEATURES.md](./docs/ADVANCED_FEATURES.md), and 10 points for beating
the baseline metrics specified. You can earn up to 10 points of extra credit for
implementing additional advanced features.

You may have *two* graded branches in your GitHub repository for these
deliverables.  The `main` branch will be used by the AG for performance testing,
and you can have additional branch of your choice. We will grade the advanced
features in both of these branches, but you cannot receive credit for the same
feature implemented on both branches. You may not submit advanced features on
any additional branches for credit. Branches are expected to meet CP3
correctness in order to net credit for the implemented advanced features.

### Implementation

15 points are awarded for implementing advanced features. In addition to the 
previously mentioned performance evaluation requirement, the following rules 
dictate how you can earn these 15 points (25 if counting extra credit):

- You must make advanced features in 3 different categories *excluding*
  Misc. and Software/Performance features before getting points for two or more
  advanced features in the same category
- Once you meet *both* the above criterion and the 15 point minimum, you may
  earn up to 10 additional advanced feature points as extra credit.
  
### Metrics
For this MP, you will be evaluated on two different metrics

- IPC
  - IPC is reported by AG's `top_tb` at the end of simulation
- $PD^4$, where $P$ is power and $D$ is delay
  - Power is reported by AG's `make power_vcs` in `/synth` in mW
  - Delay is reported by AG's `top_tb` at the end of simulation in ps
    - When testing locally, use *Monitor Segment Time*
    - Use *ms* when calculating this metric (divide by $10^9$)

You may synthesize at any frequency you wish. However, we impose an area limit of
300K as reported by Synopsys DC. For violations over this limit, we will multiply
the graded/leaderboard metrics by $2^{\frac{A - 300,000}{50,000}}$, where $A$ is the area
as reported by DC.

### Performance Scoring

In addition to the implementation points, 10 points will be allocated for how performant
your advanced features are when compared to our baseline processor. This baseline is a
conservative implementation of a CP3 core. On each of the 5 CP3 benchmarks, you will
be compared with the baseline on each of the following:

- IPC (must be above) [0.5]
- $PD^4$ (must be below) [1.5]

Full credit is awarded for beating the baseline on the respective metric. We also offer
the following partial credit.

- Within 10% of graded metric: 80% credit
- Within 20% of graded metric: 50% credit
- Any lower: no credit

For reference, the metrics of the baseline processor on each of the CP3 benchmarks is
in the table below.

| Benchmark   | IPC  | Power (mW) | Delay (ms) | $PD^4$  |
|-------------|------|------------|------------|---------|
| coremark_im | 0.41 | 18.88      | 1.43       | 78.19   |
| aes_sha     | 0.32 | 18.61      | 4.85       | 10317.6 |
| compression | 0.48 | 18.92      | 1.75       | 176.95  |
| fft         | 0.55 | 27.29      | 4.25       | 8908.97 |
| mergesort   | 0.45 | 18.92      | 2.15       | 406.84  |


### Design Competition/Leaderboard

The leaderboard, like the autograder for past MPs, will run periodically and
test a series of benchmarks on your processor. If these benchmarks run
correctly, then the Leaderboard will assign a score based on a composite score
of the benchmark results. The "open" set of benchmarks were released in CP3, and
another 5 closed benchmarks will be run for ranking purposes.

To show up on the leaderboard, your processor must:
- Be connected to RVFI
- Match Spike and RVFI on **all test cases**
- Be synthesizable and meet timing at the frequency you specify in `options.json`

Note that not all leaderboard benchmarks will be public. There will be a set 
of "closed" benchmarks that will not be distributed as part of the `mp_ooo`
release. 

You will be ranked based on the geometric mean of your $PD^4$ scores across the
10 benchmarks in our suite.

*Note*: The area penalty outlined earlier is applied for the design competition as well.

## Presentation & Final Report (10 Points)

You will submit a final report detailing your processor architecture and design
choices made throughout the MP, as well as performance metrics for the various
features and configurations you implemented. In addition, you will prepare a
brief presentation to give to course staff about your processor.

Presentations will happen during finals week by appointment with course
staff. Your final report will be due on **5/17** at midnight.

## Group Evaluations

At the end of the project, each group member will submit feedback on how well
the group worked together and how each member contributed to the project. The
evaluation, along with feedback provided at TA meetings throughout the semester,
will be used to judge individual contribution to the project.

Although the group evaluation occurs at the end of the project, this should not
be the first time your mentor TA hears about problems that might be
occurring. If there are major problems with collaboration, the problems should
be reflected in your TA meetings, or in private communication with your TA. The
responses on the group evaluation should not come as a surprise to anyone.

In the event of significant to extreme imbalance in work distribution between
team members, course staff reserves the right to apply corrective bonuses and
penalties to the affected students. This will be done only on a case-by-case
basis.

# Memory Model

Also called the DRAM model, you will use a banked burst memory model throughout
this MP.  Unlike previous memory models used throughout the semester, this model
supports **multiple outstanding requests**. This means you can send multiple
requests before the first response comes back from memory. Furthermore, if you
issue multiple read requests, they might come back in a different order than you
sent the requests. Conflicting writes will **not** be handled out of order.

You are free to use this feature or not at your discretion. It may help
performance significantly when paired with other advanced features. If you
prefer, you can ignore this feature and use it very similarly to previous memory
models which only supported one outstanding request.

<p align="center"> <img src="docs/images/bmem_read_single.svg"/> <p
  align="center">Single Read Request</p> </p>

<p align="center"> <img src="docs/images/bmem_write_single.svg"/> <p
  align="center">Single Write Request</p> </p>

<p align="center"> <img src="docs/images/bmem_read_queue_full.svg"/> <p
  align="center">Read Queue Full</p> </p>

<p align="center"> <img src="docs/images/bmem_write_queue_full.svg"/> <p
  align="center">Write Queue Full</p> </p>

<p align="center"> <img src="docs/images/bmem_read_ooo.svg"/> <p
  align="center">Out-Of-Order Read Response</p> </p>

<p align="center"> <img src="docs/images/bmem_same_addr_rw.svg"/> <p
  align="center">Mixed Reads and Writes to the Same Destination</p> </p>

# Teamwork

As mentioned earlier, it is crucial in `mp_ooo` that you split work in a
sensible manner across your team. Frequent communication with your groupmates
and your mentor are key to ensuring that this is done in a reasonable manner.

In the event that a groupmate is unresponsive or not contributing their fair
share, it is important that you address this with relative haste. Pushing this
off until group evaluations makes it harder for us as a course staff to deal
with such issues. Ensure that you take the following actions:

1. Report to your mentor as early as possible if someone in your team has
   stopped responding or isn't contributing any (non-trivial) work.
2. Document all evidence, such as chat logs, repo commit histories, etc. of
   their lack of contribution and due diligence.
3. Be explicit in your progress reports about what was and wasn't done as a
   group, and any important individual contributions.

Team members who under-contribute or contribute nothing will lose part or all
(100%) of their final `mp_ooo` grade, depending on severity. These points may be
reallocated/awarded to other groupmates who had to over-contribute as a result.
