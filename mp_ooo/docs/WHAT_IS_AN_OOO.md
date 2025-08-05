# ECE 411: mp_ooo WHAT_IS_AN_OOO

Below is a guide explaining out-of-order processors and some details
on the Tomasulo's Algorithm and Explicit Register Renaming microarchitectures.

## Why Out-of-Order?

Out-of-order architectures are popular for high-performance CPU
designs. They reduce performance penalties by eliminating false
dependencies between nearby instructions.

For example, consider the following simple program:

```
1: LW 0x12345678, x1 // load value at address into x1 

2: ADD x1, x1, x2 // add x1 to itself and put into x2

3: MUL x3, x3, x4 // multiply x3 with itself and put into x4
```

Here, instruction 2 is dependent on the result of instruction 1, but
instruction 3 is independent of both 1 and 2.

Assume the following:
- `LW`s have a latency of 20 cycles
- `ADD`s have a latency of 1 cycle
- `MUL`s have a latency of 20 cycles

In an in-order pipelined processor, we would need to execute
instructions 1-3 sequentially. Due to the imbalance between
instructions 1 and 2, there is a severe performance
penalty. Instruction 2 will sit in the decode stage for 20 cycles
while the `LW` waits for its data. Instruction 3 will be stuck in
fetch. This means the total execution time would be around 45 clock
cycles.

An out-of-order processor would identify that the `MUL` is independent of
the `LW`. Thus, the `MUL` can execute before the `ADD`, in parallel with
`LW`. The total execution time would thus drop to be around 25 cycles.

The above calculations are approximate, but the conclusion is
clear. Out-of-order processors can exploit instruction level
parallelism, which can improve performance.

## What is Tomasulo's Algorithm (+ ROB)

Tomasulo's Algorithm is an algorithm for out-of-order execution. It
allows the execution of instructions without data dependencies to be
re-ordered. This is done using two structures:
- **Reservation Stations**: These contain all the information for an
  instruction's dependencies, operands, and any other metadata needed
  to execute the instruction.
- **Re-Order Buffer (ROB)**: This is a queue which keeps track of the
  order of instructions, and stores the destination and the data to be
  written.

The ROB maintains something called **precise state**. Precise state is
the idea that if your processor is stopped for any reason at any
moment, the state of the register file and memory is the same as if
the program were executed on an in-order processor. While your
processor won't need to support interrupts, we still need some precise
state to ensure that changes appear as they should, and that
speculative execution can be rolled back accurately.

Tomasulo's Algorithm does not directly support precise state. For our
purposes, precise state means that the programmer (or OS) sees
register/memory changes as if they were modified in order. In other
words, if the programmer were to pause execution at any point in time,
they should see a program & memory state identical to as if it was
executed in order.

In order for Tomasulo's Algorithm to support precise state, an
additional structure called a reorder buffer (ROB) is used. The ROB holds the
instructions in order and waits for instructions to finish
executing. Once an instruction finishes executing and is the oldest
instruction in the ROB, it can then commit, ensuring instructions
commit from the ROB in order. When an instruction commits, the
architectural register file is updated and stores have their values
written to memory.

## Tomasulo's Algorithm Details

These are the key components of Tomasulo's Algorithm (+ Precise State)
and when you will implement them during this MP:

- Fetch Logic [CP1]
- Decoder [CP2]
- The ROB [CP2]
- The Reservation Stations [CP2]
- The Functional Units [CP2]
- The Common Data Bus (CDB) [CP2]
- The Register File [CP2]
- Load/Store Queues [CP3]
- Branch Resolution [CP3]

You can (and should) reuse logic from earlier MPs to implement the
fetch logic, decode logic, functional units, and register file.

Below is a high-level diagram of the Tomasulo architecture (taken
from lecture). Many details are not shown. Furthermore, this diagram
assumes only floating point instructions (which you do not need to
support). You will need to support out-of-order execution of the RV32IM
ISA. Thus, it is imperative that you take the time to draw out a more
detailed diagram during CP1 that more closely matches what you will be
developing.

<p align="center"> <img src="images/tomasulo.jpg"/> <p
  align="center">Tomasulo's Algorithm</p> </p>

Please refer to lecture for more details on how these hardware blocks
interact.

## Limitations of Tomasulo's Algorithm

Tomasulo's Algorithm is not perfect, as it has a number of limitations
that result in a lower clock frequency, higher area, and higher power
consumption. Here are a few:

- Register values are written twice: once to the ROB when an
  instruction finishes execution and once to the architectural
  register file once an instruction commits. This increases the energy
  consumption per instruction.
- Instruction values may come from either the ROB, the architectural
  register file, or the CDB. All of these structures are read at the
  same time. This complicates instruction dispatch and severely limits
  your clock frequency when you go superscaler.

One solution to this is a more modern algorithm: Explicit Register
Renaming, and reading data after the issue stage.

## Parts of an Out-of-Order Pipeline

Below is a list of stages, their names, and what they do. We may say
some of these words, and this is what they mean:

- **Fetch** - Fetches instructions
- **Decode** - Decodes instructions
- **Rename** - Maps an architectural register ID to a physical
  register ID.
- **Dispatch** - Moves an instruction from the in-order front-end to
  the out-of-order back-end. This involves allocating reservation
  stations and ROB entries.
  - **Front-end** - The part of the processor that gets instructions
    and figures out what to do. Made up of multiple stages like Fetch,
    Decode, Rename, and Dispatch.
  - **Back-end** - The rest of the processor, which is responsible for
    executing the instructions and committing them.
- **Issue/Wakeup** - This stage keeps track of instruction
  dependencies, and *issues* (moves) instructions to the execution
  units when the dependencies are resolved.
- **Execute** - Executes an instruction.
- **Writeback/Resolution** - After execution, moves the result to the
  proper location and marks any dependencies as resolved. Tells the
  ROB that the instruction is finished.
- **Commit/Retire** - Modifies the architectural state of the
  processor. This is done by dequeueing ready entries in the ROB. At
  this point, an instruction is finished.

## Explicit Register Renaming & Read After Issue

The basic dataflow diagram of the back-end of a Tomasulo processor
looks as follows:

<p align="center"> <img src="images/basic_tomasulo.svg"/> </p>

A few things to note:
- Rename/Dispatch gets the source operands from looking at the
architectural register file, the ROB, and the CDB. If none of them contain
the up-to-date data, the source operand is marked "Not Ready".
- The reservation station issues an instruction to the functional unit
only once it contains all of the data necessary for the
instruction. Therefore, a given register's data can be stored in three
places at once: the ROB, the architectural register file, and the
reservation station. This is wasteful!

Let's now look at a simple explicit-renaming merged-register-file
read-after-issue architecture (similar to what you will be making
if you choose this option):

<p align="center"> <img src="images/merged_v1.svg"/> </p>

A few new structures have popped up. Let's define what they are:

- **Physical Register File**\
  The basic idea behind this is that we want to store all of the data
  into one place. We don't want to waste space by storing the same
  register in three different places. Therefore, we must create that
  one place, which is what this register file is for. To avoid storing
  excess data inside the reservation station, we also only read from this
  register file *after* issuing an instruction.

  - **Physical vs Architectural Registers**\
    Architectural registers are the registers defined by an ISA. For
    example, RISC-V defines 32 general purpose registers.\
    A processor doesn't need to only contain 32 registers. In
    Tomasulo's algorithm, every ROB entry can also be used as a source
    operand for an instruction. Therefore, the total amount of
    *physical registers* is defined as the sum of all registers inside
    a processor that can store operands. In a Tomasulo processor,
    there are (32 + # of ROB entries) physical registers.

- **RAT** - Register Alias Table\
  The processor contains a bunch of physical registers. How do we know
  which physical register contains which architectural register's
  data? Here comes the RAT!

  The RAT contains 32 entries, one for each architectural
  register. Each entry stores two pieces of information: the mapped
  physical register index, and whether the physical register file
  contains the data.

- **RRF** - Retirement Register File\
  This structure looks very similar to a RAT. The major difference is that
  it comes after the ROB, and is updated only when an instruction is
  committed/retires.

- **Free List**\
  The free list is simply a queue of physical register indices which
  do not store relevant data for any architectural register. It is
  used to rename architectural  destination registers by assigning free
  physical ones to them.

Let's go through an example instruction executing in this pipeline:
`addi x2, x1, 4`.

After an instruction is fetched and decoded:

- Cycle 0: Dispatch/Rename
  - Look at the *RAT* to see what physical register contains the data
    for `x1`. Let's say the RAT entry for `x1` is `1(valid) -
    p1`. This means that the correct data is currently inside physical
    register `p1`.
  - Look at the *Free List* for an unused physical register that we
    can use to store the output of this instruction. Let's say the
    register at the head of the free list is `p33`. We dequeue it.
  - We need to update RAT entry `x2` to point to the result of this
    instruction, so that future dependent instructions execute
    correctly. Let's set RAT entry `x2` to `0(invalid) - p33`.
  - Like a Tomasulo processor, we also need to enqueue ourselves to
    the *ROB*. Let's enqueue the following: `0(not ready to commit),
    p33(destination physical register), x2(destination architectural
    register)`. The ROB returns the index where this was
    enqueued. Let's say it returned `ROB0`.
  - Now we have all of our information, let's put the renamed
    instruction inside a *reservation station*: `add, x2 --> p33,
    p1(valid), imm(4) --> ROB0`.


- Cycle 1: Issue/Regfile Read
  - Look at the *reservation stations*. Since all the source operands
    are ready (`p1(valid)` and `imm(4)`), we can issue this
    instruction.
  - Read `p1` from the *physical register file* and send that data to
    the *functional unit*.
  - Send `imm(4)` and all other metadata to the *functional unit* from
    the *reservation station*.
  - Mark the *reservation station* as free.

- Cycle 2: Execute\
  This looks the same as a pipeline processor. We just execute the operation
  as all the sources are here.

- Cycle 3: Writeback\
  - Broadcast the result, the destination architectural register index,
    the destination physical register index, and the ROB entry index on
    the CDB. It looks like this: `ROB0, x2, p33, result`.
  - Write `result` to the *physical register file* at index `p33`.
  - Set all *reservation stations* waiting for `p33` to ready. This is
    also called *wakeup*.
  - Mark the *RAT* entry at index `x2` as `valid` if it still maps to
    `p33`.
  - Mark the ROB entry `ROB0` as ready-to-commit.

- Cycle 4: Commit\
  When the instruction is the head of the ROB, it will commit when it
  is marked as ready-to-commit. When it commits, it will:
  - Look at the current mapping inside *RRF* entry `x2`. Let's say it
    was `p2`. Since that architectural register is being updated, the
    old physical register mapping needs to be freed. Therefore, we
    need to enqueue `p2` onto the *free list*.
  - Set the *RRF* entry `x2` to map to the new destination: `p33`.

You have just finished your first instruction! While this example
doesn't cover every edge case, we hope it gives you a better
understanding of what an out-of-order processor with explicit register 
renaming looks like.
