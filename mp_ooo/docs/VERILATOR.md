# Verilator

A fast, open-souce, cycle-based simulator.

# Why use Verilator?

One of the biggest gripes we tend to have with VCS is its runtime - VCS does a
lot of event management under the hood, and as a result takes a very long time
to simulate big programs like Coremark. However in ECE 411, when we want to see
how good our architectural changes are in terms of IPC, this can be prohibitive
for design space exploration (DSE).

Enter Verilator! By simulating your design only when you tell it to (usually on
clock edges), it manages to cut down simulation time significantly - like
"running Coremark in 3 seconds" significantly. Furthermore, Verilator lets you
write C++-based simulation models and testbenches. 

There's an element of convenience here. Verilator is an open-source project,
which means that unlike VCS (which is licensed), anyone can run Verilator at
home. For ECE 411, we will have some tips and tricks for getting Verilator set up
at home, but we do not *officially* support a course-staff-backed WFH
setup. This is due to the IP you will be using in `mp_ooo` - you do not have
access to this IP locally, and as a result will need to create some simulation
models to use on WFH. This does not need to be actual synthesizable HDL - just
some modules that replicate the expected timing and interface of IP you use.

## Pros and Cons

There are some other tradeoffs to consider. The big one is that Verilator
only supports dual-state simulation - this means that every signal in Verilator
is a 0 or a 1, and there is no X or Z tracking on signals. X-correctness is
something that many students struggle with, and you'll still need to use VCS to 
test your design to make sure you have no such issues. Verilator will treat Xes 
in your SV as a 0, so don't worry about potential compatibility issues in your SV. 
Due to this, **a working Verilator run does not imply correctness for a checkpoint 
or deliverable**.

Additionally, Verilator in the past has had LRM support issues when compared to
VCS. Constrained randomization and UVM, both things that are very important in
the verification industry, only started having basic Verilator support earlier
this year. There are also certain HDL constructs you write that may be flattened
correctly in VCS, but not in Verilator. This is primarily due to Verilator being
a less mature technology when compared to VCS. In this course we optionally offer 
using Verilator as a supplement to VCS for two reasons:

- Faster debug times when dealing with something like Coremark.
- Faster and more feasible design space exploration.

Overall Verilator is a very promising simulation tool with many big companies
backing it. Hopefully after `mp_ooo`, you will see some of the reasons why that is
the case!

# System Requirements

You will need to install the `verilator` program on your system. Most Linux
distributions and MacOS will have this in their package manager (`homebrew`,
`apt`, etc.). If you would like to view waveforms coming out of Verilator, you
will also need a waveform viewer - popular options are
[surfer](https://surfer-project.org) or
[gtkwave](https://gtkwave.sourceforge.net). `surfer` is best downloaded as a binary
off of their website, whereas `gtkwave` can be found in most package managers.
    
The ECE 411 tooling also compiles programs into a `.lst` file that is used to
initialize memory in the testbench. You will need the RISC-V toolchain installed
to support this functionality. EWS uses the 32-bit toolchain, but we also
support a 64-bit toolchain compiled with `multilib`. If you are attempting WFH,
you should be able to find this package on some package managers, like
`homebrew`, or compile it yourself.

If you are following this guide on EWS, `verilator` and the RISC-V toolchain has
already been installed for you, and is loaded as part of the `ece411.sh` script
you run to gain VCS access. However, you will still need a waveform viewer if
you wish to work with traces - Verdi does not support the `.fst` files that
Verilator dumps. We are currently working on getting `surfer` or `gtkwave` set up,
but for now please use the [VSCode version of
`surfer`](https://marketplace.visualstudio.com/items?itemName=surfer-project.surfer),
or some other generic waveform viewer of your choice. If you use the VSCode
waveform viewer, be aware that you should try to dump smaller traces when
possible to limit RAM consumption. More on this later.

# Running Your First Simulation

You should be comfortable running simulations with VCS by this point in the
course. Verilator simulations are near-identical. You can run the following
command in `sim`.

```bash
make run_verilator_top_tb PROG={your program}
```

That's it! You should see a bunch of terminal output, and some familiar
messages from Spike. Inside the `sim` directory, you will still see a
`commit.log` - however, note the lack of a `dump.fsdb`. `fsdb` is a trace format
used by VCS and Verdi - Verilator uses something called `fst`, which is a
different type of compressed trace format with very similar size overhead to
`fsdb`. You should see a `dump.fst` file, which contains the waveforms for 
the entire simulation.

If you are running large programs, your traces can get somewhat large -
Coremark, for example, generates about 500MB of traces in Verilator's
`dump.fst`. This is on par with the size of an FSDB for the same
benchmark. Ensure that you have sufficient storage to store the trace before 
you run simulation. You can also modify the harness C++ file to disable 
tracing altogether.

Something odd to note about our Verilator tooling is that you cannot kill the
simulation and retain your simulation results - that is to say, if you `CTRL-C`
while the program is running, you will not receive traces nor the logs. Luckily,
Verilator runs fast enough that you can reach the timeout in a reasonable amount
of time. However, if your CPU is hanging, you may be better off using VCS to
debug, since you can kill the program and still recover some traces.

# Running Lint

Verilator runs its own linter before every build. You can invoke the linter
without kicking off a build by running the following command in the `sim`
directory.

```bash
make run_verilator_lint
```

Like SpyGlass, Verilator has its own waiver file in `verilator_warn.vlt`. Note
that Verilator tends to have stricter lint than SpyGlass, so you can potentially
pass SpyGlass lint but struggle to pass Verilator lint.

Along a similar vein, since Verilator is a *simulation* framework (not a
synthesis flow), RTL that passes Verilator lint may not necessarily be
synthesizable. Multidriven signals or latches, for example, can sometimes go
uncaught by Verilator. It is in your best interest to run both lint flows
periodically!

# "False" Combinational Loops

A common issues students run into when first putting their HDL through Verilator
is combinational loops. At first glance, these messages seem erroneous. However, 
it is important to understand that Verilator looks for *event* loops, not synthesis 
loops. As a result, there are two big cases that will work on VCS but not
on Verilator.

The first is driving a member of a struct signal based on another member of the
same struct. For example:

```verilog
always_comb
  begin
    signal.rdy = signal.vld & (signal.idx == 3'b010);
  end
```

It does not matter how you register the different parts of the signal. Verilator
will usually throw a comb loop error on this logic, even though it might be
valid in synthesis. This may require you to keep extra registers and signals
outside of the struct, or rethink your struct definitions.

The other case is from ping-ponged `always_comb` statements. Consider the below example.

```verilog
logic a, b, c;

always_comb
  begin
    a = in;
    c = b;
  end
  
always_comb
  begin
    b = a;
  end
```

The first `always_comb` block will trigger an update of the second one, and that
will trigger the first one again due to the update of `b`.

This kind of issue can be mitigated by simply sectioning off your `always_comb`
logic to deal with similarly grouped signals. There's nothing wrong with having
multiple smaller `always_comb` blocks - in fact it's encouraged! 

# Final Comments

Verilator is a great tool to use for smoother WFH, fast benchmarking/DSE and
some basic debug in large program runs. That said, it can by no means be used as
a singular or exhaustive verification tool.

If you have any feedback or feature requests, please submit them to Campuswire,
or implement them yourself! You are not graded on Verilator simulations, so we
encourage playing around with the C++ and testbench collateral to make it "your
own" or add new features.
