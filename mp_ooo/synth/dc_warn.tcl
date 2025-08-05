# You requested %d cores. However, load on host %s is %0.2y
suppress_message UIO-231

# Design '%s' inherited license information from design '%s'.
suppress_message DDB-74
# Can't read link_library file '%s'
suppress_message UID-3
# The trip points for the library named %s differ from those in the library named %s.
suppress_message TIM-164

# There are %d potential problems in your design. Please run 'check_design' for more information.
suppress_message LINT-99
# Design '%s' contains %d high-fanout nets.
suppress_message TIM-134
# Design has unannotated black box outputs.
suppress_message PWR-428
# %s SV Assertions are ignored for synthesis since %s is not set to true.
suppress_message ELAB-33
# Ungrouping hierarchy %s before Pass 1.
suppress_message OPT-776

# Changed wire name %s to %s in module %s.
suppress_message VO-2
# Verilog 'assign' or 'tran' statements are written out.
suppress_message VO-4
# Verilog writer has added %d nets to module %s using %s as prefix.
suppress_message VO-11
# In the design %s,net '%s' is connecting multiple ports.
suppress_message UCN-1
# The replacement character (%c) is conflicting with the allowed or restricted character.
suppress_message UCN-4
# Design '%s' was renamed to '%s' to avoid a conflict with another design that has the same name but different parameters.
suppress_message LINK-17

# There are buffer or inverter cells in the clock tree. The clock tree has to be recreated after retiming.
suppress_message RTDC-47
# The design contains the following cellswhich have no influence on the design's function but cannot be removed (e.g. becauseadont_touchattributehas been setset on them). Retiming will ignore these cells in order toachieve good results: %s
suppress_message RTDC-60
# The following cells only drive asynchronous pins of sequential cells which have no timing  constraint.  Therefore  retiming will not optimize delay through them
suppress_message RTDC-115
# Unable  to  maintain nets '%s' and '%s' as separate entities.
suppress_message OPT-153
# The unannotated net '%s' is driven by a primary input port.
suppress_message PWR-416
# Datapath cell '%s' is not considered for gating.
suppress_message PWR-429
# The unannotated net '%s' is driven by a black box output.
suppress_message PWR-591
# Skipping clock gating on design %s, since there are no registers.
suppress_message PWR-806
# In design %s, there are sequential cells not driving any load.
suppress_message OPT-109
# In design '%s', input pin '%s' of hierarchical cell '%s' has no internal loads and is not connected to any nets.
suppress_message LINT-60
# Performing clock-gating on design
suppress_message PWR-730

# %s DEFAULT branch of CASE statement cannot be reached.
suppress_message ELAB-311
# Netlist for always_ff block does not contain a flip-flop.
suppress_message ELAB-976
# Netlist for always_comb block is empty.
suppress_message ELAB-982
# Netlist for always_ff block is empty.
suppress_message ELAB-984
# Netlist for always block is empty.
suppress_message ELAB-985

# output port '%s' is connected directly to output port '%s'
suppress_message LINT-29
# output port '%s' is connected directly to output port '%s'
suppress_message LINT-31
# In design '%s', output port '%s' is connected directly to '%s'.
suppress_message LINT-52

# In design '%s', cell '%s' does not drive any nets.
suppress_message LINT-1
# In design '%s', net '%s' driven by pin '%s' has no loads.
suppress_message LINT-2
# '%s' is not connected to any nets
suppress_message LINT-28
# a pin on submodule '%s' is connected to logic 1 or logic 0
suppress_message LINT-32
# the same net is connected to more than one pin on submodule '%s'
suppress_message LINT-33
# The register '' is a constant and will be removed.
suppress_message OPT-1206
# The register '' will be removed.
suppress_message OPT-1207
# the register '' is removed because it is merged to ''
suppress_message OPT-1215
# Complementing port ''
suppress_message OPT-319

# Converting capacitance units for library ''
suppress_message TIM-106
# Use the 'check_design' command for more information about warnings.
suppress_message LINT-98
