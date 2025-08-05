SHELL=/bin/bash -o pipefail
.SHELLFLAGS += -e

export PKG_SRCS = $(PWD)/../pkg/types.sv
export HDL_SRCS = $(shell find $(PWD)/../hdl -name '*.sv')
export SRAM_LIB = $(shell find $(PWD)/../sram/output/ -name "*.db")
export DESIGN_TOP = cpu

.PHONY: lint
lint: clean
	mkdir -p reports
	sg_shell -licqueue < lint.tcl |& tee reports/lint.log
	mv spyglass-1/consolidated_reports/*_lint_lint_turbo_rtl/*.rpt reports/
	mv reports/moresimple_turbo.rpt reports/lint.rpt
	rm -rf spyglass-1 WORK *.log
	bash check_lint_error.sh

.PHONY: clean
clean:
	rm -rf spyglass-1 WORK *.log reports
