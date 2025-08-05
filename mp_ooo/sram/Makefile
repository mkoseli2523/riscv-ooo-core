SHELL=/bin/bash -o pipefail
.SHELLFLAGS += -e

export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1

export CONDA_HOME=$(OPENRAM)/miniconda
export OPENRAM_HOME=$(OPENRAM)/compiler
export OPENRAM_TECH=$(OPENRAM)/technology
export PYTHONPATH=$(OPENRAM_HOME)

.PHONY: all
all:
	$(MAKE) clean
	python3 sram.py
	sync
	$(MAKE) --jobs=$(shell echo $(shell nproc)-2 | bc) $(notdir $(basename $(wildcard $(PWD)/output/*/*.py)))

%:
	source $(CONDA_HOME)/bin/activate ;\
	python3 $(OPENRAM_HOME)/../sram_compiler.py $(PWD)/output/$@/$@.py
	sed -i -E 's/falling_edge/rising_edge/g' $(PWD)/output/$(basename $@)/$(basename $@)_TT_1p0V_25C.lib
	source $(CONDA_HOME)/bin/activate ;\
	python3 $(PWD)/area.py $(PWD)/output/$(basename $@)/$(basename $@).gds > $(PWD)/output/$(basename $@)/$(basename $@).area.txt
	cat ./output/$(basename $(notdir $@))/$(basename $(notdir $@)).area.txt | xargs -I {} sed -i -E 's/area : .*/area : {}/g' output/$(basename $(notdir $@))/$(basename $(notdir $@))_TT_1p0V_25C.lib
	cd $(PWD)/output/$(basename $@) ;\
	lc_shell -x 'read_lib $(PWD)/output/$(basename $@)/$(basename $@)_TT_1p0V_25C.lib; write_lib $(basename $@)_TT_1p0V_25C_lib; exit' ;\
	rm -f lc*.log lc*.txt

.PHONY: clean
clean:
	rm -rf output
	mkdir output
	touch output/.gitkeep
