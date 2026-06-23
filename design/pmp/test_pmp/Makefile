RTL_SRC := eh2_pmp_test.sv
TB_SRC  := tb_eh2_pmp_test.sv
TOP     := tb_eh2_pmp
OBJ_DIR := obj_dir

VER_FLAGS := --binary --sv --trace --top-module $(TOP) --Mdir $(OBJ_DIR) -Wno-TIMESCALEMOD

all: run

run: $(OBJ_DIR)/V$(TOP)
	./$(OBJ_DIR)/V$(TOP)

$(OBJ_DIR)/V$(TOP): $(RTL_SRC) $(TB_SRC)
	verilator $(VER_FLAGS) $(RTL_SRC) $(TB_SRC)
	$(MAKE) -C $(OBJ_DIR) -f V$(TOP).mk

clean:
	rm -rf $(OBJ_DIR)
	rm -f *.vcd

.PHONY: all run clean