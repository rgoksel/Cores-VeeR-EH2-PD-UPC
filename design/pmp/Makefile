RTL_DIR := rtl
TB_DIR  := tb
OBJ_DIR := obj_dir

VERILATOR := verilator
GTKWAVE   := gtkwave

TOP := tb_eh2_pmp

# AS BAYRAKLARI
VER_FLAGS := --binary --sv --trace \
             -Wall -Wno-UNUSED -Wno-UNDRIVEN \
			 -Wno-UNSIGNED \
             -I$(RTL_DIR) \
             --Mdir $(OBJ_DIR) \
             --top-module $(TOP)

.PHONY: all run wave clean

all: run

run: $(OBJ_DIR)/V$(TOP)
	./$(OBJ_DIR)/V$(TOP)

$(OBJ_DIR)/V$(TOP): $(RTL_DIR)/eh2_pmp.sv $(TB_DIR)/tb_eh2_pmp.sv
	$(VERILATOR) $(VER_FLAGS) \
		$(RTL_DIR)/eh2_pmp.sv \
		$(TB_DIR)/tb_eh2_pmp.sv

wave: $(TOP).vcd
	$(GTKWAVE) $(TOP).vcd &

clean:
	rm -rf $(OBJ_DIR)
	rm -f *.vcd