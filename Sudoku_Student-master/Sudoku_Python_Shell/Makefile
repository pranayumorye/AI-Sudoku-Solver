# ======================================================================
# FILE:        Makefile
#
# AUTHOR:      Abdullah Younis
#
# DESCRIPTION: This file contains useful commands for this project. You
#              may use the following commands:
#
#              - make            - compiles the project and places
#                                  the executable in the bin folder
#
#              - make submission - creates the the submission, you will
#                                  submit.
#
#              - Don't make changes to this file.
# ======================================================================

RAW_SOURCES = \
	Main.py\
	BTSolver.py\
	Constraint.py\
	ConstraintNetwork.py\
	Domain.py\
	SudokuBoard.py\
	Trail.py\
	Variable.py

SOURCE_DIR = src
BIN_DIR = bin
SOURCES = $(foreach s, $(RAW_SOURCES), $(SOURCE_DIR)/$(s))

all: $(SOURCES)
	@-eval `'/usr/bin/tclsh' '/pkg/modules/modulecmd.tcl' 'bash' load python/3.5.2` ; \
	 if hash python3 &> /dev/null ; \
	 then \
		python3 -m compileall -q src ; \
	 else \
		python -m compileall -q src ; \
	 fi
	@rm -rf $(BIN_DIR)
	@mkdir -p $(BIN_DIR)
	@cp -a src/__pycache__/. bin
	@for file in bin/* ; \
	do \
		mv -f $$file $${file%%.*}.pyc; \
	done
	@rm -rf src/__pycache__/

submission: all
	@rm -f *.zip
	@echo ""
	@read -p "Enter Team Name (No spaces, '_', '/', '*'): " teamName; \
	 echo ""; \
	 zip -rqq s_$${teamName}.zip $(SOURCE_DIR) $(BIN_DIR)

run:
	java -jar bin/Sudoku.jar
