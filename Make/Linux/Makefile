MARE := ../../Util/SimRobot/Util/mare/Linux/bin/mare
CONFIG ?= Develop
CLANGINCLUDE = `for d in $(shell clang++ -c -v -E - </dev/null 2>&1 | sed -n -e '/here:/,/^End/{/here:/d; /^End/d; p}'); do [ -f $$d/xmmintrin.h ] && echo $$d; done | head -n1`

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG) clanginclude=$(CLANGINCLUDE)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG) clanginclude=$(CLANGINCLUDE)

all_dev:
	@$(MARE) config=Develop clanginclude=$(CLANGINCLUDE)

all_deb:
	@$(MARE) config=Debug clanginclude=$(CLANGINCLUDE)

all_rel:
	@$(MARE) config=Release clanginclude=$(CLANGINCLUDE)

nao_dev:
	@$(MARE) Nao config=Develop clanginclude=$(CLANGINCLUDE)

nao_deb:
	@$(MARE) Nao config=Debug clanginclude=$(CLANGINCLUDE)

nao_rel:
	@$(MARE) Nao config=Release clanginclude=$(CLANGINCLUDE)

sim_dev:
	@$(MARE) SimRobot config=Develop clanginclude=$(CLANGINCLUDE)

sim_deb:
	@$(MARE) SimRobot config=Debug clanginclude=$(CLANGINCLUDE)

sim_rel:
	@$(MARE) SimRobot config=Release clanginclude=$(CLANGINCLUDE)
