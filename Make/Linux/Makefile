MARE := ../../Util/SimRobot/Util/mare/Linux/bin/mare
CONFIG ?= Develop

.PHONY: all clean

all:
	@$(MARE) config=$(CONFIG)

clean:
	@$(MARE) clean config=$(CONFIG)

%:
	@$(MARE) $@ config=$(CONFIG)

all_dev:
	@$(MARE) config=Develop

all_deb:
	@$(MARE) config=Debug

all_rel:
	@$(MARE) config=Release

nao_dev:
	@$(MARE) Nao config=Develop

nao_deb:
	@$(MARE) Nao config=Debug

nao_rel:
	@$(MARE) Nao config=Release

sim_dev:
	@$(MARE) SimRobot config=Develop

sim_deb:
	@$(MARE) SimRobot config=Debug

sim_rel:
	@$(MARE) SimRobot config=Release
