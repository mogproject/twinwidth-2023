RM = /bin/rm
MV = /bin/mv
CP = /bin/cp
PYTHON=python3
MKDIR=mkdir
GCOV=gcov
LCOV=lcov
GENHTML=genhtml
TAR=gtar
EXACT_SOLVER_BIN=exact-solver

ifneq ("$(wildcard /opt/homebrew/bin/g++-12)","")
export CC=/opt/homebrew/bin/gcc-12
export CXX=/opt/homebrew/bin/g++-12
GCOV=/opt/homebrew/bin/gcov-12
else
ifneq ("$(wildcard /opt/homebrew/bin/g++-11)","")
export CC=/opt/homebrew/bin/gcc-11
export CXX=/opt/homebrew/bin/g++-11
GCOV=/opt/homebrew/bin/gcov-11
endif
endif

CMAKE=cmake
CMAKE_OPTS=-DCMAKE_C_COMPILER=$(CC) -DCMAKE_CXX_COMPILER=$(CXX)

SRC_CPP=src/main/cpp
PROJ_DIR=$(PWD)
BUILD_DIR=$(PROJ_DIR)/build
DIST_DIR=$(PROJ_DIR)/dist
TEST_BIN_DIR=$(BUILD_DIR)/test
TEST_EXEC=$(TEST_BIN_DIR)/tww_test
COV_CPP_DIR=coverage/cpp
COV_CPP=$(COV_CPP_DIR)/lcov.info
COV_PY_DIR=coverage/py
COV_PY=$(COV_PY_DIR)/lcov.info
COV_HTML=coverage/html
COV_MERGED=coverage/lcov.info


build:
	cd $(SRC_CPP) && $(CMAKE) -S . -B "$(BUILD_DIR)/Release" $(CMAKE_OPTS) -DCMAKE_BUILD_TYPE=Release
	cd $(SRC_CPP) && $(CMAKE) --build "$(BUILD_DIR)/Release"
	$(CP) -f build/Release/$(EXACT_SOLVER_BIN) dist/
	@echo "Created: dist/$(EXACT_SOLVER_BIN)"

clean:
	@echo "Cleaning..."
	@$(RM) -rf build/* dist/*
	@echo "Cleaning done."

publish:
	@$(MKDIR) -p dist
	cd $(SRC_CPP) && $(TAR) -zcvf $(DIST_DIR)/$(EXACT_SOLVER_BIN).tgz --exclude-from=$(PROJ_DIR)/.tarignore *
	@echo "Created: dist/$(EXACT_SOLVER_BIN).tgz"

.PHONY: build clean publish

