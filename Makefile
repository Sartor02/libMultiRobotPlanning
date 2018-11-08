build_dir=build

C_compiler=/usr/bin/clang
CXX_compiler=/usr/bin/clang++
#C_compiler=/usr/bin/gcc
#CXX_compiler=/usr/bin/g++

all: $(build_dir) $(build_dir)/CMakeLists.txt.copy
	$(MAKE) --no-print-directory -C $(build_dir)

$(build_dir):
	mkdir -p $(build_dir)

$(build_dir)/CMakeLists.txt.copy: CMakeLists.txt Makefile $(build_dir)
	cd $(build_dir) && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
		-DCMAKE_CXX_COMPILER=$(CXX_compiler) \
		-DCMAKE_C_COMPILER=$(C_compiler) ..
	cp CMakeLists.txt $(build_dir)/CMakeLists.txt.copy

clang-format:
	$(MAKE) --no-print-directory -C $(build_dir) clang-format

test:
	$(MAKE) --no-print-directory -C $(build_dir) run-test

clean:
	rm -rf $(build_dir) output.yaml input.txt
