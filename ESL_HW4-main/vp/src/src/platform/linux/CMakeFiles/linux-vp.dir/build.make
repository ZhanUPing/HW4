# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/ee6470/test/ESL_HW4-main/vp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/ee6470/test/ESL_HW4-main/vp/src

# Include any dependencies generated for this target.
include src/platform/linux/CMakeFiles/linux-vp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/platform/linux/CMakeFiles/linux-vp.dir/compiler_depend.make

# Include the progress variables for this target.
include src/platform/linux/CMakeFiles/linux-vp.dir/progress.make

# Include the compile flags for this target's objects.
include src/platform/linux/CMakeFiles/linux-vp.dir/flags.make

src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o: src/platform/linux/CMakeFiles/linux-vp.dir/flags.make
src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o: platform/linux/linux_main.cpp
src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o: src/platform/linux/CMakeFiles/linux-vp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/ee6470/test/ESL_HW4-main/vp/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o"
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o -MF CMakeFiles/linux-vp.dir/linux_main.cpp.o.d -o CMakeFiles/linux-vp.dir/linux_main.cpp.o -c /home/user/ee6470/test/ESL_HW4-main/vp/src/platform/linux/linux_main.cpp

src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linux-vp.dir/linux_main.cpp.i"
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/ee6470/test/ESL_HW4-main/vp/src/platform/linux/linux_main.cpp > CMakeFiles/linux-vp.dir/linux_main.cpp.i

src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linux-vp.dir/linux_main.cpp.s"
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/ee6470/test/ESL_HW4-main/vp/src/platform/linux/linux_main.cpp -o CMakeFiles/linux-vp.dir/linux_main.cpp.s

# Object files for target linux-vp
linux__vp_OBJECTS = \
"CMakeFiles/linux-vp.dir/linux_main.cpp.o"

# External object files for target linux-vp
linux__vp_EXTERNAL_OBJECTS =

bin/linux-vp: src/platform/linux/CMakeFiles/linux-vp.dir/linux_main.cpp.o
bin/linux-vp: src/platform/linux/CMakeFiles/linux-vp.dir/build.make
bin/linux-vp: src/core/rv64/librv64.a
bin/linux-vp: src/platform/common/libplatform-common.a
bin/linux-vp: src/core/common/gdb-mc/libgdb-mc.a
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_log.so.1.74.0
bin/linux-vp: systemc/src/libsystemc.a
bin/linux-vp: src/vendor/softfloat/libsoftfloat.a
bin/linux-vp: src/core/common/libcore-common.a
bin/linux-vp: systemc/src/libsystemc.a
bin/linux-vp: src/core/common/gdb-mc/libgdb/libgdb.a
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.74.0
bin/linux-vp: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.74.0
bin/linux-vp: src/platform/linux/CMakeFiles/linux-vp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/ee6470/test/ESL_HW4-main/vp/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/linux-vp"
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linux-vp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/platform/linux/CMakeFiles/linux-vp.dir/build: bin/linux-vp
.PHONY : src/platform/linux/CMakeFiles/linux-vp.dir/build

src/platform/linux/CMakeFiles/linux-vp.dir/clean:
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux && $(CMAKE_COMMAND) -P CMakeFiles/linux-vp.dir/cmake_clean.cmake
.PHONY : src/platform/linux/CMakeFiles/linux-vp.dir/clean

src/platform/linux/CMakeFiles/linux-vp.dir/depend:
	cd /home/user/ee6470/test/ESL_HW4-main/vp/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/ee6470/test/ESL_HW4-main/vp /home/user/ee6470/test/ESL_HW4-main/vp/src/platform/linux /home/user/ee6470/test/ESL_HW4-main/vp/src /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux /home/user/ee6470/test/ESL_HW4-main/vp/src/src/platform/linux/CMakeFiles/linux-vp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/platform/linux/CMakeFiles/linux-vp.dir/depend

