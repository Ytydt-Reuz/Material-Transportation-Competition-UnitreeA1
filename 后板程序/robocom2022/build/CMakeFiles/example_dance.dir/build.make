# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/unitree/robocom2022

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/robocom2022/build

# Include any dependencies generated for this target.
include CMakeFiles/example_dance.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_dance.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_dance.dir/flags.make

CMakeFiles/example_dance.dir/examples/example_dance.cpp.o: CMakeFiles/example_dance.dir/flags.make
CMakeFiles/example_dance.dir/examples/example_dance.cpp.o: ../examples/example_dance.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/robocom2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_dance.dir/examples/example_dance.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_dance.dir/examples/example_dance.cpp.o -c /home/unitree/robocom2022/examples/example_dance.cpp

CMakeFiles/example_dance.dir/examples/example_dance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_dance.dir/examples/example_dance.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/robocom2022/examples/example_dance.cpp > CMakeFiles/example_dance.dir/examples/example_dance.cpp.i

CMakeFiles/example_dance.dir/examples/example_dance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_dance.dir/examples/example_dance.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/robocom2022/examples/example_dance.cpp -o CMakeFiles/example_dance.dir/examples/example_dance.cpp.s

CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.requires:

.PHONY : CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.requires

CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.provides: CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.requires
	$(MAKE) -f CMakeFiles/example_dance.dir/build.make CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.provides.build
.PHONY : CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.provides

CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.provides.build: CMakeFiles/example_dance.dir/examples/example_dance.cpp.o


# Object files for target example_dance
example_dance_OBJECTS = \
"CMakeFiles/example_dance.dir/examples/example_dance.cpp.o"

# External object files for target example_dance
example_dance_EXTERNAL_OBJECTS =

example_dance: CMakeFiles/example_dance.dir/examples/example_dance.cpp.o
example_dance: CMakeFiles/example_dance.dir/build.make
example_dance: CMakeFiles/example_dance.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/robocom2022/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_dance"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_dance.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_dance.dir/build: example_dance

.PHONY : CMakeFiles/example_dance.dir/build

CMakeFiles/example_dance.dir/requires: CMakeFiles/example_dance.dir/examples/example_dance.cpp.o.requires

.PHONY : CMakeFiles/example_dance.dir/requires

CMakeFiles/example_dance.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_dance.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_dance.dir/clean

CMakeFiles/example_dance.dir/depend:
	cd /home/unitree/robocom2022/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/robocom2022 /home/unitree/robocom2022 /home/unitree/robocom2022/build /home/unitree/robocom2022/build /home/unitree/robocom2022/build/CMakeFiles/example_dance.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_dance.dir/depend

