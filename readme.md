# Planner Developer Tools (PDT)

This repository contains tools developed at ESP that aim to facilitate scientifically sound path planning research.

We're working on better documentation, but for now here's our internal documentation.

## Installation

### Dependencies

First install the dependencies of the tools.

#### OMPL

First install the dependencies of [OMPL](http://ompl.kavrakilab.org/):

```bash
sudo apt update
sudo apt upgrade
sudo apt install build-essential cmake
sudo apt install libboost-all-dev
sudo apt install libeigen3-dev
sudo apt install libode-dev # optional
```
Some of the PDT features require our OMPL fork. You could alternatively use the regular upstream version of OMPL but will not be able to use all the PDT features.

```bash
git clone git@github.com:robotic-esp/ompl.git
cd ompl
git fetch
git checkout main_esp
mkdir build && cd build
cmake ..
make -j 4
sudo make install
```
You can use `git branch` to check which branch you have checked out, if you are not on the `main_esp` branch you will need to disable some features in PDT below.

#### Boost

Our tools depend on the `thread` and `program_options` components of [Boost](https://www.boost.org/), which were installed as dependencies of OMPL above.

#### Pangolin

Follow the installation guide of the [official repo](https://github.com/stevenlovegrove/Pangolin), it's well documented.

You then need to install it. From within the Pangolin directory:
```bash
sudo cmake --install build
```

#### FFmpeg

FFmpeg is only needed if you want to record videos. The code should compile without it.

```bash
sudo apt install ffmpeg
```

#### LaTeX / LuaLaTeX

The automatic report generation relies on LuaLaTeX to dynamically allocate as much memory as needed. The infallible wisdom of the internet suggests that all major LaTeX distributions include LuaLaTeX, so I figured this is not too much of an additional dependency.

```bash
sudo apt install texlive-latex-base texlive-latex-extra texlive-luatex
```

If running `which lualatex` echoes a path, you should be good to go, otherwise install lualatex.

The reporting tool loads the whole `result.csv` into memory. This file can be quite large, depending on `logFrequency`, `maxTime`, and the number of planners. If your system runs out of memory, ensure you have swap turned on ([see, e.g., here on how to do it](https://tecadmin.net/enable-swap-on-ubuntu/)), or rerun experiment with a lower `logFrequency`, a lower `maxTime`, or fewer planners.

##### Visualization

LaTeX is also used by the visualizer to export frames, which requires a system installation of the Roboto font:
```bash
sudo apt install fonts-roboto
```

#### ImageMagick

The report generation uses ImageMagick to convert PDFs to PNGs. It is likely installed on your machine by default but can be installed with
```bash
sudo apt install imagemagick
```

If you get an error during report compilation related to "operation not allowed by the security policy `PDF'", then see you need to [change your GhostScript security policy](https://stackoverflow.com/questions/52998331/imagemagick-security-policy-pdf-blocking-conversion).

#### Doxygen and Graphviz

Doxygen and Graphviz are only necessary if you want to generate documentation. The code will compile without it.
```bash
sudo apt install doxygen graphviz xdot
```
The program `xdot` is not strictly necessary, but is a useful lightweight viewer for `.dot` files.


### Planner Developer Tools (PDT)

This project includes two header-only libraries (N. Lohmann's [JSON](https://github.com/nlohmann/json) and A. Fallah's [CSV-parser](https://github.com/AriaFallah/csv-parser)) as submodules (which is why we need to clone recursively).

```bash
git clone --recursive git@github.com:robotic-esp/pdt.git
cd pdt
mkdir build && cd build
cmake ..
make
```

If you are compiling against an upstream version of OMPL, you must add `-DPDT_BASE_OMPL_ONLY=TRUE` to the `cmake` call above.

If you have Doxygen installed, you can also build the documentation
```bash
make docs
```

## Reproducible research with Planner Developer Tools (PDT)

Most sampling-based planning algorithms depend on random numbers and can be configured by parameters (e.g., the connection radius for `RRT*`). Planning contexts depend on parameters as well (e.g, the number and shape of obstacles, the positions of the start and goal states, and the collision detection resolution). The result of an experiment, i.e., of a comparison of the performance of planners in a planning context, depends on the random number seed and all parameters. To reproduce an experiment, the seed and all parameters must be identical.

This is why our tools set all parameters explicitly and keep track of which parameters were set. But because explicitly specifying all parameters by hand for every experiment is tedious, the tools are shipped with a set of default parameters (you can inspect them in `pdt/parameters/defaults/`). The intended way to change a parameter is through a configuration patch. Parameters that are specified in such a patch can extend **and overwrite** the defaults.

Regardless of whether a parameter was specified in the defaults or through a patch, all accessed parameters are exported at the end of an experiment. This allows to use the exported configuration as a patch for a new experiment, which ensures that the experiment is run with the same random seed and set of parameters and therefore makes it reproducible.

Note that it is unreasonable to expect the exact same results because they depend on many aspects we cannot control, e.g., the current memory layout of the underlying system.

## Running Planner Developer Tools (PDT)

### Testing the installation

You can test your installation by running two demos. The first one runs a quick benchmark:

```bash
cd /path/to/pdt/build
./bin/benchmark -c ../parameters/demo/benchmark_demo.json
```

The above command has created the `build/benchmarks/<date-string>_<context-name>` directory and placed a couple of files in it, including a report that summarizes the results of the benchmark.

To test the visualization, you can run:

```bash
./bin/visualization -c ../parameters/demo/visualization_demo.json
```

Feel free to play around by substitution different contexts/planners into these `.json` files. Available options can be taken from the default config files in `pdt/parameters/defaults/`. The recommended way to change a default parameter is to **override** it by placing a parameter with the same name in the configuration provided with the `-c` option.

### Benchmark

The tools include an executable called `benchmark`. This executable should be invoked as `benchmark -c path/to/benchmark.json`, where `path/to/benchmark.json` points to a configuration patch. The configuration patch has to specify a family of `"Experiment"` parameters, such as the planners, the planning context, and the number of runs per planner. You can find an example of a suitable configuration patch at `pdt/parameters/demos/benchmark_demo.json`. Notice that you can specify two planners of the same type but different configurations by giving them different names.

All experiments get their own folders in `benchmarks/`. The naming convention for the folders is `<date-string>_<context-name>`. The corresponding folder contains various files when the experiment is finished. The most important are
- `config.json`: All accessed parameters for this experiment;
- `results.csv`: The raw data of the experiment;
- `report.pdf`: A summary of the results of the experiment.

### Visualization

At ESP we design new planning algorithms. To facilitate this, it is often helpful to visualize process of planning and not just the result. The executable `visualization` does exactly this. It is again invoked with a configuration patch, which specifies which planner and which context is to be visualized. You can invoke it as `visualization -c path/to/visualization.json`.

Implementation detail: The `maxTime` parameter of all planning contexts is ignored and a planner is allowed to run indefinitely when visualized. The current implementation tries to buffer iterations ahead of the currently visualized iteration. **It does not perform any checks on available memory, so keep your available memory in sight as your system might freeze.**

The visualization is interactive. These are the five keys that are currently bound to some action:

- 'f': forward one iteration
- 'b': backward one iteration
- 'F': forward 10% of largest computed iteration
- 'B': backward 10% of largest computed iteration
- ' ': Toggle tracking

When tracking is on, the most recently computed iteration is visualized.

# Troubleshooting

## The compilation fails because some headers are not found

You have built and installed the ESP version of OMPL, but the compilation fails because it is missing headers? Make sure the correct OMPL is found (`cmake` will report the path of the OMPL it finds). Say you installed the ESP version of OMPL to `/usr/local/`, but also have ROS installed on your system and `cmake` prefers the ROS version. You can specify the path PDT looks for OMPL in the variable `PDT_OMPL_DIR`, e.g., by running `cmake -DPDT_OMPL_DIR=/usr/local`. If you don't define the variable, CMake will look in the standard places.

## Executing a program complains about missing symbols

This can happen if the wrong version of OMPL is dynamically linked to your executable. I don't have an elegant solution to this problem. A quick and dirty hack is to wrap your executable, e.g., `build/bin/benchmark` into a script and modify `LD_LIBRARY_PATH` in that script:

```bash
#!/bin/bash
export LD_LIBRARY_PATH="/usr/local/lib/:${LD_LIBRARY_PATH}"
/path/to/pdt/build/bin/benchmark -c /path/to/pdt/parameters/demo/benchmark_demo.json
```

## The benchmark report doesn't compile

Turn on the verbose compilation in the benchmark configuration `.json` file. If the error is related to LuaTeX, e.g., `! LaTeX Error: file 'luatex85.sty' not found.`, make sure LuaTeX is installed on your system. An easy way to install it in Ubuntu is `sudo apt install texlive-luatex`.

# Miscellaneous

## Formatting code

We format our code on a style based on [Google's style guide](https://google.github.io/styleguide/cppguide.html). A good way to ensure that your contribution conforms to this style is to run `clang-format` on it before committing. You can format a specific file with

```bash
clang-format -i ./path/to/file
```

or you can use

```bash
git clang-format -f
```

to format all lines in all files that changed between the current working directory and the last commit. You can likely also configure your editor to run `clang-format` for you, e.g., whenever you save a C++ file.

## Statically analysing code

Please consider using static analysis tools, such as `clang-tidy`, when working in PDT. On Ubuntu `clang-tidy` can be installed from the repos (`sudo apt install clang-tidy`). Good editors can be configured to automatically run `clang-tidy` and visualize its output directly in your code. We recommend this workflow, as it doesn't require you to remember to run `clang-tidy`.

Alternatively, you can also run `clang-tidy` from the terminal. You can use it out of the box on a single file, but this requires you to specify the included directories, linked libraries, and compiler flags on the command line. For example

```bash
clang-tidy test.cpp -- -Imy_project/include -DMY_DEFINES ...
```

You could run `clang-tidy` like this on all files in the project, but you would have to find the correct includes, libraries, and flags for each file.

A less cumbersome way to run `clang-tidy` on a whole project is to let your build system generate a compile commands database for you and run `clang-tidy` from the folder in which this file is generated. In our case, you can tell CMake to generate this database by adding `-DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE` to your call to `cmake`. You can then run `clang-tidy` on all files in the project by invoking `run-clang-tidy` from the folder in which the database is generated (CMake calls this file `compile_commands.json` and it is typically generated in the `build` folder).

```bash
cd /path/to/pdt/build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=TRUE .. 
run-clang-tidy
```

The `run-clang-tidy` executable is included in the `clang-tidy` package installed with `sudo apt install clang-tidy`.

## Old Notes

## How to create maps from `.png`s

To create a map from a `.png` follow these steps:
   1. Open the `.png` in GIMP
   2. Remove shadows if necessary `Layer -> Transparency -> Threshold Alpha`
   3. Convert to B&W `Colors -> Brightness/Contrast`
   4. Make the background white `Layer -> Transparency -> Remove Alpha Channel`
   5. Thicken the obstacle if necessary e.g., via
     1. `Select -> By Color`
     2. `Select -> Grow`
     3. `Edit -> Fill`
   6. Convert to a binary image `Image -> Mode -> Indexed -> Black & White Palette`
   7. Save as `.bmp`
   8. Open `.bmp` in MATLAB
   9. Export to a csv file `csvwrite('filename.csv', bmpFileName)`
   10. Use the csvObstacle class (currently not implemented)

## Create a regression class of a planner

This allows for testing different versions of the algorithm (e.g., BIT*) in the same executable.

```bash
cd ompl/geometric/planners/
cp -r bitstar bitstar_regression
cd bitstar_regression
mv BITstar.h BITstarRegression.h
mv src/BITstar.cpp src/BITstarRegression.cpp
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' BITstarRegression.h
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' src/BITstarRegression.cpp
cd datastructures
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' *.h
cd src
sed -i.bak -e 's|_BITSTAR|_BITSTARREGRESSION|g' -e 's|/bitstar|/bitstar_regression|g' -e 's|BITstar|BITstarRegression|g' *.cpp
```
