# ESP OMPL TOOLS

This repository contains our inhouse tools developed around OMPL.

## Mission

ESP's OMPL tools aim to facilitate scientifically sound path planning research.

## Features

- [x] Reproducible experiments
- [x] Automatic report generation
- [x] Interactive visualization of planner progress
- [x] Videos of planner progress
- [x] Easy to define experiments (obstacles and antiobstacles)
- [x] Statistical analysis
- [ ] Automatic stopping based on p-values
- [ ] Guidance on number of runs
- [ ] Reporting of p-values
- [ ] Large automated runs
- [ ] Physically founded generated experiments (remove bespoke problems from literature)

## Installation

### Dependencies

#### OMPL

We currently depend on our own version of OMPL, because SBIT* hasn't been published yet. This should work (its probably best to uninstall any other versions of OMPL first):

```bash
git clone git@github.com:robotic-esp/ompl-private.git
cd ompl-private
git fetch
git checkout marlin/abitstar
mkdir build && cd build
cmake ..
make
sudo make install
```

Make sure you are on the `marlin/abitstar` branch.

#### Boost

Our tools depend on the `thread` and `program_options` components of Boost:

```bash
sudo apt update
sudo apt install libboost-thread-dev libboost-program-options-dev
```

But you can probably afford to install all boost libraries:

```bash
sudo apt update
sudo apt install libboost-all-dev
```

#### FFmpeg

FFmpeg is only needed if you want to record videos. The code should compile without it.

```bash
sudo apt install ffmpeg
```

#### Pangolin

I've had to adapt the default paths in which Pangolin looks for `ffmpeg`. I opened a PR ([#498](https://github.com/stevenlovegrove/Pangolin/pull/498)) on the official repo, but it hasn't been approved yet. My suggestion is to just clone the official repo and install that. If it doesn't work look at the PR above and implement the changes yourself (it's literally 10 lines of code that need to be changed). The official repo lives [here](https://github.com/stevenlovegrove/Pangolin), please follow its installation guide, it's well documented.

#### LaTeX / LuaLaTeX

The executable `benchmark_report` compiles a latex report of a benchmark. Initially this was done using `pdflatex`, which allocates a fixed amount of memory for each compilation. For very large experiments, `pdflatex` allocated too little memory, which resulted in an error. A quick fix was to compile the reports with `lualatex` instead. LuaLaTeX dynamically allocates as much memory as needed. The unfallible wisdom of the internet suggests that all major LaTeX distributions include LuaLaTeX, so I figured this is not too much of an additional dependency.

You can run

```bash
which lualatex
```

to see wether you have it installed on your system.

### ESP OMPL TOOLS

This project includes two header-only libraries (N. Lohmann's [JSON](https://github.com/nlohmann/json) and A. Fallah's [CSV-parser](https://github.com/AriaFallah/csv-parser)) as submodules (which is why we need to clone recursively).

```bash
git clone --recursive git@github.com:robotic-esp/esp_ompl_tools.git
cd esp_ompl_tools
mkdir build && cd build
cmake ..
make
```

## Reproducible research with ESP OMPL TOOLS

Virtually all sampling-based planning algorithms can be configured by setting parameters (e.g., the connection radius for `RRT*`), or disabling parts of the algorithms (e.g, graph-pruning for `BIT*`). Planning contexts depend on parameters as well (e.g, the number of obstacles and the positions of the start and goal states). The result of an experiment, i.e., of a comparison of the performance of planners in a planning context, depends heavily on these parameters and to reproduce such a result all parameters must be known.

This is why our tools set all parameters explicitly and keep track of which parameters were set. But because explicitly specifying all parameters by hand for every experiment is tedious, the tools are shipped with a set of default parameters (you can inspect them in `esp_ompl_tools/parameters/defaults/`). The intended way to change a parameter is through a configuration patch. Parameters that are specified in such a patch can extend **and overwrite** the defaults.

Regardless of whether a parameter was specified in the defaults or through a patch, all accessed parameters are exported at the end of an experiment. This allows to use the exported configuration as a patch for a new experiment, which ensures that the experiment is run with the exact set of parameters and therefore makes it reproducible. When using an exported parameter set of a previous experiment, no default parameters are loaded.

## Running ESP OMPL TOOLS

### Testing the installation

You can test your installation with the following commands. The first one runs a quick benchmark:

```bash
cd /path/to/esp_ompl_tools/build
./bin/benchmark -c bin/parameters/executables/benchmark.json
```

The above command has created the `build/benchmarks/<date-string>_<context-name>` directory and placed a `config.json` file, a `results.csv` file, and a `log.txt` file into it. You can generate a report by running:

```bash
./bin/benchmark_report -c benchmarks/<date-string>_<context-name>/config.json
```

To test the visualization, you can run:

```bash
./bin/visualization -c bin/parameters/executables/visualization.json
```

Feel free to play around by substitution different contexts/planners into these `.json` files. Available options can be taken from the default config files in `esp_ompl_tools/parameters/defaults/`. The recommended way to change a default parameter is to **override** it by placing a parameter with the same name in the configuration provided with the `-c` option.

### Benchmark

The tools include an executable called `benchmark`. This executable should be invoked as `benchmark -c path/to/benchmark.json`, where `path/to/benchmark.json` points to a configuration patch. The configuration patch has to specify a family of `"Experiment"` parameters, such as the planners, the planning context, and the number of runs per planner. You can find an example of a suitable configuration patch at `esp_ompl_tools/parameters/executables/benchmark.json`. Notice that you can specify two planners of the same type but different configurations by giving them different names.

All experiments get their own folders in `benchmarks/`. The naming convention for the folders is `<date-string>_<context-name>`. The corresponding folder contains three files when the experiment is finished:
- `config.json`: All accessed parameters for this experiment;
- `results.csv`: The raw data of the experiment;
- `log.txt`: Some additional information that is not required to reproduce the results of the experiment.

### Benchmark report

The `benchmark_report` generates a LaTeX document that presents some statistics and plots of an experiment. It should be invoked as `benchmark_report -c path/to/benchmarks/<date-string>_<context-name>/config.json`.

### Visualization

At ESP we design new planning algorithms. To facilitate this, it is often helpful to visualize process of planning and not just the result. The executable `visualization` does exactly this. It is again invoked with a configuration patch, which specifies which planner and which context is to be visualized. You can invoke it as `visualization -c path/to/visualization.json`.

**Important implementation detail**: The `maxTime` parameter of all planning contexts is ignored and a planner is allowed to run indefinitely when visualized. The current implementation tries to buffer 5000 iterations ahead of the currently visualized iteration. It does not perform any checks on available memory.

The visualization is interactive. These are the five keys that are currently bound to some action:

- 'f': forward one iteration
- 'b': backward one iteration
- 'F': forward 10% of largest computed iteration
- 'B': backward 10% of largest computed iteration
- ' ': Toggle tracking

When tracking is on, the most recently computed iteration is visualized.

## Why OMPL?

- Already well adopted in field (helps our mission)
- Major changes in OMPL base classes needed
- Mark & Lydia have release experience
- Mark & Lydia can help with reasonable problem definitions
- Possible integration with PlannerArena
- Possible integration in core OMPL

## Backlog
- [ ] Implement feature to record/play at constant 'iteration per second' rate
- [ ] Implement feature to export every iteration as png
- [ ] Improve quality of recorded videos
- [ ] Implement "pageing" mechanism for planner visualization

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
   10. Use the csvObstacle class

## Create a regression class of BIT*

This allows for testing different versions of the algorithm in the same executable.

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