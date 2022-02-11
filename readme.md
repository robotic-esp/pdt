# ESP OMPL TOOLS

This repository contains tools developed at ESP that aim to facilitate scientifically sound path planning research.

## Installation

### Dependencies

First install the dependencies of the tools.

#### OMPL

First install the dependencies of [OMPL](http://ompl.kavrakilab.org/):

```bash
sudo apt update
sudo apt upgrade
sudo apt install cmake
sudo apt install libboost-all-dev
sudo apt install libeigen3-dev
sudo apt install libode-dev # optional
```
We want to test our own planners, which are developed in our private OMPL fork:

```bash
git clone git@github.com:robotic-esp/ompl-private.git
cd ompl-private
git fetch
git checkout esp-master
mkdir build && cd build
cmake ..
make -j 4
sudo make install
```

Make sure you are on the `esp-master` branch. You can use `git branch` to check which branch you have checked out.

#### Boost

Our tools depend on the `thread` and `program_options` components of [Boost](https://www.boost.org/), which were installed as dependencies of OMPL above.

#### Pangolin

Follow the installation guide of the [official repo](https://github.com/stevenlovegrove/Pangolin), it's well documented.

**Note**: Pangolin got refactored in early 2021, but our tools haven't been updated yet. The latest stable version of Pangolin that our tools compile with is v0.6. Once you have cloned the Pangolin repo, cd into it and run

```bash
git checkout v0.6
```

before building and installing.

#### FFmpeg

FFmpeg is only needed if you want to record videos. The code should compile without it.

```bash
sudo apt install ffmpeg
```

#### LaTeX / LuaLaTeX

The automatic report generation relies on LuaLaTeX to dynamically allocate as much memory as needed. The infallible wisdom of the internet suggests that all major LaTeX distributions include LuaLaTeX, so I figured this is not too much of an additional dependency.

```bash
sudo apt install texlive-latex-base texlive-luatex
```

If running `which lualatex` echoes a path, you should be good to go, otherwise install lualatex.

The reporting tool loads the whole `result.csv` into memory. This file can be quite large, depending on `logFrequency`, `maxTime`, and the number of planners. If your system runs out of memory, ensure you have swap turned on ([see, e.g., here on how to do it](https://tecadmin.net/enable-swap-on-ubuntu/)), or rerun experiment with a lower `logFrequency`, a lower `maxTime`, or fewer planners.

#### ImageMagick

The report generation uses ImageMagick to convert PDFs to PNGs. It is likely installed on your machine by default but can be installed with
```bash
sudo apt install imagemagick
```

If you get an error during report compilation related to "operation not allowed by the security policy `PDF'", then see you need to [change your GhostScript security policy](https://stackoverflow.com/questions/52998331/imagemagick-security-policy-pdf-blocking-conversion).


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

Most sampling-based planning algorithms depend on random numbers and can be configured by parameters (e.g., the connection radius for `RRT*`). Planning contexts depend on parameters as well (e.g, the number and shape of obstacles, the positions of the start and goal states, and the collision detection resolution). The result of an experiment, i.e., of a comparison of the performance of planners in a planning context, depends on the random number seed and all parameters. To reproduce an experiment, the seed and all parameters must be identical.

This is why our tools set all parameters explicitly and keep track of which parameters were set. But because explicitly specifying all parameters by hand for every experiment is tedious, the tools are shipped with a set of default parameters (you can inspect them in `esp_ompl_tools/parameters/defaults/`). The intended way to change a parameter is through a configuration patch. Parameters that are specified in such a patch can extend **and overwrite** the defaults.

Regardless of whether a parameter was specified in the defaults or through a patch, all accessed parameters are exported at the end of an experiment. This allows to use the exported configuration as a patch for a new experiment, which ensures that the experiment is run with the same random seed and set of parameters and therefore makes it reproducible.

Note that it is unreasonable to expect the exact same results because they depend on many aspects we cannot control, e.g., the current memory layout of the underlying system.

## Running ESP OMPL TOOLS

### Testing the installation

You can test your installation by running two demos. The first one runs a quick benchmark:

```bash
cd /path/to/esp_ompl_tools/build
./bin/benchmark -c ../parameters/demo/benchmark_demo.json
```

The above command has created the `build/benchmarks/<date-string>_<context-name>` directory and placed a couple of files in it, including a report that summarizes the results of the benchmark.

To test the visualization, you can run:

```bash
./bin/visualization -c ../parameters/demo/visualization_demo.json
```

Feel free to play around by substitution different contexts/planners into these `.json` files. Available options can be taken from the default config files in `esp_ompl_tools/parameters/defaults/`. The recommended way to change a default parameter is to **override** it by placing a parameter with the same name in the configuration provided with the `-c` option.

### Benchmark

The tools include an executable called `benchmark`. This executable should be invoked as `benchmark -c path/to/benchmark.json`, where `path/to/benchmark.json` points to a configuration patch. The configuration patch has to specify a family of `"Experiment"` parameters, such as the planners, the planning context, and the number of runs per planner. You can find an example of a suitable configuration patch at `esp_ompl_tools/parameters/demos/benchmark_demo.json`. Notice that you can specify two planners of the same type but different configurations by giving them different names.

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

# Miscellaneous

## Why OMPL?

- Already well adopted in field (helps our mission)
- Major changes in OMPL base classes needed
- Mark & Lydia have release experience
- Mark & Lydia can help with reasonable problem definitions
- Possible integration with PlannerArena
- Possible integration in core OMPL

## Backlog

### ESP OMPL TOOLS

- [ ] Improve visualization tool.

### OMPL

- [ ] Add method to scale costs. When the objective is a minimization (e.g., of path length) scaling a cost means multiplying it with the scaling factor, but when the objective is a maximization (e.g., of minimum clearance) scaling a cost means dividing it by the scaling factor.
- [ ] Add a method to generate (possibly inadmissible) estimates of motion and state costs.

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
