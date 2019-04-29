# ESP OMPL TOOLS

This repository contains our inhouse tools developed around OMPL.

## Mission

ESP's OMPL tools aim to facilitate scientifically sound path planning research.

## Features

- [x] Reproducible experiments
- [x] Automatic plot generation
- [x] Interactive visualization of planner progress
- [x] Videos of planner progress
- [x] Easy to define experiments (obstacles and antiobstacles)
- [ ] Statistical analysis
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

The executable `benchmark_report` compiles a latex report of a benchmark. Initially this was done with `pdflatex`, but this has a fixed memory size (which can be too small for large experiments). A quick fix was to compile the reports with `lualatex`, which dynamically allocates as much memory as needed. The unfallible wisdom of the internet suggests that all major latex distributions include lualatex, so I figured this is not too much of an additional dependency.

### ESP OMPL TOOLS

This project includes two header-only libraries (N. Lohmann's [JSON](https://github.com/nlohmann/json) and A. Fallah's [CSV-parser](https://github.com/AriaFallah/csv-parser)) as submodules (which is why we need to clone recursively).

```bash
git clone --recursive git@github.com:robotic-esp/esp_ompl_tools.git
cd esp_ompl_tools
mkdir build && cd build
cmake ..
make
```

## Running ESP OMPL TOOLS

You can test your installation with the following commands. The first one runs a quick benchmark:

```bash
cd /path/to/esp_ompl_tools/build
./bin/benchmark -c bin/parameters/executables/benchmark.json
```

The above command has created the `build/benchmark_logs/<date-string>_<context-name>` directory and placed a `config.json` file, a `results.csv` file, and a `log.txt` file into it. You can generate a report by running:

```bash
./bin/report -c benchmark_logs/<date-string>_<context-name>/config.json
```

To test the visualization, you can run:

```bash
./bin/visualization -c bin/parameters/executables/visualization.json
```

Feel free to play around by substitution different contexts/planners into these `.json` files. Available options can be taken from the default config files in `esp_ompl_tools/parameters/defaults/`.

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
