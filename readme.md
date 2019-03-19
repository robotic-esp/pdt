# ESP OMPL TOOLS

This repository contains our inhouse tools developed around OMPL.

## Maps

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
