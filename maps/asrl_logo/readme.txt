Open the source png in GIMP
Go Layer -> Transparency -> Threshold Alpha to remove the shadows
Go Colors -> Brightness/Contrast to turn into a B&W image
Go Layer -> Transparency -> Remove Alpha Channel to make the background white
(Optional - Thicken the obstacles)
  Go Select -> By Color and select the black
  Go Select -> Grow and grow the selection by 1 pixel
  Go Edit -> Fill with FG color
Go Image -> Mode -> Indexed and select the Black & White Palette to turn into a binary imate
Save as a BMP
Open in MATLAB
Export to a csv with: csvwrite('asrl.csv', cdata)