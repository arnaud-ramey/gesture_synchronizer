
  1. Generate the video:
     $ gtk-recordmydesktop
  2. Convert to PNG:
     $ ffmpeg -i out.ogv -r 15 $filename%03d.png
  3. Convert batch of PNG to GIF:
     $ convert *.png output.gif
  4. Optimize
     $ convert output.gif -fuzz 10% -layers Optimize optimised.gif
