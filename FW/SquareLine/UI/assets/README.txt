FUEL GAUGE - SQUARELINE ASSETS v2

This package matches the reference image more closely.

MAIN ASSET
segment_white_16x16.png
- 16x16 px
- transparent background
- white radial LED block
- intended for ALL 36 SquareLine Image widgets
- recolor at runtime with LVGL image recolor

Do NOT import the *_REFERENCE color files for the runtime gauge.
They are only visual references.

Recommended SquareLine:
Screen
  fuel_gauge (Group)
    img_segment_00
    img_segment_01
    ...
    img_segment_35
    img_fuel_icon
    label_fuel
    label_percent
    label_liters

COLOR THRESHOLDS
0-10%     RED
11-25%    ORANGE
26-50%    YELLOW
51-75%    BLUE
76-95%    TURQUOISE
96-100%   GREEN

For each update:
1. Calculate active_segments = percent * 36 / 100.
2. Hide all segments >= active_segments.
3. Recolor all visible segments with the SAME color selected from the
   threshold above.

The supplied 360 reference is only a positioning/reference image.
