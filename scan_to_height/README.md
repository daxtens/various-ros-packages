`scan_to_height`
================

Splits a scan into downwards reflected points, dropped points, and a clean scan.

Refer to the relevant section of the report for details.

In demo, `height` and `heightbag` provide launch files for measuring the height from live data and a data bag. `heightbag` expects to find the bag in `laser_hover/demo/mounted.bag` for historical reasons. 

Height is published to `/height` (10 pt moving average) and `/height_raw`. This can't be visualised in RViz, so consider `rostopic echo /height`.
