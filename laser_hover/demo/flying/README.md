Notes on the .csv files
======================

 * They're very raw data. To be precise, they are the raw scan data: there's no height processing or even filtering of close points. In order to make use of the data, you'll need to split out the height scan and filter the close points. Then you can process it.

 * Both files are timed so that they start just as it's starting to pick up height, and finish after it lands. You'll probably want to drop the first little bit (maybe until you hit 1m of height, as tree_fix does) and you'll definitely want to drop the last bit as it lands.

 * Specs:

    angle_min: -2.35619449615 (rads)
    angle_max: 2.35619449615 (rads)
    angle_increment: 0.00436332309619 (rads)

