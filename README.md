# smc-lma-EI-Spring2020
Some implementations of computational Laban Movement Analysis (LMA)

- OpenPose + p5.js: Juan Alonso's library lma5 https://github.com/juanalonso/lma5 (only 7.5 kB in size!)
  Online Demos:
  - [lma5 - PoseNet example](https://neuronasmuertas.com/lma5/examples/posenet)
  - [Nighstcape](https://neuronasmuertas.com/lma5/examples/nightscape.html) A simple echosystem that reacts to user's movement.

- OpenFrameworks: C++ real-time implementation that fetches Leap Motion Data (smoothed or unsmoothed), computes low-level kinematic descriptors (velocity, acceleration, jerk) and Laban Effort descriptors (Weight, Time, Flow, Space) for 12 joints (5 fingertips and palm position for both hands) and sends them via OSC. Dependencies: Leap Motion Orion Beta 4.0.0 (LeapC API), OpenFrameworks 0.11.0. By Francesco Bigoni.

- Processing:  Movement driven ambient sounds for relaxation enhancement and feedback during a movement exercise, by Lars Schalkwijk, 2016. Note: the data folder removed so it may not work in Processing directly.

- Python1 : implementation that fetches movement data from tsv file, computes low-level kinematic (velocity, acceleration, jerk), dynamic (curvature, Quantity of Motion) and geometric (Center of Mass) descriptors and Laban Effort descriptors (Weight, Time, Flow, Space). By Sophus Béneé Olsen

- Python2: implementation that fetches movement data from tsv file, computes low-level kinematic descriptors (velocity, acceleration, jerk) that are compared interactively with measurements, then computes Laban Effort descriptors (Weight, Time, Flow, Space). By Juan Alonso Moreno

- Unity1: Kinect-based implementation in Unity C#. By Eoin Rafferty, 2019.

- Max/MSP: Check the fantastic GIMLet https://github.com/federicoVisi/GIMLeT by [Federico Visi](http://www.federicovisi.com/)

All implementations are based on the review contained in C. Larboulette and S. Gibet, "A review of computable expressive descriptors of human motion," in *Proceedings of the 2nd International Workshop on Movement and Computing*, 2015, pp. 21–28, https://doi.org/10.1145/2790994.2790998.
