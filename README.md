# smc-lma-EI-Spring2020
Some implementations of computational Laban Movement Analysis (LMA)

- OpenFrameworks: C++ real-time implementation that fetches Leap Motion Data (smoothed or unsmoothed), computes low-level kinematic descriptors (velocity, acceleration, jerk) and Laban Effort descriptors (Weight, Time, Flow, Space) for 12 joints (5 fingertips and palm position for both hands) and sends them via OSC. Dependencies: Leap Motion Orion Beta 4.0.0 (LeapC API), OpenFrameworks 0.11.0. By Francesco Bigoni.

- Processing:  Movement driven ambient sounds for relaxation enhancement and feedback during a movement exercise, by Lars Schalkwijk, 2016. Note: the data folder removed so it may not work in Processing directly.

- Python1 : implementation that fetches movement data from tsv file, computes low-level kinematic (velocity, acceleration, jerk), dynamic (curvature, Quantity of Motion) and geometric (Center of Mass) descriptors and Laban Effort descriptors (Weight, Time, Flow, Space). By Sophus Béneé Olsen

- Python2: implementation that fetches movement data from tsv file, computes low-level kinematic descriptors (velocity, acceleration, jerk) that are compared interactively with measurements, then computes Laban Effort descriptors (Weight, Time, Flow, Space). By Juan Alonso Moreno

- Unity1: Kinect-based implementation in Unity C#. By Eoin Rafferty, 2019.

All implementations are based on the review contained in C. Larboulette and S. Gibet, "A review of computable expressive descriptors of human motion," in *Proceedings of the 2nd International Workshop on Movement and Computing*, 2015, pp. 21–28, https://doi.org/10.1145/2790994.2790998.
