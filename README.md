# smc-lma-EI-Spring2020
Some implementations of computational Laban Movement Analysis (LMA)

- Francesco Bigoni: C++ real-time implementation that fetches Leap Motion Data (smoothed or unsmoothed), computes low-level kinematic descriptors (velocity, acceleration, jerk) and Laban Effort descriptors (Weight, Time, Flow, Space) for 12 joints (5 fingertips and palm position for both hands) and sends them via OSC. Dependencies: Leap Motion Orion Beta 4.0.0 (LeapC API), OpenFrameworks 0.11.0.
- Sophus Béneé Olsen: Python implementation that fetches movement data from tsv file, computes low-level kinematic (velocity, acceleration, jerk), dynamic (curvature, Quantity of Motion) and geometric (Center of Mass) descriptors and Laban Effort descriptors (Weight, Time, Flow, Space).
- Juan Alonso Moreno: Python implementation that fetches movement data from tsv file, computes low-level kinematic descriptors (velocity, acceleration, jerk) that are compared interactively with measurements, then computes Laban Effort descriptors (Weight, Time, Flow, Space).

All implementations are based on the review contained in C. Larboulette and S. Gibet, "A review of computable expressive descriptors of human motion," in *Proceedings of the 2nd International Workshop on Movement and Computing*, 2015, pp. 21–28, doi: 10.1145/2790994.2790998.
