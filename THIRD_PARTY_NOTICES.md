# Third-Party Notices

This repository contains motion-planning algorithm implementations derived from
or based on examples from Atsushi Sakai's PythonRobotics project:

https://github.com/AtsushiSakai/PythonRobotics

PythonRobotics is released under the MIT License. Its original license notice is
included below.

The adapted planning modules include `global_planning/hybrid_a_star.py`
(header credits Zheng Zh), `global_planning/a_star.py` (Nikos Kanargias),
`global_planning/reeds_shepp_path_planning.py`, `local_planning/rrt.py`,
`local_planning/rrt_1.py`, `local_planning/rrt_reeds_shepp.py`, and
`path_interpolation/cubic_spline_planner.py` under `motion_planning/`.
`path_interpolation/cubic_spline_hybrid_a_star.py` also contains adapted spline
code. Existing upstream author headers are retained where present.

The thesis project integrates these components with parking-lot scenarios,
behavior decisions, velocity-profile handling, vehicle tracking, and simulation.
The algorithm implementations are not claimed as original thesis algorithms.

## PythonRobotics

The MIT License (MIT)

Copyright (c) 2016 - now Atsushi Sakai and other contributors:

https://github.com/AtsushiSakai/PythonRobotics/contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
