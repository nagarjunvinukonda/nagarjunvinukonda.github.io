# Portfolio animation sources

The portfolio uses short, silent MP4 loops for GIF-like playback. Company and framework reference footage is credited in the white project or experience card; these clips do not represent personal authorship of the depicted demonstrations.

| File | Source | Use |
| --- | --- | --- |
| tri.mp4 | https://toyotaresearchinstitute.github.io/lbm1/videos/bike.mp4 | 10–16 seconds; TRI company research context |
| neuro42.mp4 | https://www.neuro42.ai/product-page | 18–25 seconds of the official neuro42 product animation; MRI workflow and equipment positioning |
| forterra.mp4 | https://www.forterra.com/ | 2–8 seconds of the official Tested_1 vehicle footage |
| pick-place.mp4 | https://huggingface.co/blog/smolvla | Complete uncropped 816×360 task-variation montage; reference footage, not a personal policy rollout |
| policy-eval.mp4 | ../tools/render_portfolio_demos.py | Original scripted robot-arm pushing illustration; not a recorded SmolVLA rollout |
| dqn-obstacles.mp4, dqn-obstacles.gif | ../tools/turtlebot_navigation.py | Collision-checked TurtleBot-style navigation to a goal, moving discs with collision responses, a solid fixed cube, and LiDAR rays; illustrative planner simulation, not a recorded DQN/Gazebo rollout. Clearance checks are recorded in dqn-validation.json. |
| mimic.mp4 | https://mimicgen.github.io/resources/new_robots/panda.mp4 | Public Franka manipulation simulation; reference footage |
| bci.mp4 | ../images/BCI.gif | Original portfolio BCI project, compressed |
| turtlebot.mp4 | ../images/PID_control.gif | Original portfolio PID project; used only on the PID card |
| driving-split.svg | Generated in-repo | Illustrative synchronized front-camera + BEV round-road driving animation for the Behavior Cloning card; not a recorded CARLA rollout |

JPG files are the corresponding first-frame fallbacks. Silent autoplay loops respect reduced-motion preferences through scripts/portfolio-media.js. All media is contained within equal 16:10 frames, without cropping.
