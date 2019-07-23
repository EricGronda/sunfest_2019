# Real-time Event Based Catadioptric Stereo

## Abstract

The goal of this project is to develop a vision based stereo system that is capable of accurately measuring fast moving objects in the field of view. The approach that we are pursuing builds on two previously developed technologies: event based cameras and catadioptric stereo systems. Event based cameras track changes in light intensity using sensors that read data from each pixel asynchronously. These cameras provide high speed, low power, and a large dynamic range. However, these cameras are relatively new and expensive, so we propose a novel idea that merges the event camera with a catadioptric system which uses mirrors to create multiple virtual viewpoints, allowing for a single camera to mimic the effect of trinocular setup. A system that merges these two advances takes the benefits from both the reduced cost and complexity of a single camera and the strengths of event based vision. Finally, we propose a novel algorithm for stereo depth calculation using the timestamp and polarity of events to find the correspondence between the virtual viewpoints, which then allows for depth estimation. 

**For information on how to run this program, see** ```sunfest_2019/docs/usage.md```

