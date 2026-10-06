"""Tier-2 simulation: Gazebo Harmonic worlds and ray-cast perception (ADR-0003).

This subpackage is ROS-agnostic (AGENTS.md §3). It

* writes a Gazebo SDF world and a ``ros_gz_bridge`` configuration from a
  Tier-1 :class:`~avatar.sim.scenarios.Scenario` (:mod:`avatar.tier2.sdf`), so
  that both tiers share one scenario definition and seed;
* turns recorded range data (VLP-16 point clouds, D435i depth images, and a
  ray-cast imaging-sonar proxy) into landmark-part detections with a
  geometric front-end and a per-agent tracker (:mod:`avatar.tier2.frontend`);
* assembles a :class:`~avatar.sim.measurements.SimData` whose odometry,
  absolute height and ground truth are those of Tier 1 for the same seed, and
  whose detections come from Gazebo (:mod:`avatar.tier2.dataset`). Every
  estimation mode in :mod:`avatar.runner` then runs unchanged.

The recorder that drives Gazebo lives in ``experiments/gazebo/`` (it needs
ROS 2 and a running simulator).
"""
