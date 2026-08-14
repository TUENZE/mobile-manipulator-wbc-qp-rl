# Weekly Progress Log

> Update this file every week. Add a new entry at the top for each week.
> This is the first thing reviewers check during progress review.

---

## Week 4 - 2026-07-01

**Attended this week's meeting:** Yes

**Progress this week**
- Tested UR5 arm motion through ROS 2, pymoveit2, and MoveIt 2.
- Documented the computation graph for the working arm-motion pipeline, including data types, units, and data directions.
- Added separate diagrams for the pose-goal path and the joint-goal path.

**Challenges & blockers**
- The current implementation uses MoveIt 2 planning and trajectory execution. The custom WBC/QP/RL controller is not implemented yet.

**Next steps**
- Continue from MoveIt-based motion verification toward whole-body-control, QP, or RL-based control experiments.
- Add experiment logs and screenshots/video evidence for successful arm motion.

**Links (optional):**
- Computation graph: `docs/ur5_moveit_computation_graph.md`

---

## Week 1 - 2026-06-09

**Attended this week's meeting:** Yes

**Progress this week**
- Set up one public GitHub repository for the full FURP project.
- Checked the repository against the official FURP template.
- Added required weekly log and meeting notes structure.
- Summarized the FURP rules and project requirements in `docs/project_requirements.md`.

**Challenges & blockers**
- The cited paper to replicate has not been finalized yet.
- The final `FURP_Showcase.pdf` poster is not available yet and will be added before the Showcase.

**Next steps**
- Confirm the exact research objective, baseline paper, and replication target.
- Add source code, simulation scripts, or experiment setup under `src/`.
- Continue weekly updates and meeting note records.

**Hours spent (optional):**

**Links (optional):**
- Repository: https://github.com/TUENZE/mobile-manipulator-wbc-qp-rl
